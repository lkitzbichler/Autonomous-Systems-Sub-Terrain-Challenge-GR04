#include "pathplanner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/ros_conversions.h>
#include <mav_trajectory_generation/timing.h>
#include <octomap_msgs/conversions.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <sensor_msgs/image_encodings.hpp>

#include "protocol.hpp"

namespace {

namespace ob = ompl::base;
namespace og = ompl::geometric;

constexpr size_t kVisitedGoalsCap = 200;

geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d &p)
{
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

const std::array<Eigen::Vector3d, 18> kProbeDirections = {
  Eigen::Vector3d(1.0, 0.0, 0.0),
  Eigen::Vector3d(-1.0, 0.0, 0.0),
  Eigen::Vector3d(0.0, 1.0, 0.0),
  Eigen::Vector3d(0.0, -1.0, 0.0),
  Eigen::Vector3d(0.0, 0.0, 1.0),
  Eigen::Vector3d(0.0, 0.0, -1.0),
  Eigen::Vector3d(1.0, 1.0, 0.0).normalized(),
  Eigen::Vector3d(1.0, -1.0, 0.0).normalized(),
  Eigen::Vector3d(-1.0, 1.0, 0.0).normalized(),
  Eigen::Vector3d(-1.0, -1.0, 0.0).normalized(),
  Eigen::Vector3d(1.0, 0.0, 1.0).normalized(),
  Eigen::Vector3d(1.0, 0.0, -1.0).normalized(),
  Eigen::Vector3d(-1.0, 0.0, 1.0).normalized(),
  Eigen::Vector3d(-1.0, 0.0, -1.0).normalized(),
  Eigen::Vector3d(0.0, 1.0, 1.0).normalized(),
  Eigen::Vector3d(0.0, 1.0, -1.0).normalized(),
  Eigen::Vector3d(0.0, -1.0, 1.0).normalized(),
  Eigen::Vector3d(0.0, -1.0, -1.0).normalized(),
};

}  // namespace

PathPlannerNode::PathPlannerNode()
: rclcpp::Node("path_planner")
{
  // Step 1: Load parameters.
  declareAndLoadParameters();

  // Step 2: Create command/sensor subscriptions.
  command_sub_ = this->create_subscription<statemachine_pkg::msg::Command>(
    command_topic_, 10, std::bind(&PathPlannerNode::onCommand, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 10, std::bind(&PathPlannerNode::onOdometry, this, std::placeholders::_1));
  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, 1, std::bind(&PathPlannerNode::onOctomap, this, std::placeholders::_1));
  lantern_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    lantern_topic_, 10, std::bind(&PathPlannerNode::onLanternDetections, this, std::placeholders::_1));
  depth_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    depth_image_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PathPlannerNode::onDepthImage, this, std::placeholders::_1));
  depth_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    depth_camera_info_topic_, rclcpp::QoS(10),
    std::bind(&PathPlannerNode::onDepthCameraInfo, this, std::placeholders::_1));

  // Step 3: Create output publishers.
  heartbeat_pub_ = this->create_publisher<statemachine_pkg::msg::Answer>(heartbeat_topic_, 10);
  trajectory_pub_ =
    this->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(trajectory_topic_, 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);
  selected_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(selected_path_topic_, 10);

  // Step 4: Create periodic timers.
  planner_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(planner_period_sec_)),
    std::bind(&PathPlannerNode::onPlannerTimer, this));
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(heartbeat_period_sec_)),
    std::bind(&PathPlannerNode::onHeartbeatTimer, this));
  parameter_refresh_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(parameter_refresh_period_sec_)),
    std::bind(&PathPlannerNode::onParameterRefreshTimer, this));

  RCLCPP_INFO(this->get_logger(),
    "path_planner ready: cmd='%s', odom='%s', octomap='%s', traj='%s', marker='%s', selected_path='%s'",
    command_topic_.c_str(), odom_topic_.c_str(), octomap_topic_.c_str(), trajectory_topic_.c_str(),
    marker_topic_.c_str(), selected_path_topic_.c_str());
}

void PathPlannerNode::declareAndLoadParameters()
{
  // Step 1: Declare topic/timer parameters.
  command_topic_ = this->declare_parameter<std::string>("command_topic", "statemachine/cmd");
  heartbeat_topic_ = this->declare_parameter<std::string>("heartbeat_topic", "heartbeat");
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "current_state_est");
  octomap_topic_ = this->declare_parameter<std::string>("octomap_topic", "octomap_binary");
  trajectory_topic_ = this->declare_parameter<std::string>("trajectory_topic", "trajectory");
  lantern_topic_ = this->declare_parameter<std::string>("lantern_topic", "detected_lanterns");
  depth_image_topic_ =
    this->declare_parameter<std::string>("depth_image_topic", "/realsense/depth/image");
  depth_camera_info_topic_ = this->declare_parameter<std::string>(
    "depth_camera_info_topic", "/realsense/depth/camera_info");
  marker_topic_ = this->declare_parameter<std::string>("marker_topic", "path_planner/viz/markers");
  selected_path_topic_ =
    this->declare_parameter<std::string>("selected_path_topic", "path_planner/selected_path");
  viz_frame_id_ = this->declare_parameter<std::string>("viz_frame_id", "world");

  planner_period_sec_ = this->declare_parameter<double>("planner_period_sec", planner_period_sec_);
  heartbeat_period_sec_ =
    this->declare_parameter<double>("heartbeat_period_sec", heartbeat_period_sec_);
  parameter_refresh_period_sec_ = this->declare_parameter<double>(
    "parameter_refresh_period_sec", parameter_refresh_period_sec_);

  // Step 2: Declare dynamic planning parameters.
  max_v_ = this->declare_parameter<double>("max_v", max_v_);
  max_a_ = this->declare_parameter<double>("max_a", max_a_);
  waypoint_dist_min_ = this->declare_parameter<double>("waypoint_dist_min", waypoint_dist_min_);
  waypoint_dist_max_ = this->declare_parameter<double>("waypoint_dist_max", waypoint_dist_max_);
  wall_clearance_min_ = this->declare_parameter<double>("wall_clearance_min", wall_clearance_min_);
  path_check_step_m_ = this->declare_parameter<double>("path_check_step_m", path_check_step_m_);
  clearance_probe_radius_m_ =
    this->declare_parameter<double>("clearance_probe_radius_m", clearance_probe_radius_m_);
  goal_reached_radius_m_ =
    this->declare_parameter<double>("goal_reached_radius_m", goal_reached_radius_m_);
  replan_timeout_sec_ = this->declare_parameter<double>("replan_timeout_sec", replan_timeout_sec_);

  frontier_goal_dist_min_ =
    this->declare_parameter<double>("frontier_goal_dist_min", frontier_goal_dist_min_);
  frontier_goal_dist_max_ =
    this->declare_parameter<double>("frontier_goal_dist_max", frontier_goal_dist_max_);
  frontier_unknown_min_neighbors_ =
    this->declare_parameter<int>("frontier_unknown_min_neighbors", frontier_unknown_min_neighbors_);
  frontier_known_free_min_neighbors_ = this->declare_parameter<int>(
    "frontier_known_free_min_neighbors", frontier_known_free_min_neighbors_);
  max_frontier_fail_cycles_ =
    this->declare_parameter<int>("max_frontier_fail_cycles", max_frontier_fail_cycles_);

  ompl_timeout_sec_ = this->declare_parameter<double>("ompl_timeout_sec", ompl_timeout_sec_);
  ompl_range_m_ = this->declare_parameter<double>("ompl_range_m", ompl_range_m_);
  bounds_padding_m_ = this->declare_parameter<double>("bounds_padding_m", bounds_padding_m_);

  required_lantern_count_ =
    this->declare_parameter<int>("required_lantern_count", required_lantern_count_);
  lantern_merge_dist_m_ = this->declare_parameter<double>("lantern_merge_dist_m", lantern_merge_dist_m_);

  score_unknown_weight_ = this->declare_parameter<double>("score_unknown_weight", score_unknown_weight_);
  score_clearance_weight_ =
    this->declare_parameter<double>("score_clearance_weight", score_clearance_weight_);
  score_distance_weight_ =
    this->declare_parameter<double>("score_distance_weight", score_distance_weight_);
  score_revisit_weight_ = this->declare_parameter<double>("score_revisit_weight", score_revisit_weight_);
  score_forward_weight_ = this->declare_parameter<double>("score_forward_weight", score_forward_weight_);
  heading_hint_min_speed_ =
    this->declare_parameter<double>("heading_hint_min_speed", heading_hint_min_speed_);
  forward_min_dot_ = this->declare_parameter<double>("forward_min_dot", forward_min_dot_);
  adaptive_waypoint_step_enabled_ =
    this->declare_parameter<bool>("adaptive_waypoint_step_enabled", adaptive_waypoint_step_enabled_);
  adaptive_step_clearance_range_m_ = this->declare_parameter<double>(
    "adaptive_step_clearance_range_m", adaptive_step_clearance_range_m_);
  adaptive_step_turn_weight_ =
    this->declare_parameter<double>("adaptive_step_turn_weight", adaptive_step_turn_weight_);
  adaptive_step_min_ratio_ =
    this->declare_parameter<double>("adaptive_step_min_ratio", adaptive_step_min_ratio_);
  depth_free_min_m_ = this->declare_parameter<double>("depth_free_min_m", depth_free_min_m_);
  depth_max_m_ = this->declare_parameter<double>("depth_max_m", depth_max_m_);
  depth_min_component_pixels_ =
    this->declare_parameter<int>("depth_min_component_pixels", depth_min_component_pixels_);
  depth_hint_weight_ = this->declare_parameter<double>("depth_hint_weight", depth_hint_weight_);
  depth_hint_timeout_sec_ =
    this->declare_parameter<double>("depth_hint_timeout_sec", depth_hint_timeout_sec_);
  depth_hint_min_align_dot_ =
    this->declare_parameter<double>("depth_hint_min_align_dot", depth_hint_min_align_dot_);
  depth_body_yaw_correction_deg_ = this->declare_parameter<double>(
    "depth_body_yaw_correction_deg", depth_body_yaw_correction_deg_);
  depth_far_percentile_ = this->declare_parameter<double>("depth_far_percentile", depth_far_percentile_);
  depth_target_range_min_m_ =
    this->declare_parameter<double>("depth_target_range_min_m", depth_target_range_min_m_);
  map_escape_lookahead_m_ =
    this->declare_parameter<double>("map_escape_lookahead_m", map_escape_lookahead_m_);
  forward_smoothing_alpha_ =
    this->declare_parameter<double>("forward_smoothing_alpha", forward_smoothing_alpha_);
  escape_blend_weight_ =
    this->declare_parameter<double>("escape_blend_weight", escape_blend_weight_);
  centerline_horizon_points_ =
    this->declare_parameter<int>("centerline_horizon_points", centerline_horizon_points_);
  centerline_step_m_ = this->declare_parameter<double>("centerline_step_m", centerline_step_m_);
  centerline_slice_half_extent_m_ = this->declare_parameter<double>(
    "centerline_slice_half_extent_m", centerline_slice_half_extent_m_);
  centerline_slice_step_m_ =
    this->declare_parameter<double>("centerline_slice_step_m", centerline_slice_step_m_);
  centerline_min_component_cells_ = this->declare_parameter<int>(
    "centerline_min_component_cells", centerline_min_component_cells_);
  centerline_max_secondary_goals_ = this->declare_parameter<int>(
    "centerline_max_secondary_goals", centerline_max_secondary_goals_);
  centerline_execute_dist_m_ = this->declare_parameter<double>(
    "centerline_execute_dist_m", centerline_execute_dist_m_);
  centerline_min_commit_dist_m_ = this->declare_parameter<double>(
    "centerline_min_commit_dist_m", centerline_min_commit_dist_m_);
  marker_max_candidates_ = this->declare_parameter<int>("marker_max_candidates", marker_max_candidates_);

  // Step 3: Clamp to robust ranges.
  refreshDynamicParameters();
}

void PathPlannerNode::refreshDynamicParameters()
{
  // Step 1: Reload values from parameter server.
  (void)this->get_parameter("max_v", max_v_);
  (void)this->get_parameter("max_a", max_a_);
  (void)this->get_parameter("waypoint_dist_min", waypoint_dist_min_);
  (void)this->get_parameter("waypoint_dist_max", waypoint_dist_max_);
  (void)this->get_parameter("wall_clearance_min", wall_clearance_min_);
  (void)this->get_parameter("path_check_step_m", path_check_step_m_);
  (void)this->get_parameter("clearance_probe_radius_m", clearance_probe_radius_m_);
  (void)this->get_parameter("goal_reached_radius_m", goal_reached_radius_m_);
  (void)this->get_parameter("replan_timeout_sec", replan_timeout_sec_);
  (void)this->get_parameter("frontier_goal_dist_min", frontier_goal_dist_min_);
  (void)this->get_parameter("frontier_goal_dist_max", frontier_goal_dist_max_);
  (void)this->get_parameter("frontier_unknown_min_neighbors", frontier_unknown_min_neighbors_);
  (void)this->get_parameter("frontier_known_free_min_neighbors", frontier_known_free_min_neighbors_);
  (void)this->get_parameter("max_frontier_fail_cycles", max_frontier_fail_cycles_);
  (void)this->get_parameter("ompl_timeout_sec", ompl_timeout_sec_);
  (void)this->get_parameter("ompl_range_m", ompl_range_m_);
  (void)this->get_parameter("bounds_padding_m", bounds_padding_m_);
  (void)this->get_parameter("required_lantern_count", required_lantern_count_);
  (void)this->get_parameter("lantern_merge_dist_m", lantern_merge_dist_m_);
  (void)this->get_parameter("score_unknown_weight", score_unknown_weight_);
  (void)this->get_parameter("score_clearance_weight", score_clearance_weight_);
  (void)this->get_parameter("score_distance_weight", score_distance_weight_);
  (void)this->get_parameter("score_revisit_weight", score_revisit_weight_);
  (void)this->get_parameter("score_forward_weight", score_forward_weight_);
  (void)this->get_parameter("heading_hint_min_speed", heading_hint_min_speed_);
  (void)this->get_parameter("forward_min_dot", forward_min_dot_);
  (void)this->get_parameter("adaptive_waypoint_step_enabled", adaptive_waypoint_step_enabled_);
  (void)this->get_parameter("adaptive_step_clearance_range_m", adaptive_step_clearance_range_m_);
  (void)this->get_parameter("adaptive_step_turn_weight", adaptive_step_turn_weight_);
  (void)this->get_parameter("adaptive_step_min_ratio", adaptive_step_min_ratio_);
  (void)this->get_parameter("depth_free_min_m", depth_free_min_m_);
  (void)this->get_parameter("depth_max_m", depth_max_m_);
  (void)this->get_parameter("depth_min_component_pixels", depth_min_component_pixels_);
  (void)this->get_parameter("depth_hint_weight", depth_hint_weight_);
  (void)this->get_parameter("depth_hint_timeout_sec", depth_hint_timeout_sec_);
  (void)this->get_parameter("depth_hint_min_align_dot", depth_hint_min_align_dot_);
  (void)this->get_parameter("depth_body_yaw_correction_deg", depth_body_yaw_correction_deg_);
  (void)this->get_parameter("depth_far_percentile", depth_far_percentile_);
  (void)this->get_parameter("depth_target_range_min_m", depth_target_range_min_m_);
  (void)this->get_parameter("map_escape_lookahead_m", map_escape_lookahead_m_);
  (void)this->get_parameter("forward_smoothing_alpha", forward_smoothing_alpha_);
  (void)this->get_parameter("escape_blend_weight", escape_blend_weight_);
  (void)this->get_parameter("centerline_horizon_points", centerline_horizon_points_);
  (void)this->get_parameter("centerline_step_m", centerline_step_m_);
  (void)this->get_parameter("centerline_slice_half_extent_m", centerline_slice_half_extent_m_);
  (void)this->get_parameter("centerline_slice_step_m", centerline_slice_step_m_);
  (void)this->get_parameter("centerline_min_component_cells", centerline_min_component_cells_);
  (void)this->get_parameter("centerline_max_secondary_goals", centerline_max_secondary_goals_);
  (void)this->get_parameter("centerline_execute_dist_m", centerline_execute_dist_m_);
  (void)this->get_parameter("centerline_min_commit_dist_m", centerline_min_commit_dist_m_);
  (void)this->get_parameter("marker_max_candidates", marker_max_candidates_);

  // Step 2: Clamp to safe ranges.
  max_v_ = std::max(0.2, max_v_);
  max_a_ = std::max(0.2, max_a_);
  waypoint_dist_min_ = std::max(0.2, waypoint_dist_min_);
  waypoint_dist_max_ = std::max(waypoint_dist_min_ + 0.1, waypoint_dist_max_);
  wall_clearance_min_ = std::max(0.2, wall_clearance_min_);
  path_check_step_m_ = std::max(0.05, path_check_step_m_);
  clearance_probe_radius_m_ = std::max(wall_clearance_min_, clearance_probe_radius_m_);
  goal_reached_radius_m_ = std::max(0.2, goal_reached_radius_m_);
  replan_timeout_sec_ = std::max(0.5, replan_timeout_sec_);
  frontier_goal_dist_min_ = std::max(0.5, frontier_goal_dist_min_);
  frontier_goal_dist_max_ = std::max(frontier_goal_dist_min_ + 0.2, frontier_goal_dist_max_);
  frontier_unknown_min_neighbors_ = std::max(1, frontier_unknown_min_neighbors_);
  frontier_known_free_min_neighbors_ = std::clamp(frontier_known_free_min_neighbors_, 1, 26);
  max_frontier_fail_cycles_ = std::max(1, max_frontier_fail_cycles_);
  ompl_timeout_sec_ = std::max(0.02, ompl_timeout_sec_);
  ompl_range_m_ = std::max(0.2, ompl_range_m_);
  bounds_padding_m_ = std::max(0.2, bounds_padding_m_);
  required_lantern_count_ = std::max(1, required_lantern_count_);
  lantern_merge_dist_m_ = std::max(0.1, lantern_merge_dist_m_);
  heading_hint_min_speed_ = std::max(0.01, heading_hint_min_speed_);
  forward_min_dot_ = std::clamp(forward_min_dot_, -1.0, 1.0);
  adaptive_step_clearance_range_m_ = std::max(0.2, adaptive_step_clearance_range_m_);
  adaptive_step_turn_weight_ = std::clamp(adaptive_step_turn_weight_, 0.0, 1.0);
  adaptive_step_min_ratio_ = std::clamp(adaptive_step_min_ratio_, 0.1, 1.0);
  depth_free_min_m_ = std::max(0.2, depth_free_min_m_);
  depth_max_m_ = std::max(depth_free_min_m_ + 0.5, depth_max_m_);
  depth_min_component_pixels_ = std::max(20, depth_min_component_pixels_);
  depth_hint_weight_ = std::clamp(depth_hint_weight_, 0.0, 1.0);
  depth_hint_timeout_sec_ = std::clamp(depth_hint_timeout_sec_, 0.1, 3.0);
  depth_hint_min_align_dot_ = std::clamp(depth_hint_min_align_dot_, -1.0, 0.95);
  depth_body_yaw_correction_deg_ = std::clamp(depth_body_yaw_correction_deg_, -180.0, 180.0);
  depth_far_percentile_ = std::clamp(depth_far_percentile_, 0.35, 0.98);
  depth_target_range_min_m_ = std::max(depth_free_min_m_ + 0.3, depth_target_range_min_m_);
  map_escape_lookahead_m_ = std::clamp(map_escape_lookahead_m_, 2.0, 20.0);
  forward_smoothing_alpha_ = std::clamp(forward_smoothing_alpha_, 0.0, 0.98);
  escape_blend_weight_ = std::clamp(escape_blend_weight_, 0.05, 0.95);
  centerline_horizon_points_ = std::clamp(centerline_horizon_points_, 3, 24);
  centerline_step_m_ = std::clamp(centerline_step_m_, 0.5, 10.0);
  centerline_slice_half_extent_m_ = std::clamp(centerline_slice_half_extent_m_, 1.0, 12.0);
  centerline_slice_step_m_ = std::clamp(centerline_slice_step_m_, 0.2, 2.0);
  centerline_min_component_cells_ = std::clamp(centerline_min_component_cells_, 1, 500);
  centerline_max_secondary_goals_ = std::clamp(centerline_max_secondary_goals_, 0, 12);
  centerline_execute_dist_m_ = std::clamp(centerline_execute_dist_m_, 2.0, 60.0);
  centerline_min_commit_dist_m_ = std::clamp(
    centerline_min_commit_dist_m_, 1.0, std::max(1.0, centerline_execute_dist_m_ - 0.5));
  marker_max_candidates_ = std::max(10, marker_max_candidates_);
}

void PathPlannerNode::onCommand(const statemachine_pkg::msg::Command::SharedPtr msg)
{
  // Step 1: Validate message and target.
  if (!msg) {
    return;
  }
  if (msg->target != this->get_name()) {
    return;
  }

  // Step 2: Apply mission command.
  switch (msg->command) {
    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::START):
      // START is sent cyclically by statemachine: avoid resetting active exploration on each message.
      if (mode_ != PlannerMode::EXPLORE || planner_done_) {
        mode_ = PlannerMode::EXPLORE;
        planner_done_ = false;
        has_current_goal_ = false;
        has_smoothed_forward_hint_ = false;
        frontier_fail_cycles_ = 0;
        debug_candidate_points_.clear();
        debug_centerline_points_.clear();
        RCLCPP_INFO(this->get_logger(), "[cmd] START -> EXPLORE");
      }
      break;

    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::RETURN_HOME):
    {
      const bool mode_changed = (mode_ != PlannerMode::RETURN_HOME);
      if (mode_changed) {
        mode_ = PlannerMode::RETURN_HOME;
        planner_done_ = false;
        has_current_goal_ = false;
        has_smoothed_forward_hint_ = false;
        frontier_fail_cycles_ = 0;
        debug_candidate_points_.clear();
        debug_centerline_points_.clear();
      }
      if (msg->has_target) {
        home_position_ = Eigen::Vector3d(msg->target_pos.x, msg->target_pos.y, msg->target_pos.z);
        has_home_pose_ = true;
      }
      if (mode_changed || msg->has_target) {
        RCLCPP_INFO(this->get_logger(), "[cmd] RETURN_HOME");
      }
      break;
    }

    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::HOLD):
    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::ABORT):
    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::LAND):
      if (mode_ != PlannerMode::IDLE || has_current_goal_) {
        mode_ = PlannerMode::IDLE;
        has_current_goal_ = false;
        has_smoothed_forward_hint_ = false;
        RCLCPP_INFO(this->get_logger(), "[cmd] HOLD/ABORT/LAND -> IDLE");
      }
      break;

    default:
      RCLCPP_INFO(this->get_logger(), "[cmd] ignored id=%u", msg->command);
      break;
  }
}

void PathPlannerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Step 1: Validate input.
  if (!msg) {
    return;
  }

  // Step 2: Cache current pose/velocity.
  current_position_.x() = msg->pose.pose.position.x;
  current_position_.y() = msg->pose.pose.position.y;
  current_position_.z() = msg->pose.pose.position.z;

  current_velocity_.x() = msg->twist.twist.linear.x;
  current_velocity_.y() = msg->twist.twist.linear.y;
  current_velocity_.z() = msg->twist.twist.linear.z;

  const auto &q_msg = msg->pose.pose.orientation;
  Eigen::Quaterniond q_body_to_world(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (std::isfinite(q_body_to_world.w()) && std::isfinite(q_body_to_world.x()) &&
      std::isfinite(q_body_to_world.y()) && std::isfinite(q_body_to_world.z()) &&
      (q_body_to_world.norm() > 1e-6))
  {
    q_body_to_world.normalize();

    const Eigen::Vector3d fwd = q_body_to_world * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d left = q_body_to_world * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d up = q_body_to_world * Eigen::Vector3d::UnitZ();

    Eigen::Vector3d fwd_corr = fwd;
    Eigen::Vector3d left_corr = left;
    Eigen::Vector3d up_corr = up;
    if (std::abs(depth_body_yaw_correction_deg_) > 1e-3) {
      constexpr double kPi = 3.14159265358979323846;
      const double yaw_rad = depth_body_yaw_correction_deg_ * (kPi / 180.0);
      const Eigen::AngleAxisd yaw_corr(yaw_rad, Eigen::Vector3d::UnitZ());
      fwd_corr = yaw_corr * fwd;
      left_corr = yaw_corr * left;
      up_corr = yaw_corr * up;
    }

    if (fwd_corr.norm() > 1e-3 && left_corr.norm() > 1e-3 && up_corr.norm() > 1e-3) {
      body_forward_hint_ = fwd_corr.normalized();
      body_left_hint_ = left_corr.normalized();
      body_up_hint_ = up_corr.normalized();
    }
  }

  const double vxy = std::hypot(current_velocity_.x(), current_velocity_.y());
  if (vxy >= heading_hint_min_speed_) {
    heading_hint_ =
      Eigen::Vector3d(current_velocity_.x() / vxy, current_velocity_.y() / vxy, 0.0);
  }

  has_current_pose_ = true;

  // Step 3: Cache first valid position as return-home anchor.
  if (!has_home_pose_) {
    home_position_ = current_position_;
    has_home_pose_ = true;
  }
}

void PathPlannerNode::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
  // Step 1: Validate input.
  if (!msg) {
    return;
  }
  if (!msg->binary) {
    RCLCPP_WARN(this->get_logger(), "Received non-binary octomap on '%s'.", octomap_topic_.c_str());
    return;
  }

  // Step 2: Convert message into an OcTree.
  octomap::AbstractOcTree *tree_raw = octomap_msgs::binaryMsgToMap(*msg);
  if (!tree_raw) {
    RCLCPP_WARN(this->get_logger(), "Failed to decode octomap message.");
    return;
  }

  auto *tree = dynamic_cast<octomap::OcTree *>(tree_raw);
  if (!tree) {
    RCLCPP_WARN(this->get_logger(), "Decoded map is not an octomap::OcTree.");
    delete tree_raw;
    return;
  }

  // Step 3: Store latest map.
  octree_.reset(tree);
  has_octomap_ = true;
}

void PathPlannerNode::onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  // Step 1: Validate input.
  if (!msg || msg->poses.empty()) {
    return;
  }

  // Step 2: Merge detections by distance threshold.
  for (const auto &pose : msg->poses) {
    const Eigen::Vector3d p(pose.position.x, pose.position.y, pose.position.z);

    bool already_tracked = false;
    for (const auto &tracked : tracked_lanterns_) {
      if (distance3(tracked, p) <= lantern_merge_dist_m_) {
        already_tracked = true;
        break;
      }
    }

    if (!already_tracked) {
      tracked_lanterns_.push_back(p);
    }
  }
}

void PathPlannerNode::onDepthCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  // Step 1: Validate input.
  if (!msg || msg->k.size() < 9) {
    return;
  }

  // Step 2: Cache intrinsic matrix.
  const double fx = msg->k[0];
  const double fy = msg->k[4];
  const double cx = msg->k[2];
  const double cy = msg->k[5];
  if (!(std::isfinite(fx) && std::isfinite(fy) && std::isfinite(cx) && std::isfinite(cy))) {
    return;
  }
  if (fx <= 1e-6 || fy <= 1e-6) {
    return;
  }

  depth_fx_ = fx;
  depth_fy_ = fy;
  depth_cx_ = cx;
  depth_cy_ = cy;
  has_depth_intrinsics_ = true;

  if (msg->width > 0U && msg->height > 0U) {
    const int w = static_cast<int>(msg->width);
    const int h = static_cast<int>(msg->height);
    depth_anchor_u_ = std::clamp(static_cast<int>(std::lround(cx)), 0, w - 1);
    depth_anchor_v_ = std::clamp(static_cast<int>(std::lround(cy)), 0, h - 1);
  }
}

bool PathPlannerNode::decodeDepthMeters(
  const sensor_msgs::msg::Image &img,
  int u,
  int v,
  float &depth_m_out) const
{
  depth_m_out = 0.0F;
  if (u < 0 || v < 0) {
    return false;
  }
  const int width = static_cast<int>(img.width);
  const int height = static_cast<int>(img.height);
  if (u >= width || v >= height) {
    return false;
  }
  if (img.step == 0U || img.data.empty()) {
    return false;
  }

  const size_t row_off = static_cast<size_t>(v) * img.step;
  if (row_off >= img.data.size()) {
    return false;
  }
  const uint8_t *row = img.data.data() + row_off;

  if (img.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      img.encoding == sensor_msgs::image_encodings::MONO16)
  {
    const size_t px_off = static_cast<size_t>(u) * sizeof(uint16_t);
    if (row_off + px_off + sizeof(uint16_t) > img.data.size()) {
      return false;
    }
    uint16_t raw_mm = 0U;
    std::memcpy(&raw_mm, row + px_off, sizeof(uint16_t));
    if (raw_mm == 0U) {
      return false;
    }
    depth_m_out = static_cast<float>(raw_mm) * 0.001F;
    return std::isfinite(depth_m_out) && depth_m_out > 0.0F;
  }

  if (img.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    const size_t px_off = static_cast<size_t>(u) * sizeof(float);
    if (row_off + px_off + sizeof(float) > img.data.size()) {
      return false;
    }
    float raw_m = 0.0F;
    std::memcpy(&raw_m, row + px_off, sizeof(float));
    if (!std::isfinite(raw_m) || raw_m <= 0.0F) {
      return false;
    }
    depth_m_out = raw_m;
    return true;
  }

  return false;
}

void PathPlannerNode::onDepthImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Step 1: Validate input and intrinsics.
  if (!msg || !has_depth_intrinsics_ || msg->width == 0U || msg->height == 0U) {
    return;
  }

  const int width = static_cast<int>(msg->width);
  const int height = static_cast<int>(msg->height);
  if (depth_anchor_u_ < 0 || depth_anchor_v_ < 0) {
    depth_anchor_u_ = std::clamp(static_cast<int>(std::lround(depth_cx_)), 0, width - 1);
    depth_anchor_v_ = std::clamp(static_cast<int>(std::lround(depth_cy_)), 0, height - 1);
  }

  // Step 2: Build a low-resolution free-space mask from depth.
  constexpr int kStride = 3;
  const int w = (width + kStride - 1) / kStride;
  const int h = (height + kStride - 1) / kStride;
  if (w <= 0 || h <= 0) {
    return;
  }

  const auto idx_of = [w](int x, int y) {
    return y * w + x;
  };

  std::vector<uint8_t> valid_mask(static_cast<size_t>(w * h), 0U);
  std::vector<float> depth_samples(static_cast<size_t>(w * h), 0.0F);
  std::vector<float> valid_depths;
  valid_depths.reserve(static_cast<size_t>(w * h));
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int u = std::min(width - 1, x * kStride);
      const int v = std::min(height - 1, y * kStride);
      float d_m = 0.0F;
      if (!decodeDepthMeters(*msg, u, v, d_m)) {
        continue;
      }
      if (d_m < static_cast<float>(depth_free_min_m_) || d_m > static_cast<float>(depth_max_m_)) {
        continue;
      }
      const int idx = idx_of(x, y);
      valid_mask[static_cast<size_t>(idx)] = 1U;
      depth_samples[static_cast<size_t>(idx)] = d_m;
      valid_depths.push_back(d_m);
    }
  }

  if (valid_depths.size() < static_cast<size_t>(std::max(30, depth_min_component_pixels_ / 3))) {
    has_depth_hint_ = false;
    depth_wall_blocked_ = true;
    return;
  }

  const int anchor_x = std::clamp(depth_anchor_u_ / kStride, 0, w - 1);
  const int anchor_y = std::clamp(depth_anchor_v_ / kStride, 0, h - 1);
  const int anchor_idx = idx_of(anchor_x, anchor_y);
  const int min_pixels = std::max(8, depth_min_component_pixels_ / (kStride * kStride));

  // Step 3: Build far-depth mask to approximate the grayscale escape opening.
  const size_t q_index = std::clamp(
    static_cast<size_t>(depth_far_percentile_ * static_cast<double>(valid_depths.size() - 1)),
    static_cast<size_t>(0),
    valid_depths.size() - 1);
  std::nth_element(
    valid_depths.begin(),
    valid_depths.begin() + static_cast<std::ptrdiff_t>(q_index),
    valid_depths.end());
  float far_threshold_m = valid_depths[q_index];
  far_threshold_m = std::clamp(
    far_threshold_m,
    static_cast<float>(depth_target_range_min_m_),
    static_cast<float>(depth_max_m_));

  std::vector<uint8_t> target_mask(static_cast<size_t>(w * h), 0U);
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int idx = idx_of(x, y);
      if (valid_mask[static_cast<size_t>(idx)] == 0U) {
        continue;
      }
      if (depth_samples[static_cast<size_t>(idx)] >= far_threshold_m) {
        target_mask[static_cast<size_t>(idx)] = 1U;
      }
    }
  }

  double center_depth_sum = 0.0;
  int center_depth_count = 0;
  const int center_win = std::max(1, 4 / kStride);
  for (int dy = -center_win; dy <= center_win; ++dy) {
    for (int dx = -center_win; dx <= center_win; ++dx) {
      const int x = anchor_x + dx;
      const int y = anchor_y + dy;
      if (x < 0 || x >= w || y < 0 || y >= h) {
        continue;
      }
      const int idx = idx_of(x, y);
      if (valid_mask[static_cast<size_t>(idx)] == 0U) {
        continue;
      }
      center_depth_sum += static_cast<double>(depth_samples[static_cast<size_t>(idx)]);
      center_depth_count++;
    }
  }
  const bool has_center_depth = (center_depth_count > 0);
  const double center_depth_m = has_center_depth
    ? (center_depth_sum / static_cast<double>(center_depth_count))
    : 0.0;
  depth_wall_blocked_ = (!has_center_depth) || (center_depth_m < depth_target_range_min_m_);

  // Step 4: Extract connected target components and choose best opening.
  struct Component
  {
    int count{0};
    double sum_x{0.0};
    double sum_y{0.0};
    double sum_d{0.0};
    bool has_anchor{false};
  };

  std::vector<uint8_t> visited(static_cast<size_t>(w * h), 0U);
  std::vector<Component> components;
  components.reserve(16);

  for (int y0 = 0; y0 < h; ++y0) {
    for (int x0 = 0; x0 < w; ++x0) {
      const int start_idx = idx_of(x0, y0);
      if (target_mask[static_cast<size_t>(start_idx)] == 0U ||
          visited[static_cast<size_t>(start_idx)] != 0U)
      {
        continue;
      }

      Component comp;
      std::queue<std::pair<int, int>> queue;
      queue.emplace(x0, y0);
      visited[static_cast<size_t>(start_idx)] = 1U;

      while (!queue.empty()) {
        const auto [x, y] = queue.front();
        queue.pop();
        const int idx = idx_of(x, y);
        comp.count++;
        comp.sum_x += static_cast<double>(x);
        comp.sum_y += static_cast<double>(y);
        comp.sum_d += static_cast<double>(depth_samples[static_cast<size_t>(idx)]);
        if (idx == anchor_idx) {
          comp.has_anchor = true;
        }

        const std::array<std::pair<int, int>, 4> offsets = {
          std::make_pair(1, 0),
          std::make_pair(-1, 0),
          std::make_pair(0, 1),
          std::make_pair(0, -1),
        };
        for (const auto &[dx, dy] : offsets) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= w || ny < 0 || ny >= h) {
            continue;
          }
          const int nidx = idx_of(nx, ny);
          if (target_mask[static_cast<size_t>(nidx)] == 0U ||
              visited[static_cast<size_t>(nidx)] != 0U)
          {
            continue;
          }
          visited[static_cast<size_t>(nidx)] = 1U;
          queue.emplace(nx, ny);
        }
      }

      if (comp.count >= min_pixels) {
        components.push_back(comp);
      }
    }
  }

  if (components.empty()) {
    has_depth_hint_ = false;
    depth_wall_blocked_ = true;
    return;
  }

  const Component *selected_comp = nullptr;
  double best_score = -std::numeric_limits<double>::infinity();
  for (const auto &comp : components) {
    const double cx = comp.sum_x / static_cast<double>(comp.count);
    const double cy = comp.sum_y / static_cast<double>(comp.count);
    const double mean_d = comp.sum_d / static_cast<double>(comp.count);
    const double dx = cx - static_cast<double>(anchor_x);
    const double dy = cy - static_cast<double>(anchor_y);
    const double dist_center = std::hypot(dx, dy);

    // Why: Prefer large/far components while biasing toward centered tunnel opening.
    double score = 0.0;
    score += 1.2 * mean_d;
    score += 0.010 * static_cast<double>(comp.count);
    score -= 0.030 * dist_center;
    if (comp.has_anchor) {
      score += 0.6;
    }

    if (score > best_score) {
      best_score = score;
      selected_comp = &comp;
    }
  }
  if (!selected_comp || selected_comp->count <= 0) {
    has_depth_hint_ = false;
    return;
  }

  // Step 5: Convert selected component centroid into a world-space direction and aim point.
  const double mean_x = selected_comp->sum_x / static_cast<double>(selected_comp->count);
  const double mean_y = selected_comp->sum_y / static_cast<double>(selected_comp->count);
  const int u = std::clamp(static_cast<int>(std::lround(mean_x * kStride)), 0, width - 1);
  const int v = std::clamp(static_cast<int>(std::lround(mean_y * kStride)), 0, height - 1);
  const double mean_depth_m = selected_comp->sum_d / static_cast<double>(selected_comp->count);
  if (!std::isfinite(mean_depth_m) || mean_depth_m < depth_target_range_min_m_) {
    has_depth_hint_ = false;
    depth_wall_blocked_ = true;
    return;
  }

  const double cam_x = (static_cast<double>(u) - depth_cx_) / depth_fx_;
  const double cam_y = (static_cast<double>(v) - depth_cy_) / depth_fy_;
  Eigen::Vector3d ray_optical(cam_x, cam_y, 1.0);
  if (ray_optical.norm() < 1e-6) {
    has_depth_hint_ = false;
    return;
  }
  ray_optical.normalize();

  // Optical frame (z forward, x right, y down) -> body frame (x forward, y left, z up).
  Eigen::Vector3d ray_body(ray_optical.z(), -ray_optical.x(), -ray_optical.y());
  if (ray_body.norm() < 1e-6) {
    has_depth_hint_ = false;
    return;
  }
  ray_body.normalize();

  Eigen::Vector3d ray_world =
    ray_body.x() * body_forward_hint_ + ray_body.y() * body_left_hint_ + ray_body.z() * body_up_hint_;
  ray_world.z() = 0.0;
  if (ray_world.norm() < 1e-3) {
    has_depth_hint_ = false;
    return;
  }

  depth_forward_hint_ = ray_world.normalized();
  has_depth_hint_ = true;
  last_depth_hint_time_ = this->now();
  depth_aim_point_world_ = current_position_ +
    std::clamp(mean_depth_m, depth_target_range_min_m_, depth_max_m_) * depth_forward_hint_;
  depth_wall_blocked_ = false;
}

bool PathPlannerNode::chooseMapEscapeDirection(
  const Eigen::Vector3d &preferred_xy,
  Eigen::Vector3d &direction_xy_out) const
{
  direction_xy_out = Eigen::Vector3d::Zero();
  if (!octree_) {
    return false;
  }

  Eigen::Vector3d preferred = preferred_xy;
  preferred.z() = 0.0;
  if (preferred.norm() < 1e-3) {
    preferred = body_forward_hint_;
    preferred.z() = 0.0;
  }
  if (preferred.norm() < 1e-3) {
    preferred = Eigen::Vector3d::UnitX();
  }
  preferred.normalize();

  const double lookahead = std::max(map_escape_lookahead_m_, wall_clearance_min_ + 1.0);
  const std::array<double, 9> yaw_offsets_deg = {
    0.0, 25.0, -25.0, 50.0, -50.0, 75.0, -75.0, 100.0, -100.0};
  constexpr double kPi = 3.14159265358979323846;

  double best_score = -std::numeric_limits<double>::infinity();
  Eigen::Vector3d best_dir = Eigen::Vector3d::Zero();
  for (const double yaw_offset_deg : yaw_offsets_deg) {
    const double yaw_rad = yaw_offset_deg * (kPi / 180.0);
    const Eigen::AngleAxisd rot(yaw_rad, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d dir = rot * preferred;
    dir.z() = 0.0;
    if (dir.norm() < 1e-3) {
      continue;
    }
    dir.normalize();

    const Eigen::Vector3d probe = current_position_ + lookahead * dir;
    if (!isSegmentSafe(current_position_, probe)) {
      continue;
    }
    const double clr = estimateClearance(probe, clearance_probe_radius_m_);
    if (clr + 1e-3 < wall_clearance_min_) {
      continue;
    }

    const double align = dir.dot(preferred);
    const double unknown = static_cast<double>(countUnknownNeighbors(probe));
    const double score = 1.3 * clr + 0.9 * align + 0.08 * unknown;
    if (score > best_score) {
      best_score = score;
      best_dir = dir;
    }
  }

  if (best_dir.norm() < 1e-3) {
    return false;
  }
  direction_xy_out = best_dir.normalized();
  return true;
}

void PathPlannerNode::onPlannerTimer()
{
  // Step 1: Exit early when planner is inactive or done.
  if (mode_ == PlannerMode::IDLE || planner_done_) {
    return;
  }

  // Step 2: Require current pose + map.
  if (!hasValidMapAndPose()) {
    return;
  }

  // Step 3: Mark current goal as visited when reached.
  if (has_current_goal_ && hasReachedGoal()) {
    storeVisitedGoal(current_goal_);
    has_current_goal_ = false;
  }

  // Step 4: Respect replan timeout to avoid publish spam.
  const auto now = this->now();
  const bool timed_out =
    (last_plan_time_.nanoseconds() > 0) && ((now - last_plan_time_).seconds() >= replan_timeout_sec_);

  if (has_current_goal_ && !timed_out) {
    return;
  }

  // Step 5: Choose next goal.
  Eigen::Vector3d goal = current_position_;
  std::string reason;
  if (!chooseGoal(goal, reason)) {
    frontier_fail_cycles_++;

    if (shouldReportDone()) {
      planner_done_ = true;
      mode_ = PlannerMode::IDLE;
      has_current_goal_ = false;
      RCLCPP_INFO(this->get_logger(),
        "exploration done: frontier exhausted and %zu/%d lanterns tracked",
        tracked_lanterns_.size(), required_lantern_count_);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "No valid exploration goal found yet (fail_cycles=%d, lanterns=%zu/%d).",
        frontier_fail_cycles_, tracked_lanterns_.size(), required_lantern_count_);
    }
    return;
  }

  // Step 6: Plan geometric path with OMPL.
  std::vector<Eigen::Vector3d> path;
  if (!planPathOmpl(current_position_, goal, path)) {
    frontier_fail_cycles_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "OMPL planning failed for selected goal (fail_cycles=%d).", frontier_fail_cycles_);
    return;
  }

  // Step 7: Convert geometric path to compact waypoint chain.
  std::vector<Eigen::Vector3d> waypoints;
  if (!resamplePath(path, waypoints)) {
    frontier_fail_cycles_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Path resampling failed (fail_cycles=%d).", frontier_fail_cycles_);
    return;
  }

  // Step 8: Generate and publish polynomial trajectory.
  if (!buildAndPublishTrajectory(waypoints, true)) {
    frontier_fail_cycles_++;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Trajectory generation failed (fail_cycles=%d).", frontier_fail_cycles_);
    return;
  }

  // Step 9: Publish selected path/debug markers.
  publishSelectedPath(path);
  publishPlanningVisualization(path, waypoints);

  // Step 10: Store planning state.
  current_goal_ = goal;
  has_current_goal_ = true;
  frontier_fail_cycles_ = 0;
  last_plan_time_ = now;

  RCLCPP_INFO(this->get_logger(),
    "Trajectory published: mode=%u reason=%s goal=(%.2f, %.2f, %.2f) waypoints=%zu",
    static_cast<unsigned>(mode_), reason.c_str(), goal.x(), goal.y(), goal.z(), waypoints.size());
}

void PathPlannerNode::onHeartbeatTimer()
{
  // Step 1: Build heartbeat state.
  if (planner_done_) {
    publishHeartbeat(static_cast<uint8_t>(statemachine_pkg::protocol::AnswerStates::DONE),
      "DONE_FRONTIER_EXHAUSTED");
    return;
  }

  std::string info = "RUNNING_IDLE";
  if (mode_ == PlannerMode::EXPLORE) {
    info = has_current_goal_ ? "RUNNING_EXPLORE_ACTIVE" : "RUNNING_EXPLORE_WAIT";
  } else if (mode_ == PlannerMode::RETURN_HOME) {
    info = has_current_goal_ ? "RUNNING_RETURN_ACTIVE" : "RUNNING_RETURN_WAIT";
  }

  publishHeartbeat(static_cast<uint8_t>(statemachine_pkg::protocol::AnswerStates::RUNNING), info);
}

void PathPlannerNode::onParameterRefreshTimer()
{
  // Step 1: Re-load runtime-tunable parameters.
  refreshDynamicParameters();
}

bool PathPlannerNode::chooseGoal(Eigen::Vector3d &goal_out, std::string &reason_out)
{
  if (mode_ == PlannerMode::RETURN_HOME) {
    return chooseReturnGoal(goal_out, reason_out);
  }

  if (mode_ != PlannerMode::EXPLORE) {
    return false;
  }

  // Step 1: Prefer connected slice-centerline planning inside the tunnel.
  if (chooseCenterlineGoal(goal_out, reason_out)) {
    return true;
  }

  // Step 2: Fallback to true frontier search.
  if (chooseExploreGoal(goal_out, reason_out)) {
    return true;
  }

  // Step 3: Last fallback to free-space sweep.
  return chooseFallbackGoal(goal_out, reason_out);
}

bool PathPlannerNode::computeForwardDirection(Eigen::Vector3d &forward_out)
{
  forward_out = Eigen::Vector3d::Zero();

  Eigen::Vector3d vel_dir = Eigen::Vector3d::Zero();
  Eigen::Vector3d heading_dir = Eigen::Vector3d::Zero();
  Eigen::Vector3d goal_dir = Eigen::Vector3d::Zero();
  bool has_vel = false;
  bool has_heading = false;
  bool has_goal = false;

  // Step 1: Current motion has priority to avoid abrupt reversals.
  const double vxy = std::hypot(current_velocity_.x(), current_velocity_.y());
  if (vxy > 0.25) {
    vel_dir = Eigen::Vector3d(current_velocity_.x() / vxy, current_velocity_.y() / vxy, 0.0);
    has_vel = true;
  }

  // Step 2: Heading hint fallback.
  if (!heading_hint_.isZero(1e-6)) {
    heading_dir = heading_hint_;
    heading_dir.z() = 0.0;
    if (heading_dir.norm() > 1e-3) {
      heading_dir.normalize();
      has_heading = true;
    }
  }

  // Step 3: Goal direction is only used when it does not imply a reverse turn.
  if (has_current_goal_) {
    goal_dir = current_goal_ - current_position_;
    goal_dir.z() = 0.0;
    if (goal_dir.norm() > 0.4) {
      goal_dir.normalize();
      has_goal = true;
    }
  }

  Eigen::Vector3d base = Eigen::Vector3d::Zero();
  if (has_vel) {
    base = vel_dir;
  } else if (has_heading) {
    base = heading_dir;
  }

  if (has_goal) {
    if (base.norm() > 1e-3) {
      const double align = base.dot(goal_dir);
      // Ignore stale "behind me" goals which would trigger an unnecessary turn.
      if (align > -0.15) {
        base = (0.82 * base + 0.18 * goal_dir).normalized();
      }
    } else {
      base = goal_dir;
    }
  }

  // Step 4: Blend with recent depth-image heading to stay inside visible tunnel opening.
  const bool depth_fresh = has_depth_hint_ &&
    (last_depth_hint_time_.nanoseconds() > 0) &&
    ((this->now() - last_depth_hint_time_).seconds() <= depth_hint_timeout_sec_);
  if (depth_fresh) {
    Eigen::Vector3d depth_dir = depth_forward_hint_;
    depth_dir.z() = 0.0;
    if (depth_dir.norm() > 1e-3) {
      depth_dir.normalize();
      if (base.norm() > 1e-3) {
        const double align = base.dot(depth_dir);
        if (align >= depth_hint_min_align_dot_) {
          base = ((1.0 - depth_hint_weight_) * base + depth_hint_weight_ * depth_dir).normalized();
        }
      }
    }
  }

  // Step 5: If camera is wall-facing or straight path is blocked, pick a map-safe side direction.
  if (octree_) {
    bool needs_escape = depth_wall_blocked_;
    if (base.norm() > 1e-3) {
      const Eigen::Vector3d probe = current_position_ + std::max(2.5, map_escape_lookahead_m_) * base;
      if (!isSegmentSafe(current_position_, probe)) {
        needs_escape = true;
      }
    } else {
      needs_escape = true;
    }

    if (needs_escape) {
      Eigen::Vector3d escape_dir = Eigen::Vector3d::Zero();
      if (chooseMapEscapeDirection(base, escape_dir)) {
        if (base.norm() > 1e-3) {
          base = ((1.0 - escape_blend_weight_) * base + escape_blend_weight_ * escape_dir).normalized();
        } else {
          base = escape_dir;
        }
      }
    }
  }

  // Step 6: Temporal smoothing to suppress left-right oscillations between replans.
  if (base.norm() > 1e-3) {
    if (has_smoothed_forward_hint_) {
      if (smoothed_forward_hint_.dot(base) < -0.2) {
        smoothed_forward_hint_ = base;
      } else {
        smoothed_forward_hint_ =
          (forward_smoothing_alpha_ * smoothed_forward_hint_ + (1.0 - forward_smoothing_alpha_) * base).normalized();
      }
    } else {
      smoothed_forward_hint_ = base;
      has_smoothed_forward_hint_ = true;
    }
    base = smoothed_forward_hint_;
  }

  if (base.norm() < 1e-3) {
    return false;
  }
  forward_out = base.normalized();
  return true;
}

bool PathPlannerNode::extractConnectedSliceCenters(
  const Eigen::Vector3d &slice_center,
  const Eigen::Vector3d &lateral_axis,
  const Eigen::Vector3d &vertical_axis,
  const Eigen::Vector3d &anchor_point,
  Eigen::Vector3d &main_center_out,
  std::vector<Eigen::Vector3d> &secondary_centers_out) const
{
  main_center_out = Eigen::Vector3d::Zero();
  secondary_centers_out.clear();
  if (!octree_) {
    return false;
  }

  Eigen::Vector3d lateral = lateral_axis;
  Eigen::Vector3d vertical = vertical_axis;
  if (lateral.norm() < 1e-3 || vertical.norm() < 1e-3) {
    return false;
  }
  lateral.normalize();
  vertical.normalize();

  const double half_extent = std::max(centerline_slice_half_extent_m_, wall_clearance_min_ + 0.3);
  const double step = std::max(centerline_slice_step_m_, octree_->getResolution());
  const int half_cells = std::max(1, static_cast<int>(std::ceil(half_extent / step)));
  const int size = 2 * half_cells + 1;

  const auto idx_of = [size](int x, int y) {
    return y * size + x;
  };

  std::vector<uint8_t> free_mask(static_cast<size_t>(size * size), 0U);
  std::vector<Eigen::Vector2d> uv_coords(static_cast<size_t>(size * size), Eigen::Vector2d::Zero());
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      const double u = static_cast<double>(x - half_cells) * step;
      const double v = static_cast<double>(y - half_cells) * step;
      const int idx = idx_of(x, y);
      uv_coords[static_cast<size_t>(idx)] = Eigen::Vector2d(u, v);

      const Eigen::Vector3d p = slice_center + u * lateral + v * vertical;
      if (isPointKnownFree(p)) {
        free_mask[static_cast<size_t>(idx)] = 1U;
      }
    }
  }

  int anchor_idx = -1;
  const Eigen::Vector3d rel = anchor_point - slice_center;
  const double anchor_u = rel.dot(lateral);
  const double anchor_v = rel.dot(vertical);
  const int ax = static_cast<int>(std::round(anchor_u / step)) + half_cells;
  const int ay = static_cast<int>(std::round(anchor_v / step)) + half_cells;
  if (ax >= 0 && ax < size && ay >= 0 && ay < size) {
    const int idx = idx_of(ax, ay);
    if (free_mask[static_cast<size_t>(idx)] != 0U) {
      anchor_idx = idx;
    }
  }

  if (anchor_idx < 0) {
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int idx = 0; idx < size * size; ++idx) {
      if (free_mask[static_cast<size_t>(idx)] == 0U) {
        continue;
      }
      const Eigen::Vector2d &uv = uv_coords[static_cast<size_t>(idx)];
      const double du = uv.x() - anchor_u;
      const double dv = uv.y() - anchor_v;
      const double d2 = du * du + dv * dv;
      if (d2 < best_d2) {
        best_d2 = d2;
        anchor_idx = idx;
      }
    }
  }

  if (anchor_idx < 0) {
    return false;
  }

  struct ComponentSummary
  {
    int id{-1};
    int count{0};
    double sum_u{0.0};
    double sum_v{0.0};
    bool has_anchor{false};
  };

  std::vector<int> component_id(static_cast<size_t>(size * size), -1);
  std::vector<ComponentSummary> components;
  components.reserve(16);

  int next_component_id = 0;
  for (int y0 = 0; y0 < size; ++y0) {
    for (int x0 = 0; x0 < size; ++x0) {
      const int start_idx = idx_of(x0, y0);
      if (free_mask[static_cast<size_t>(start_idx)] == 0U ||
          component_id[static_cast<size_t>(start_idx)] >= 0)
      {
        continue;
      }

      ComponentSummary summary;
      summary.id = next_component_id;

      std::queue<std::pair<int, int>> queue;
      queue.emplace(x0, y0);
      component_id[static_cast<size_t>(start_idx)] = next_component_id;

      while (!queue.empty()) {
        const auto [x, y] = queue.front();
        queue.pop();
        const int idx = idx_of(x, y);
        const Eigen::Vector2d &uv = uv_coords[static_cast<size_t>(idx)];
        summary.count++;
        summary.sum_u += uv.x();
        summary.sum_v += uv.y();
        if (idx == anchor_idx) {
          summary.has_anchor = true;
        }

        const std::array<std::pair<int, int>, 4> offsets = {
          std::make_pair(1, 0),
          std::make_pair(-1, 0),
          std::make_pair(0, 1),
          std::make_pair(0, -1),
        };

        for (const auto &[dx, dy] : offsets) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= size || ny < 0 || ny >= size) {
            continue;
          }
          const int nidx = idx_of(nx, ny);
          if (free_mask[static_cast<size_t>(nidx)] == 0U ||
              component_id[static_cast<size_t>(nidx)] >= 0)
          {
            continue;
          }
          component_id[static_cast<size_t>(nidx)] = next_component_id;
          queue.emplace(nx, ny);
        }
      }

      components.push_back(summary);
      next_component_id++;
    }
  }

  const ComponentSummary *main_comp = nullptr;
  for (const auto &comp : components) {
    if (!comp.has_anchor) {
      continue;
    }
    if (comp.count >= centerline_min_component_cells_) {
      main_comp = &comp;
      break;
    }
  }
  if (!main_comp) {
    // Fallback: use anchor component even if tiny.
    for (const auto &comp : components) {
      if (comp.has_anchor) {
        main_comp = &comp;
        break;
      }
    }
  }
  if (!main_comp || main_comp->count <= 0) {
    return false;
  }

  auto collect_component_boundary = [&](int comp_id, std::vector<Eigen::Vector2d> &boundary_out) {
    boundary_out.clear();
    boundary_out.reserve(128);
    for (int y = 0; y < size; ++y) {
      for (int x = 0; x < size; ++x) {
        const int idx = idx_of(x, y);
        if (component_id[static_cast<size_t>(idx)] != comp_id) {
          continue;
        }

        bool is_boundary = false;
        const std::array<std::pair<int, int>, 4> offsets = {
          std::make_pair(1, 0),
          std::make_pair(-1, 0),
          std::make_pair(0, 1),
          std::make_pair(0, -1),
        };
        for (const auto &[dx, dy] : offsets) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx < 0 || nx >= size || ny < 0 || ny >= size) {
            is_boundary = true;
            break;
          }
          const int nidx = idx_of(nx, ny);
          if (component_id[static_cast<size_t>(nidx)] != comp_id) {
            is_boundary = true;
            break;
          }
        }

        if (is_boundary) {
          boundary_out.push_back(uv_coords[static_cast<size_t>(idx)]);
        }
      }
    }
  };

  auto fit_circle_to_boundary = [&](const std::vector<Eigen::Vector2d> &boundary_pts,
                                    Eigen::Vector2d &center_uv_out,
                                    double &radius_out) -> bool {
    center_uv_out = Eigen::Vector2d::Zero();
    radius_out = 0.0;

    if (boundary_pts.size() < 6) {
      return false;
    }

    const int n = static_cast<int>(boundary_pts.size());
    Eigen::MatrixXd A(n, 3);
    Eigen::VectorXd b(n);
    for (int i = 0; i < n; ++i) {
      const double x = boundary_pts[static_cast<size_t>(i)].x();
      const double y = boundary_pts[static_cast<size_t>(i)].y();
      A(i, 0) = x;
      A(i, 1) = y;
      A(i, 2) = 1.0;
      b(i) = -(x * x + y * y);
    }

    const auto qr = A.colPivHouseholderQr();
    if (qr.rank() < 3) {
      return false;
    }
    const Eigen::Vector3d x = qr.solve(b);

    const double cx = -0.5 * x(0);
    const double cy = -0.5 * x(1);
    const double r2 = cx * cx + cy * cy - x(2);
    if (!std::isfinite(r2) || r2 <= 0.0) {
      return false;
    }

    const double radius = std::sqrt(r2);
    if (radius < 0.5 * wall_clearance_min_ || radius > 1.8 * half_extent) {
      return false;
    }

    // Reject unstable fits (large residuals or very small angular support).
    double err2_sum = 0.0;
    std::vector<double> angles;
    angles.reserve(boundary_pts.size());
    for (const auto &pt : boundary_pts) {
      const Eigen::Vector2d d = pt - Eigen::Vector2d(cx, cy);
      err2_sum += std::pow(d.norm() - radius, 2);
      angles.push_back(std::atan2(d.y(), d.x()));
    }
    const double rmse = std::sqrt(err2_sum / static_cast<double>(boundary_pts.size()));
    if (rmse > std::max(0.35, 0.28 * radius)) {
      return false;
    }

    std::sort(angles.begin(), angles.end());
    constexpr double kPi = 3.14159265358979323846;
    double max_gap = 0.0;
    for (size_t i = 1; i < angles.size(); ++i) {
      max_gap = std::max(max_gap, angles[i] - angles[i - 1]);
    }
    max_gap = std::max(max_gap, (angles.front() + 2.0 * kPi) - angles.back());
    const double coverage = 2.0 * kPi - max_gap;
    if (coverage < 1.2) {
      return false;
    }

    center_uv_out = Eigen::Vector2d(cx, cy);
    radius_out = radius;
    return true;
  };

  auto comp_to_world = [&](const ComponentSummary &comp, Eigen::Vector3d &center_out) -> bool {
    if (comp.count <= 0) {
      return false;
    }

    const Eigen::Vector2d centroid_uv(
      comp.sum_u / static_cast<double>(comp.count),
      comp.sum_v / static_cast<double>(comp.count));

    Eigen::Vector2d target_uv = centroid_uv;
    std::vector<Eigen::Vector2d> boundary_pts;
    collect_component_boundary(comp.id, boundary_pts);

    Eigen::Vector2d circle_center_uv = Eigen::Vector2d::Zero();
    double circle_radius = 0.0;
    if (fit_circle_to_boundary(boundary_pts, circle_center_uv, circle_radius)) {
      // Circle-fit center resists one-sided map incompleteness; blend with centroid for stability.
      target_uv = 0.75 * circle_center_uv + 0.25 * centroid_uv;
    }

    int best_idx = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (int idx = 0; idx < size * size; ++idx) {
      if (component_id[static_cast<size_t>(idx)] != comp.id) {
        continue;
      }
      const Eigen::Vector2d d = uv_coords[static_cast<size_t>(idx)] - target_uv;
      const double d2 = d.squaredNorm();
      if (d2 < best_d2) {
        best_d2 = d2;
        best_idx = idx;
      }
    }
    if (best_idx < 0) {
      return false;
    }

    const Eigen::Vector2d &uv = uv_coords[static_cast<size_t>(best_idx)];
    const Eigen::Vector3d c = slice_center + uv.x() * lateral + uv.y() * vertical;
    if (!isPointKnownFree(c)) {
      return false;
    }

    center_out = c;
    return true;
  };

  if (!comp_to_world(*main_comp, main_center_out)) {
    return false;
  }

  std::vector<std::pair<double, Eigen::Vector3d>> secondary_ranked;
  for (const auto &comp : components) {
    if (comp.id == main_comp->id || comp.count < centerline_min_component_cells_) {
      continue;
    }
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    if (!comp_to_world(comp, center)) {
      continue;
    }
    secondary_ranked.emplace_back(distance3(center, main_center_out), center);
  }

  std::sort(secondary_ranked.begin(), secondary_ranked.end(),
    [](const auto &a, const auto &b) { return a.first < b.first; });

  for (const auto &entry : secondary_ranked) {
    if (static_cast<int>(secondary_centers_out.size()) >= centerline_max_secondary_goals_) {
      break;
    }
    secondary_centers_out.push_back(entry.second);
  }

  return true;
}

bool PathPlannerNode::chooseCenterlineGoal(Eigen::Vector3d &goal_out, std::string &reason_out)
{
  if (!octree_) {
    return false;
  }

  Eigen::Vector3d forward = Eigen::Vector3d::Zero();
  if (!computeForwardDirection(forward)) {
    return false;
  }
  forward.z() = 0.0;
  if (forward.norm() < 1e-3) {
    return false;
  }
  forward.normalize();

  Eigen::Vector3d lateral(-forward.y(), forward.x(), 0.0);
  if (lateral.norm() < 1e-3) {
    return false;
  }
  lateral.normalize();
  const Eigen::Vector3d vertical = Eigen::Vector3d::UnitZ();

  debug_candidate_points_.clear();
  debug_centerline_points_.clear();

  std::vector<Eigen::Vector3d> centerline_points;
  centerline_points.reserve(static_cast<size_t>(centerline_horizon_points_));
  std::vector<Eigen::Vector3d> secondary_points;
  secondary_points.reserve(static_cast<size_t>(centerline_horizon_points_ * std::max(1, centerline_max_secondary_goals_)));

  Eigen::Vector3d previous = current_position_;
  for (int i = 0; i < centerline_horizon_points_; ++i) {
    const Eigen::Vector3d slice_center = previous + centerline_step_m_ * forward;

    Eigen::Vector3d main_center = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> secondary_slice;
    if (!extractConnectedSliceCenters(
          slice_center, lateral, vertical, previous, main_center, secondary_slice))
    {
      break;
    }

    if (!isSegmentSafe(previous, main_center)) {
      break;
    }

    centerline_points.push_back(main_center);
    for (const auto &p : secondary_slice) {
      bool duplicate = false;
      for (const auto &existing : secondary_points) {
        if (distance3(existing, p) <= std::max(0.8, centerline_slice_step_m_)) {
          duplicate = true;
          break;
        }
      }
      if (!duplicate) {
        secondary_points.push_back(p);
      }
    }

    Eigen::Vector3d segment = main_center - previous;
    previous = main_center;
    segment.z() = 0.0;
    if (segment.norm() > 0.2) {
      segment.normalize();
      forward = (0.65 * forward + 0.35 * segment).normalized();
      lateral = Eigen::Vector3d(-forward.y(), forward.x(), 0.0);
      if (lateral.norm() > 1e-3) {
        lateral.normalize();
      }
    }
  }

  if (centerline_points.empty()) {
    return false;
  }

  double accum_dist = 0.0;
  Eigen::Vector3d selected = centerline_points.back();
  Eigen::Vector3d prev = current_position_;
  bool committed = false;
  for (const auto &p : centerline_points) {
    accum_dist += distance3(prev, p);
    prev = p;
    if (accum_dist + 1e-6 < centerline_min_commit_dist_m_) {
      continue;
    }
    committed = true;
    selected = p;
    if (accum_dist >= centerline_execute_dist_m_) {
      break;
    }
  }
  if (!committed) {
    selected = centerline_points.back();
  }

  goal_out = selected;
  reason_out = "centerline_slice";
  debug_centerline_points_ = centerline_points;
  debug_candidate_points_.reserve(secondary_points.size() + 1);
  for (const auto &p : secondary_points) {
    debug_candidate_points_.push_back(p);
  }
  debug_candidate_points_.push_back(selected);
  return true;
}

bool PathPlannerNode::chooseExploreGoal(Eigen::Vector3d &goal_out, std::string &reason_out)
{
  if (!octree_) {
    return false;
  }

  // Step 1: Cache direction of motion for yaw-forward bias.
  Eigen::Vector3d v_dir = Eigen::Vector3d::Zero();
  if (has_current_goal_) {
    Eigen::Vector3d goal_dir = current_goal_ - current_position_;
    goal_dir.z() = 0.0;
    if (goal_dir.norm() > 0.3) {
      v_dir = goal_dir.normalized();
    }
  }
  if (v_dir.isZero(1e-6)) {
    const double vxy = std::hypot(current_velocity_.x(), current_velocity_.y());
    if (vxy > 0.3) {
      v_dir = Eigen::Vector3d(current_velocity_.x() / vxy, current_velocity_.y() / vxy, 0.0);
    }
  }
  if (v_dir.isZero(1e-6) && !heading_hint_.isZero(1e-6)) {
    v_dir = heading_hint_;
  }

  // Step 2: Score all free leaves as frontier candidates.
  debug_candidate_points_.clear();
  debug_centerline_points_.clear();

  bool found = false;
  Eigen::Vector3d best_pos = current_position_;
  double best_score = -std::numeric_limits<double>::infinity();
  std::vector<std::pair<double, Eigen::Vector3d>> top_candidates;
  top_candidates.reserve(static_cast<size_t>(marker_max_candidates_));
  auto push_top_candidate = [&](double score, const Eigen::Vector3d &p) {
    if (marker_max_candidates_ <= 0) {
      return;
    }
    top_candidates.emplace_back(score, p);
    std::sort(
      top_candidates.begin(), top_candidates.end(),
      [](const auto &a, const auto &b) { return a.first > b.first; });
    if (static_cast<int>(top_candidates.size()) > marker_max_candidates_) {
      top_candidates.pop_back();
    }
  };

  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    if (octree_->isNodeOccupied(*it)) {
      continue;
    }

    const Eigen::Vector3d p(it.getX(), it.getY(), it.getZ());
    const double d = distance3(current_position_, p);
    if (d < frontier_goal_dist_min_ || d > frontier_goal_dist_max_) {
      continue;
    }

    const int unknown_neighbors = countUnknownNeighbors(p);
    if (unknown_neighbors < frontier_unknown_min_neighbors_) {
      continue;
    }
    const int known_free_neighbors = countKnownFreeNeighbors(p);
    if (known_free_neighbors < frontier_known_free_min_neighbors_) {
      continue;
    }

    const double clearance = estimateClearance(p, clearance_probe_radius_m_);
    if (clearance < wall_clearance_min_) {
      continue;
    }

    Eigen::Vector3d dir = p - current_position_;
    dir.z() = 0.0;
    if (dir.norm() < 1e-6) {
      continue;
    }
    dir.normalize();
    const double forward = v_dir.isZero(1e-6) ? 0.0 : std::max(0.0, v_dir.dot(dir));
    if (!v_dir.isZero(1e-6) && v_dir.dot(dir) < forward_min_dot_) {
      continue;
    }

    const double score =
      score_unknown_weight_ * static_cast<double>(unknown_neighbors) +
      score_clearance_weight_ * clearance -
      score_revisit_weight_ * revisitPenalty(p) +
      score_distance_weight_ * d +
      score_forward_weight_ * forward;

    push_top_candidate(score, p);

    if (score > best_score) {
      best_pos = p;
      best_score = score;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  debug_candidate_points_.clear();
  debug_centerline_points_.clear();
  debug_candidate_points_.reserve(top_candidates.size());
  for (const auto &candidate : top_candidates) {
    debug_candidate_points_.push_back(candidate.second);
  }

  // Step 3: Snap candidate to safe free map cell.
  Eigen::Vector3d goal = best_pos;
  if (!projectGoalToFree(goal)) {
    return false;
  }

  goal_out = goal;
  reason_out = "frontier";
  return true;
}

bool PathPlannerNode::chooseFallbackGoal(Eigen::Vector3d &goal_out, std::string &reason_out)
{
  if (!octree_) {
    return false;
  }

  // Step 1: Score free cells without unknown-neighbor requirement.
  debug_candidate_points_.clear();
  debug_centerline_points_.clear();

  bool found = false;
  Eigen::Vector3d best_pos = current_position_;
  double best_score = -std::numeric_limits<double>::infinity();
  std::vector<std::pair<double, Eigen::Vector3d>> top_candidates;
  top_candidates.reserve(static_cast<size_t>(marker_max_candidates_));
  auto push_top_candidate = [&](double score, const Eigen::Vector3d &p) {
    if (marker_max_candidates_ <= 0) {
      return;
    }
    top_candidates.emplace_back(score, p);
    std::sort(
      top_candidates.begin(), top_candidates.end(),
      [](const auto &a, const auto &b) { return a.first > b.first; });
    if (static_cast<int>(top_candidates.size()) > marker_max_candidates_) {
      top_candidates.pop_back();
    }
  };

  for (auto it = octree_->begin_leafs(), end = octree_->end_leafs(); it != end; ++it) {
    if (octree_->isNodeOccupied(*it)) {
      continue;
    }

    const Eigen::Vector3d p(it.getX(), it.getY(), it.getZ());
    const double d = distance3(current_position_, p);
    if (d < frontier_goal_dist_min_ || d > frontier_goal_dist_max_) {
      continue;
    }

    const double clearance = estimateClearance(p, clearance_probe_radius_m_);
    if (clearance < wall_clearance_min_) {
      continue;
    }
    const int known_free_neighbors = countKnownFreeNeighbors(p);
    if (known_free_neighbors < std::max(3, frontier_known_free_min_neighbors_ / 2)) {
      continue;
    }

    const double score =
      score_clearance_weight_ * clearance +
      0.25 * d -
      score_revisit_weight_ * revisitPenalty(p);

    push_top_candidate(score, p);

    if (score > best_score) {
      best_pos = p;
      best_score = score;
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  debug_candidate_points_.clear();
  debug_centerline_points_.clear();
  debug_candidate_points_.reserve(top_candidates.size());
  for (const auto &candidate : top_candidates) {
    debug_candidate_points_.push_back(candidate.second);
  }

  Eigen::Vector3d goal = best_pos;
  if (!projectGoalToFree(goal)) {
    return false;
  }

  goal_out = goal;
  reason_out = "fallback";
  return true;
}

bool PathPlannerNode::chooseReturnGoal(Eigen::Vector3d &goal_out, std::string &reason_out) const
{
  if (!has_home_pose_) {
    return false;
  }

  goal_out = home_position_;
  reason_out = "return_home";
  return true;
}

bool PathPlannerNode::projectGoalToFree(Eigen::Vector3d &goal_in_out) const
{
  if (!octree_) {
    return false;
  }

  // Step 1: Keep original goal when already safe.
  if (isPointSafe(goal_in_out)) {
    return true;
  }

  // Step 2: Search nearby free points in expanding rings.
  const double res = octree_->getResolution();
  const double max_radius = std::max(2.0 * wall_clearance_min_, 3.0 * res);

  for (double r = res; r <= max_radius; r += res) {
    for (const auto &dir : kProbeDirections) {
      const Eigen::Vector3d candidate = goal_in_out + r * dir;
      if (isPointSafe(candidate)) {
        goal_in_out = candidate;
        return true;
      }
    }
  }

  return false;
}

bool PathPlannerNode::planPathOmpl(
  const Eigen::Vector3d &start,
  const Eigen::Vector3d &goal,
  std::vector<Eigen::Vector3d> &path_out) const
{
  if (!octree_) {
    return false;
  }

  // Step 1: Ensure start/goal are valid free points.
  Eigen::Vector3d start_safe = start;
  Eigen::Vector3d goal_safe = goal;
  if (!projectGoalToFree(start_safe) || !projectGoalToFree(goal_safe)) {
    return false;
  }

  // Step 1a: Reject large start projection shifts (can look like wall crossing at replan start).
  const double max_start_shift = std::max(0.8, 3.0 * octree_->getResolution());
  if (distance3(start_safe, start) > max_start_shift) {
    return false;
  }
  if (!isSegmentSafe(start, start_safe)) {
    return false;
  }

  // Step 2: Build 3D planning space and bounds from octomap extent.
  auto space = std::make_shared<ob::RealVectorStateSpace>(3);

  double min_x = 0.0;
  double min_y = 0.0;
  double min_z = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  double max_z = 0.0;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);

  min_x = std::min({min_x, start_safe.x(), goal_safe.x()}) - bounds_padding_m_;
  min_y = std::min({min_y, start_safe.y(), goal_safe.y()}) - bounds_padding_m_;
  min_z = std::min({min_z, start_safe.z(), goal_safe.z()}) - bounds_padding_m_;
  max_x = std::max({max_x, start_safe.x(), goal_safe.x()}) + bounds_padding_m_;
  max_y = std::max({max_y, start_safe.y(), goal_safe.y()}) + bounds_padding_m_;
  max_z = std::max({max_z, start_safe.z(), goal_safe.z()}) + bounds_padding_m_;

  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, min_x);
  bounds.setLow(1, min_y);
  bounds.setLow(2, min_z);
  bounds.setHigh(0, max_x);
  bounds.setHigh(1, max_y);
  bounds.setHigh(2, max_z);
  space->setBounds(bounds);

  // Step 3: Configure OMPL simple setup and collision checker.
  og::SimpleSetup simple_setup(space);
  simple_setup.setStateValidityChecker([this](const ob::State *state) {
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
    return isPointSafe(Eigen::Vector3d(s->values[0], s->values[1], s->values[2]));
  });
  simple_setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

  ob::ScopedState<ob::RealVectorStateSpace> start_state(space);
  ob::ScopedState<ob::RealVectorStateSpace> goal_state(space);
  start_state[0] = start_safe.x();
  start_state[1] = start_safe.y();
  start_state[2] = start_safe.z();
  goal_state[0] = goal_safe.x();
  goal_state[1] = goal_safe.y();
  goal_state[2] = goal_safe.z();

  simple_setup.setStartAndGoalStates(start_state, goal_state);

  auto planner = std::make_shared<og::RRTConnect>(simple_setup.getSpaceInformation());
  planner->setRange(ompl_range_m_);
  simple_setup.setPlanner(planner);

  // Step 4: Solve and simplify.
  const ob::PlannerStatus solved = simple_setup.solve(ompl_timeout_sec_);
  if (!solved) {
    return false;
  }

  auto export_path_points = [](const og::PathGeometric &path, std::vector<Eigen::Vector3d> &out) {
    out.clear();
    out.reserve(path.getStateCount());
    for (size_t i = 0; i < path.getStateCount(); ++i) {
      const auto *s = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
      out.emplace_back(s->values[0], s->values[1], s->values[2]);
    }
  };

  const og::PathGeometric raw_path = simple_setup.getSolutionPath();
  std::vector<Eigen::Vector3d> raw_points;
  export_path_points(raw_path, raw_points);
  if (raw_points.size() < 2 || !isPathSafe(raw_points)) {
    return false;
  }

  og::PathGeometric path = raw_path;
  og::PathSimplifier simplifier(simple_setup.getSpaceInformation());
  simplifier.reduceVertices(path);
  simplifier.shortcutPath(path);
  if (path.getStateCount() >= 4) {
    simplifier.smoothBSpline(path, 2);
  }

  std::vector<Eigen::Vector3d> simplified_points;
  export_path_points(path, simplified_points);
  if (simplified_points.size() >= 2 && isPathSafe(simplified_points)) {
    path_out = std::move(simplified_points);
  } else {
    path_out = std::move(raw_points);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Simplified path violated safety check, fallback to raw OMPL path.");
  }

  // Step 5: Anchor path at actual current start position to avoid discontinuity artifacts.
  if (path_out.empty()) {
    return false;
  }
  if (distance3(path_out.front(), start) > 1e-3) {
    if (!isSegmentSafe(start, path_out.front())) {
      return false;
    }
    path_out.insert(path_out.begin(), start);
  }

  if (!isPathSafe(path_out)) {
    return false;
  }

  return true;
}

bool PathPlannerNode::resamplePath(
  const std::vector<Eigen::Vector3d> &path_in,
  std::vector<Eigen::Vector3d> &waypoints_out) const
{
  waypoints_out.clear();
  if (path_in.size() < 2) {
    return false;
  }

  // Step 1: Build points with adaptive spacing:
  // larger in open/straight tunnel parts, smaller near walls/turns.
  const double span = std::max(0.1, waypoint_dist_max_ - waypoint_dist_min_);
  Eigen::Vector3d prev_dir = Eigen::Vector3d::Zero();
  bool has_prev_dir = false;

  waypoints_out.push_back(path_in.front());

  for (size_t i = 1; i < path_in.size(); ++i) {
    const Eigen::Vector3d p0 = path_in[i - 1];
    const Eigen::Vector3d p1 = path_in[i];
    const Eigen::Vector3d seg = p1 - p0;
    const double seg_len = seg.norm();
    if (seg_len < 1e-6) {
      continue;
    }

    const Eigen::Vector3d seg_dir = seg / seg_len;
    double step = std::clamp(0.5 * (waypoint_dist_min_ + waypoint_dist_max_),
      waypoint_dist_min_, waypoint_dist_max_);

    if (adaptive_waypoint_step_enabled_) {
      const Eigen::Vector3d midpoint = 0.5 * (p0 + p1);
      const double clearance = estimateClearance(
        midpoint, wall_clearance_min_ + adaptive_step_clearance_range_m_);
      const double clearance_ratio = std::clamp(
        (clearance - wall_clearance_min_) / std::max(0.1, adaptive_step_clearance_range_m_), 0.0, 1.0);

      double turn_straightness = 1.0;
      if (has_prev_dir) {
        turn_straightness = std::clamp(0.5 * (1.0 + prev_dir.dot(seg_dir)), 0.0, 1.0);
      }
      const double turn_ratio =
        (1.0 - adaptive_step_turn_weight_) + adaptive_step_turn_weight_ * turn_straightness;
      const double ratio = std::clamp(
        adaptive_step_min_ratio_ + (1.0 - adaptive_step_min_ratio_) * clearance_ratio * turn_ratio,
        adaptive_step_min_ratio_, 1.0);

      step = waypoint_dist_min_ + ratio * span;
      step = std::clamp(step, waypoint_dist_min_, waypoint_dist_max_);
    }

    double dist = step;
    while (dist < seg_len) {
      const double t = dist / seg_len;
      const Eigen::Vector3d sample = p0 + t * seg;
      if (distance3(sample, waypoints_out.back()) >= waypoint_dist_min_ * 0.8) {
        waypoints_out.push_back(sample);
      }
      dist += step;
    }
    prev_dir = seg_dir;
    has_prev_dir = true;
  }

  // Step 2: Ensure final goal is included.
  if (distance3(waypoints_out.back(), path_in.back()) > 1e-3) {
    waypoints_out.push_back(path_in.back());
  }

  // Step 3: Enforce maximum segment length (split long segments).
  std::vector<Eigen::Vector3d> split_waypoints;
  split_waypoints.reserve(waypoints_out.size() * 2);
  split_waypoints.push_back(waypoints_out.front());

  for (size_t i = 1; i < waypoints_out.size(); ++i) {
    const Eigen::Vector3d p0 = split_waypoints.back();
    const Eigen::Vector3d p1 = waypoints_out[i];
    const double d = distance3(p0, p1);
    if (d <= waypoint_dist_max_) {
      split_waypoints.push_back(p1);
      continue;
    }

    const int pieces = std::max(1, static_cast<int>(std::ceil(d / waypoint_dist_max_)));
    for (int k = 1; k <= pieces; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(pieces);
      split_waypoints.push_back(p0 + t * (p1 - p0));
    }
  }

  waypoints_out = std::move(split_waypoints);

  return waypoints_out.size() >= 2;
}

bool PathPlannerNode::buildAndPublishTrajectory(
  const std::vector<Eigen::Vector3d> &waypoints,
  bool force_stop_at_end)
{
  if (waypoints.size() < 2) {
    return false;
  }

  // Step 1: Convert waypoint chain into MAV trajectory vertices.
  const int dimension = 3;
  const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

  mav_trajectory_generation::Vertex::Vector vertices;
  vertices.reserve(waypoints.size() + 1);

  mav_trajectory_generation::Vertex start(dimension);
  start.makeStartOrEnd(current_position_, derivative_to_optimize);

  Eigen::Vector3d v0 = current_velocity_;
  if (v0.norm() > max_v_) {
    v0 = v0.normalized() * max_v_;
  }
  start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v0);
  vertices.push_back(start);

  for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
    mav_trajectory_generation::Vertex middle(dimension);
    middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints[i]);
    vertices.push_back(middle);
  }

  mav_trajectory_generation::Vertex end(dimension);
  end.makeStartOrEnd(waypoints.back(), derivative_to_optimize);
  if (force_stop_at_end) {
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
    end.addConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION, Eigen::Vector3d::Zero());
  }
  vertices.push_back(end);

  // Step 2: Solve polynomial optimization with dynamic v/a limits.
  std::vector<double> segment_times =
    estimateSegmentTimes(vertices, std::max(0.2, max_v_), std::max(0.2, max_a_));
  if (segment_times.empty()) {
    return false;
  }

  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(
    mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
  opt.optimize();

  mav_trajectory_generation::Trajectory trajectory;
  opt.getTrajectory(&trajectory);

  // Step 3: Publish for trajectory sampler node.
  mav_planning_msgs::msg::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
  msg.header.frame_id = "world";
  const auto now = this->now();
  msg.header.stamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
  msg.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
  trajectory_pub_->publish(msg);

  return true;
}

bool PathPlannerNode::hasValidMapAndPose() const
{
  return has_current_pose_ && has_octomap_ && static_cast<bool>(octree_);
}

bool PathPlannerNode::isPointKnownFree(const Eigen::Vector3d &p) const
{
  if (!octree_) {
    return false;
  }

  const octomap::OcTreeNode *node = octree_->search(p.x(), p.y(), p.z());
  if (!node) {
    return false;
  }

  return !octree_->isNodeOccupied(node);
}

bool PathPlannerNode::isPointSafe(const Eigen::Vector3d &p) const
{
  if (!isPointKnownFree(p)) {
    return false;
  }

  const double clearance = estimateClearance(p, wall_clearance_min_);
  return clearance + 1e-6 >= wall_clearance_min_;
}

bool PathPlannerNode::isSegmentSafe(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const
{
  if (!octree_) {
    return false;
  }

  const double length = distance3(a, b);
  if (length <= 1e-6) {
    return isPointSafe(a);
  }

  const double res = octree_->getResolution();
  const double step = std::max(path_check_step_m_, 0.5 * res);
  const int steps = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(steps);
    if (!isPointSafe(a + t * (b - a))) {
      return false;
    }
  }
  return true;
}

bool PathPlannerNode::isPathSafe(const std::vector<Eigen::Vector3d> &path) const
{
  if (path.size() < 2) {
    return false;
  }
  for (size_t i = 1; i < path.size(); ++i) {
    if (!isSegmentSafe(path[i - 1], path[i])) {
      return false;
    }
  }
  return true;
}

int PathPlannerNode::countUnknownNeighbors(const Eigen::Vector3d &p) const
{
  if (!octree_) {
    return 0;
  }

  const double res = octree_->getResolution();
  int unknown_count = 0;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }

        const octomap::point3d q(
          static_cast<float>(p.x() + static_cast<double>(dx) * res),
          static_cast<float>(p.y() + static_cast<double>(dy) * res),
          static_cast<float>(p.z() + static_cast<double>(dz) * res));

        if (!octree_->search(q)) {
          unknown_count++;
        }
      }
    }
  }

  return unknown_count;
}

int PathPlannerNode::countKnownFreeNeighbors(const Eigen::Vector3d &p) const
{
  if (!octree_) {
    return 0;
  }

  const double res = octree_->getResolution();
  int known_free_count = 0;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }

        const Eigen::Vector3d q(
          p.x() + static_cast<double>(dx) * res,
          p.y() + static_cast<double>(dy) * res,
          p.z() + static_cast<double>(dz) * res);
        if (isPointKnownFree(q)) {
          known_free_count++;
        }
      }
    }
  }

  return known_free_count;
}

double PathPlannerNode::estimateClearance(const Eigen::Vector3d &p, double max_radius) const
{
  if (!octree_) {
    return 0.0;
  }

  const double res = octree_->getResolution();
  const double probe_step = std::max(res, 0.05);
  const double radius = std::max(probe_step, max_radius);

  double min_hit = radius;

  for (const auto &dir : kProbeDirections) {
    for (double d = probe_step; d <= radius; d += probe_step) {
      const Eigen::Vector3d q = p + d * dir;
      const octomap::OcTreeNode *node = octree_->search(q.x(), q.y(), q.z());
      if (node && octree_->isNodeOccupied(node)) {
        min_hit = std::min(min_hit, d);
        break;
      }
    }
  }

  return min_hit;
}

double PathPlannerNode::revisitPenalty(const Eigen::Vector3d &p) const
{
  if (visited_goals_.empty()) {
    return 0.0;
  }

  const double influence_radius = std::max(frontier_goal_dist_min_, 2.0 * waypoint_dist_max_);
  if (influence_radius <= 1e-6) {
    return 0.0;
  }

  double penalty = 0.0;
  for (const auto &visited : visited_goals_) {
    const double d = distance3(p, visited);
    if (d < influence_radius) {
      penalty += (influence_radius - d) / influence_radius;
    }
  }

  return penalty;
}

double PathPlannerNode::distance3(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
  return (a - b).norm();
}

bool PathPlannerNode::hasReachedGoal() const
{
  if (!has_current_goal_) {
    return false;
  }

  return distance3(current_position_, current_goal_) <= goal_reached_radius_m_;
}

void PathPlannerNode::storeVisitedGoal(const Eigen::Vector3d &goal)
{
  visited_goals_.push_back(goal);
  if (visited_goals_.size() > kVisitedGoalsCap) {
    visited_goals_.erase(visited_goals_.begin());
  }
}

void PathPlannerNode::publishSelectedPath(const std::vector<Eigen::Vector3d> &path_points)
{
  if (!selected_path_pub_ || path_points.empty()) {
    return;
  }

  // Step 1: Publish only the currently selected path. This avoids artificial
  // line-strip links between different re-plans that can look like wall crossings.
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = viz_frame_id_;
  const auto now = this->now();
  path_msg.header.stamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
  path_msg.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
  path_msg.poses.reserve(path_points.size());

  for (const auto &p : path_points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position = toPointMsg(p);
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(std::move(pose));
  }

  selected_path_pub_->publish(path_msg);
}

void PathPlannerNode::publishPlanningVisualization(
  const std::vector<Eigen::Vector3d> &ompl_path,
  const std::vector<Eigen::Vector3d> &waypoints)
{
  if (!marker_pub_) {
    return;
  }

  const auto now = this->now();
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.reserve(10);

  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = viz_frame_id_;
  clear.header.stamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
  clear.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear);

  auto make_marker = [&](int id, const std::string &ns, int type, float sx, float sy, float sz,
                         float r, float g, float b, float a) {
    visualization_msgs::msg::Marker m;
    m.header = clear.header;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    m.lifetime = rclcpp::Duration::from_seconds(0.0);
    return m;
  };

  auto candidates = make_marker(
    1, "candidate_nodes", visualization_msgs::msg::Marker::SPHERE_LIST,
    0.24F, 0.24F, 0.24F, 0.2F, 0.6F, 1.0F, 0.7F);
  for (const auto &p : debug_candidate_points_) {
    candidates.points.push_back(toPointMsg(p));
  }
  marker_array.markers.push_back(candidates);

  auto centerline_points = make_marker(
    5, "centerline_midpoints", visualization_msgs::msg::Marker::SPHERE_LIST,
    0.22F, 0.22F, 0.22F, 0.15F, 1.0F, 0.2F, 0.95F);
  for (const auto &p : debug_centerline_points_) {
    centerline_points.points.push_back(toPointMsg(p));
  }
  marker_array.markers.push_back(centerline_points);

  auto centerline_line = make_marker(
    6, "centerline_polyline", visualization_msgs::msg::Marker::LINE_STRIP,
    0.08F, 0.0F, 0.0F, 0.15F, 0.95F, 0.25F, 0.85F);
  centerline_line.points.push_back(toPointMsg(current_position_));
  for (const auto &p : debug_centerline_points_) {
    centerline_line.points.push_back(toPointMsg(p));
  }
  marker_array.markers.push_back(centerline_line);

  const bool depth_fresh = has_depth_hint_ &&
    (last_depth_hint_time_.nanoseconds() > 0) &&
    ((now - last_depth_hint_time_).seconds() <= depth_hint_timeout_sec_);
  if (depth_fresh) {
    auto depth_center = make_marker(
      7, "depth_midpoint", visualization_msgs::msg::Marker::SPHERE,
      0.34F, 0.34F, 0.34F, 0.95F, 0.2F, 0.85F, 0.95F);
    depth_center.pose.position = toPointMsg(depth_aim_point_world_);
    depth_center.pose.orientation.w = 1.0;
    marker_array.markers.push_back(depth_center);

    auto depth_heading = make_marker(
      8, "depth_heading", visualization_msgs::msg::Marker::LINE_STRIP,
      0.09F, 0.0F, 0.0F, 0.95F, 0.45F, 0.2F, 0.95F);
    depth_heading.points.push_back(toPointMsg(current_position_));
    depth_heading.points.push_back(toPointMsg(current_position_ + 4.0 * depth_forward_hint_));
    marker_array.markers.push_back(depth_heading);
  }

  auto ompl_line = make_marker(
    2, "selected_ompl_path", visualization_msgs::msg::Marker::LINE_STRIP,
    0.12F, 0.0F, 0.0F, 0.1F, 0.9F, 0.9F, 1.0F);
  for (const auto &p : ompl_path) {
    ompl_line.points.push_back(toPointMsg(p));
  }
  marker_array.markers.push_back(ompl_line);

  auto waypoint_nodes = make_marker(
    3, "trajectory_waypoints", visualization_msgs::msg::Marker::SPHERE_LIST,
    0.28F, 0.28F, 0.28F, 1.0F, 0.7F, 0.1F, 1.0F);
  for (const auto &p : waypoints) {
    waypoint_nodes.points.push_back(toPointMsg(p));
  }
  marker_array.markers.push_back(waypoint_nodes);

  if (!waypoints.empty()) {
    auto goal_marker = make_marker(
      4, "selected_goal", visualization_msgs::msg::Marker::SPHERE,
      0.5F, 0.5F, 0.5F, 1.0F, 0.2F, 0.2F, 1.0F);
    goal_marker.pose.position = toPointMsg(waypoints.back());
    goal_marker.pose.orientation.w = 1.0;
    marker_array.markers.push_back(goal_marker);
  }

  marker_pub_->publish(marker_array);
}

bool PathPlannerNode::shouldReportDone() const
{
  if (mode_ != PlannerMode::EXPLORE) {
    return false;
  }

  if (frontier_fail_cycles_ < max_frontier_fail_cycles_) {
    return false;
  }

  // Keep exploring until all expected lanterns were observed.
  return static_cast<int>(tracked_lanterns_.size()) >= required_lantern_count_;
}

void PathPlannerNode::publishHeartbeat(uint8_t state, const std::string &info) const
{
  statemachine_pkg::msg::Answer hb;
  hb.node_name = this->get_name();
  hb.state = state;
  hb.info = info;
  const auto now = this->now();
  hb.timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
  hb.timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
  heartbeat_pub_->publish(hb);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
