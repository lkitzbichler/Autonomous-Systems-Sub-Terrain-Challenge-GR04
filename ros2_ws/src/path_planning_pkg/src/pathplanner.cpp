#include "pathplanner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

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
  active_path_follow_dist_m_ =
    this->declare_parameter<double>("active_path_follow_dist_m", active_path_follow_dist_m_);
  active_path_replan_progress_ratio_ = this->declare_parameter<double>(
    "active_path_replan_progress_ratio", active_path_replan_progress_ratio_);
  active_path_replan_front_obstacle_m_ = this->declare_parameter<double>(
    "active_path_replan_front_obstacle_m", active_path_replan_front_obstacle_m_);

  ompl_timeout_sec_ = this->declare_parameter<double>("ompl_timeout_sec", ompl_timeout_sec_);
  ompl_range_m_ = this->declare_parameter<double>("ompl_range_m", ompl_range_m_);
  ompl_max_turn_deg_ = this->declare_parameter<double>("ompl_max_turn_deg", ompl_max_turn_deg_);
  ompl_corner_smoothing_passes_ = this->declare_parameter<int>(
    "ompl_corner_smoothing_passes", ompl_corner_smoothing_passes_);
  ompl_corner_blend_ =
    this->declare_parameter<double>("ompl_corner_blend", ompl_corner_blend_);
  bounds_padding_m_ = this->declare_parameter<double>("bounds_padding_m", bounds_padding_m_);
  max_frontier_fail_cycles_ =
    this->declare_parameter<int>("max_frontier_fail_cycles", max_frontier_fail_cycles_);

  lantern_merge_dist_m_ = this->declare_parameter<double>("lantern_merge_dist_m", lantern_merge_dist_m_);
  heading_hint_min_speed_ =
    this->declare_parameter<double>("heading_hint_min_speed", heading_hint_min_speed_);
  adaptive_waypoint_step_enabled_ =
    this->declare_parameter<bool>("adaptive_waypoint_step_enabled", adaptive_waypoint_step_enabled_);
  adaptive_step_clearance_range_m_ = this->declare_parameter<double>(
    "adaptive_step_clearance_range_m", adaptive_step_clearance_range_m_);
  adaptive_step_turn_weight_ =
    this->declare_parameter<double>("adaptive_step_turn_weight", adaptive_step_turn_weight_);
  adaptive_step_min_ratio_ =
    this->declare_parameter<double>("adaptive_step_min_ratio", adaptive_step_min_ratio_);
  traj_start_speed_max_ratio_ =
    this->declare_parameter<double>("traj_start_speed_max_ratio", traj_start_speed_max_ratio_);
  traj_start_align_cos_min_ =
    this->declare_parameter<double>("traj_start_align_cos_min", traj_start_align_cos_min_);
  traj_time_scale_ =
    this->declare_parameter<double>("traj_time_scale", traj_time_scale_);
  marker_max_candidates_ = this->declare_parameter<int>("marker_max_candidates", marker_max_candidates_);

  // Step 3: Declare graph parameters.
  graph_backbone_spacing_m_ =
    this->declare_parameter<double>("graph_backbone_spacing_m", graph_backbone_spacing_m_);
  graph_explore_spacing_m_ =
    this->declare_parameter<double>("graph_explore_spacing_m", graph_explore_spacing_m_);
  graph_merge_dist_m_ = this->declare_parameter<double>("graph_merge_dist_m", graph_merge_dist_m_);
  graph_junction_probe_dist_m_ =
    this->declare_parameter<double>("graph_junction_probe_dist_m", graph_junction_probe_dist_m_);
  graph_branch_goal_dist_m_ =
    this->declare_parameter<double>("graph_branch_goal_dist_m", graph_branch_goal_dist_m_);
  graph_branch_merge_dist_m_ =
    this->declare_parameter<double>("graph_branch_merge_dist_m", graph_branch_merge_dist_m_);
  graph_frontier_revisit_penalty_ =
    this->declare_parameter<double>("graph_frontier_revisit_penalty", graph_frontier_revisit_penalty_);
  graph_forward_bias_weight_ =
    this->declare_parameter<double>("graph_forward_bias_weight", graph_forward_bias_weight_);
  graph_vertical_penalty_weight_ =
    this->declare_parameter<double>("graph_vertical_penalty_weight", graph_vertical_penalty_weight_);
  graph_view_align_weight_ =
    this->declare_parameter<double>("graph_view_align_weight", graph_view_align_weight_);
  graph_center_search_radius_m_ =
    this->declare_parameter<double>("graph_center_search_radius_m", graph_center_search_radius_m_);
  graph_min_junction_free_dirs_ =
    this->declare_parameter<int>("graph_min_junction_free_dirs", graph_min_junction_free_dirs_);
  graph_max_frontiers_per_node_ =
    this->declare_parameter<int>("graph_max_frontiers_per_node", graph_max_frontiers_per_node_);
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
  body_yaw_correction_deg_ = this->declare_parameter<double>(
    "body_yaw_correction_deg", body_yaw_correction_deg_);

  // Step 4: Clamp to robust ranges.
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
  (void)this->get_parameter("active_path_follow_dist_m", active_path_follow_dist_m_);
  (void)this->get_parameter("active_path_replan_progress_ratio", active_path_replan_progress_ratio_);
  (void)this->get_parameter("active_path_replan_front_obstacle_m", active_path_replan_front_obstacle_m_);
  (void)this->get_parameter("ompl_timeout_sec", ompl_timeout_sec_);
  (void)this->get_parameter("ompl_range_m", ompl_range_m_);
  (void)this->get_parameter("ompl_max_turn_deg", ompl_max_turn_deg_);
  (void)this->get_parameter("ompl_corner_smoothing_passes", ompl_corner_smoothing_passes_);
  (void)this->get_parameter("ompl_corner_blend", ompl_corner_blend_);
  (void)this->get_parameter("bounds_padding_m", bounds_padding_m_);
  (void)this->get_parameter("max_frontier_fail_cycles", max_frontier_fail_cycles_);
  (void)this->get_parameter("lantern_merge_dist_m", lantern_merge_dist_m_);
  (void)this->get_parameter("heading_hint_min_speed", heading_hint_min_speed_);
  (void)this->get_parameter("adaptive_waypoint_step_enabled", adaptive_waypoint_step_enabled_);
  (void)this->get_parameter("adaptive_step_clearance_range_m", adaptive_step_clearance_range_m_);
  (void)this->get_parameter("adaptive_step_turn_weight", adaptive_step_turn_weight_);
  (void)this->get_parameter("adaptive_step_min_ratio", adaptive_step_min_ratio_);
  (void)this->get_parameter("traj_start_speed_max_ratio", traj_start_speed_max_ratio_);
  (void)this->get_parameter("traj_start_align_cos_min", traj_start_align_cos_min_);
  (void)this->get_parameter("traj_time_scale", traj_time_scale_);
  (void)this->get_parameter("marker_max_candidates", marker_max_candidates_);

  (void)this->get_parameter("graph_backbone_spacing_m", graph_backbone_spacing_m_);
  (void)this->get_parameter("graph_explore_spacing_m", graph_explore_spacing_m_);
  (void)this->get_parameter("graph_merge_dist_m", graph_merge_dist_m_);
  (void)this->get_parameter("graph_junction_probe_dist_m", graph_junction_probe_dist_m_);
  (void)this->get_parameter("graph_branch_goal_dist_m", graph_branch_goal_dist_m_);
  (void)this->get_parameter("graph_branch_merge_dist_m", graph_branch_merge_dist_m_);
  (void)this->get_parameter("graph_frontier_revisit_penalty", graph_frontier_revisit_penalty_);
  (void)this->get_parameter("graph_forward_bias_weight", graph_forward_bias_weight_);
  (void)this->get_parameter("graph_vertical_penalty_weight", graph_vertical_penalty_weight_);
  (void)this->get_parameter("graph_view_align_weight", graph_view_align_weight_);
  (void)this->get_parameter("graph_center_search_radius_m", graph_center_search_radius_m_);
  (void)this->get_parameter("graph_min_junction_free_dirs", graph_min_junction_free_dirs_);
  (void)this->get_parameter("graph_max_frontiers_per_node", graph_max_frontiers_per_node_);
  (void)this->get_parameter("centerline_horizon_points", centerline_horizon_points_);
  (void)this->get_parameter("centerline_step_m", centerline_step_m_);
  (void)this->get_parameter("centerline_slice_half_extent_m", centerline_slice_half_extent_m_);
  (void)this->get_parameter("centerline_slice_step_m", centerline_slice_step_m_);
  (void)this->get_parameter("centerline_min_component_cells", centerline_min_component_cells_);
  (void)this->get_parameter("centerline_max_secondary_goals", centerline_max_secondary_goals_);
  (void)this->get_parameter("centerline_execute_dist_m", centerline_execute_dist_m_);
  (void)this->get_parameter("centerline_min_commit_dist_m", centerline_min_commit_dist_m_);
  (void)this->get_parameter("body_yaw_correction_deg", body_yaw_correction_deg_);

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
  active_path_follow_dist_m_ = std::max(0.3, active_path_follow_dist_m_);
  active_path_replan_progress_ratio_ = std::clamp(active_path_replan_progress_ratio_, 0.05, 0.95);
  active_path_replan_front_obstacle_m_ = std::max(1.0, active_path_replan_front_obstacle_m_);
  ompl_timeout_sec_ = std::max(0.02, ompl_timeout_sec_);
  ompl_range_m_ = std::max(0.2, ompl_range_m_);
  ompl_max_turn_deg_ = std::clamp(ompl_max_turn_deg_, 20.0, 170.0);
  ompl_corner_smoothing_passes_ = std::clamp(ompl_corner_smoothing_passes_, 0, 8);
  ompl_corner_blend_ = std::clamp(ompl_corner_blend_, 0.0, 1.0);
  bounds_padding_m_ = std::max(0.2, bounds_padding_m_);
  max_frontier_fail_cycles_ = std::max(1, max_frontier_fail_cycles_);
  lantern_merge_dist_m_ = std::max(0.1, lantern_merge_dist_m_);
  heading_hint_min_speed_ = std::max(0.01, heading_hint_min_speed_);
  adaptive_step_clearance_range_m_ = std::max(0.2, adaptive_step_clearance_range_m_);
  adaptive_step_turn_weight_ = std::clamp(adaptive_step_turn_weight_, 0.0, 1.0);
  adaptive_step_min_ratio_ = std::clamp(adaptive_step_min_ratio_, 0.1, 1.0);
  traj_start_speed_max_ratio_ = std::clamp(traj_start_speed_max_ratio_, 0.0, 1.0);
  traj_start_align_cos_min_ = std::clamp(traj_start_align_cos_min_, -1.0, 1.0);
  traj_time_scale_ = std::clamp(traj_time_scale_, 1.0, 3.0);
  marker_max_candidates_ = std::max(10, marker_max_candidates_);

  graph_backbone_spacing_m_ = std::max(0.5, graph_backbone_spacing_m_);
  graph_explore_spacing_m_ = std::max(0.3, graph_explore_spacing_m_);
  graph_merge_dist_m_ = std::max(0.2, graph_merge_dist_m_);
  graph_junction_probe_dist_m_ = std::max(0.5, graph_junction_probe_dist_m_);
  graph_branch_goal_dist_m_ = std::max(0.8, graph_branch_goal_dist_m_);
  graph_branch_merge_dist_m_ = std::max(0.2, graph_branch_merge_dist_m_);
  graph_frontier_revisit_penalty_ = std::max(0.0, graph_frontier_revisit_penalty_);
  graph_forward_bias_weight_ = std::max(0.0, graph_forward_bias_weight_);
  graph_vertical_penalty_weight_ = std::max(0.0, graph_vertical_penalty_weight_);
  graph_view_align_weight_ = std::max(0.0, graph_view_align_weight_);
  graph_center_search_radius_m_ = std::max(0.5, graph_center_search_radius_m_);
  graph_min_junction_free_dirs_ = std::clamp(graph_min_junction_free_dirs_, 2, 12);
  graph_max_frontiers_per_node_ = std::clamp(graph_max_frontiers_per_node_, 1, 30);
  centerline_horizon_points_ = std::clamp(centerline_horizon_points_, 2, 16);
  centerline_step_m_ = std::clamp(centerline_step_m_, 1.0, 10.0);
  centerline_slice_half_extent_m_ = std::clamp(centerline_slice_half_extent_m_, 1.0, 10.0);
  centerline_slice_step_m_ = std::clamp(centerline_slice_step_m_, 0.2, 1.5);
  centerline_min_component_cells_ = std::clamp(centerline_min_component_cells_, 1, 100);
  centerline_max_secondary_goals_ = std::clamp(centerline_max_secondary_goals_, 0, 10);
  centerline_execute_dist_m_ = std::clamp(centerline_execute_dist_m_, 2.0, 60.0);
  centerline_min_commit_dist_m_ = std::clamp(
    centerline_min_commit_dist_m_, 1.0, std::max(1.0, centerline_execute_dist_m_ - 0.5));
  body_yaw_correction_deg_ = std::clamp(body_yaw_correction_deg_, -360.0, 360.0);
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
    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::TAKEOFF):
      if (msg->has_target) {
        seedHomeNodeFromCommand(msg->target_pos);
        RCLCPP_INFO(this->get_logger(),
          "[cmd] TAKEOFF seed graph home=(%.2f, %.2f, %.2f)",
          msg->target_pos.x, msg->target_pos.y, msg->target_pos.z);
      }
      break;

    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::START):
      if (mode_ != PlannerMode::EXPLORE || planner_done_) {
        setActiveBranchStatus(BranchStatus::OPEN, true);
        mode_ = PlannerMode::EXPLORE;
        planner_done_ = false;
        has_current_goal_ = false;
        frontier_fail_cycles_ = 0;
        prefer_local_expansion_ = false;
        local_expansion_node_id_ = -1;
        prefer_view_aligned_entry_ = true;
        planned_forward_hint_.setZero();
        active_path_points_.clear();
        debug_candidate_points_.clear();
        RCLCPP_INFO(this->get_logger(), "[cmd] START -> EXPLORE");
      }
      break;

    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::RETURN_HOME):
    {
      const bool mode_changed = (mode_ != PlannerMode::RETURN_HOME);
      if (mode_changed) {
        setActiveBranchStatus(BranchStatus::OPEN, true);
        mode_ = PlannerMode::RETURN_HOME;
        planner_done_ = false;
        has_current_goal_ = false;
        frontier_fail_cycles_ = 0;
        prefer_local_expansion_ = false;
        local_expansion_node_id_ = -1;
        prefer_view_aligned_entry_ = false;
        planned_forward_hint_.setZero();
        active_path_points_.clear();
        debug_candidate_points_.clear();
      }
      if (msg->has_target) {
        seedHomeNodeFromCommand(msg->target_pos);
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
        setActiveBranchStatus(BranchStatus::OPEN, true);
        mode_ = PlannerMode::IDLE;
        has_current_goal_ = false;
        prefer_local_expansion_ = false;
        local_expansion_node_id_ = -1;
        prefer_view_aligned_entry_ = false;
        planned_forward_hint_.setZero();
        active_path_points_.clear();
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

  const Eigen::Quaterniond q(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z);
  constexpr double kDegToRad = 0.01745329251994329576923690768489;
  const double yaw_correction_rad = body_yaw_correction_deg_ * kDegToRad;
  const Eigen::Quaterniond yaw_correction(
    Eigen::AngleAxisd(yaw_correction_rad, Eigen::Vector3d::UnitZ()));
  const Eigen::Quaterniond corrected_q = q * yaw_correction;
  Eigen::Vector3d forward = corrected_q * Eigen::Vector3d::UnitX();
  forward.z() = 0.0;
  if (forward.norm() > 1e-3) {
    body_forward_hint_ = forward.normalized();
  }

  const double vxy = std::hypot(current_velocity_.x(), current_velocity_.y());
  if (vxy >= heading_hint_min_speed_) {
    heading_hint_ =
      Eigen::Vector3d(current_velocity_.x() / vxy, current_velocity_.y() / vxy, 0.0);
  } else if (heading_hint_.norm() < 1e-3 && body_forward_hint_.norm() > 1e-3) {
    heading_hint_ = body_forward_hint_;
  }

  has_current_pose_ = true;

  // Step 3: Keep a fallback home position if no explicit seed was received yet.
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

void PathPlannerNode::seedHomeNodeFromCommand(const geometry_msgs::msg::Point &target)
{
  // Step 1: Cache command-provided home seed.
  home_seed_position_ = Eigen::Vector3d(target.x, target.y, target.z);
  has_home_seed_ = true;
  home_position_ = home_seed_position_;
  has_home_pose_ = true;

  // Step 2: Bind/insert home node into graph.
  const int near_home = findClosestGraphNode(home_seed_position_, graph_merge_dist_m_ * 1.5);
  if (near_home >= 0) {
    home_node_id_ = near_home;
  } else {
    home_node_id_ = addOrMergeGraphNode(home_seed_position_, false, false);
  }

  graph_nodes_[home_node_id_].is_home = true;
  if (last_graph_node_id_ < 0) {
    last_graph_node_id_ = home_node_id_;
  }
}

void PathPlannerNode::onPlannerTimer()
{
  // Step 1: Require current pose + map.
  if (!hasValidMapAndPose()) {
    return;
  }

  // Step 2: Keep graph updated in every mission phase.
  updateGraphModel();

  // Step 3: Mark active branch as visited when current goal was reached.
  if (has_current_goal_ && hasReachedGoal()) {
    const int source_node = active_branch_node_id_;
    const int follow_node = addOrMergeGraphNode(current_position_, true, false);
    if (source_node >= 0 && follow_node >= 0 && source_node != follow_node) {
      connectGraphNodes(source_node, follow_node);
      previous_graph_node_id_ = source_node;
    }
    if (follow_node >= 0 && follow_node < static_cast<int>(graph_nodes_.size())) {
      current_node_id_ = follow_node;
      last_graph_node_id_ = follow_node;
      refreshBranchesAroundNode(follow_node);
    }
    if (source_node >= 0 && source_node < static_cast<int>(graph_nodes_.size())) {
      refreshBranchesAroundNode(source_node);
    }

    storeVisitedGoal(current_goal_);
    markActiveBranchVisited();
    has_current_goal_ = false;
    active_path_points_.clear();
    prefer_local_expansion_ = true;
    local_expansion_node_id_ = current_node_id_;
  }

  // Step 4: Exit when planner is inactive or done.
  if (mode_ == PlannerMode::IDLE || planner_done_) {
    return;
  }

  // Step 5: Respect replan timeout to avoid publish spam.
  const auto now = this->now();
  const bool timed_out =
    (last_plan_time_.nanoseconds() > 0) && ((now - last_plan_time_).seconds() >= replan_timeout_sec_);

  // Step 5a: Replan immediately when the active path became invalid due to map updates.
  if (has_current_goal_ && !active_path_points_.empty() && !isPathSafe(active_path_points_)) {
    setActiveBranchStatus(BranchStatus::OPEN, true);
    has_current_goal_ = false;
    active_path_points_.clear();
  }

  if (has_current_goal_ && timed_out) {
    if (isFollowingActivePath()) {
      double dist_to_path = 0.0;
      double progress_ratio = 0.0;
      double remaining_dist = 0.0;
      bool has_metrics = pathProgressMetrics(
        current_position_, active_path_points_, dist_to_path, progress_ratio, remaining_dist);

      Eigen::Vector3d desired_dir = Eigen::Vector3d::Zero();
      const bool has_desired_dir = computeDesiredDirection(current_node_id_, desired_dir);
      const double front_depth = has_desired_dir
        ? estimateForwardObstacleDistance(current_position_, desired_dir, 20.0)
        : 20.0;

      const bool can_defer_replan =
        has_metrics &&
        progress_ratio < active_path_replan_progress_ratio_ &&
        front_depth > active_path_replan_front_obstacle_m_;

      if (can_defer_replan) {
        // Keep current plan active while vehicle is still in the early path section.
        // Intentionally do not reset last_plan_time_ so we can re-evaluate every planner tick.
        return;
      }
    }

    setActiveBranchStatus(BranchStatus::OPEN, true);
    has_current_goal_ = false;
    active_path_points_.clear();
  }

  if (has_current_goal_ && !timed_out) {
    return;
  }

  // Step 6: Choose next graph route.
  std::vector<Eigen::Vector3d> route_points;
  std::string reason;
  if (!chooseGoal(route_points, reason)) {
    frontier_fail_cycles_++;

    if (shouldReportDone()) {
      planner_done_ = true;
      mode_ = PlannerMode::IDLE;
      has_current_goal_ = false;
      setActiveBranchStatus(BranchStatus::OPEN, true);
      prefer_local_expansion_ = false;
      local_expansion_node_id_ = -1;
      RCLCPP_INFO(this->get_logger(),
        "exploration done: graph exhausted, nodes=%zu, lanterns=%zu",
        graph_nodes_.size(), tracked_lanterns_.size());
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "No valid graph route found yet (fail_cycles=%d, graph_nodes=%zu).",
        frontier_fail_cycles_, graph_nodes_.size());
    }
    return;
  }

  // Step 7: Build geometric path from graph route segments.
  std::vector<Eigen::Vector3d> path;
  if (!buildPathFromRoute(route_points, path)) {
    frontier_fail_cycles_++;
    if (active_branch_node_id_ >= 0 && active_branch_node_id_ < static_cast<int>(graph_nodes_.size())) {
      setActiveBranchStatus(BranchStatus::BLOCKED, false);
      updateNodeClassification(active_branch_node_id_);
    }
    setActiveBranchStatus(BranchStatus::BLOCKED, true);
    prefer_local_expansion_ = false;
    local_expansion_node_id_ = -1;

    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Graph route planning failed (fail_cycles=%d).", frontier_fail_cycles_);
    return;
  }

  // Step 8: Convert geometric path to compact waypoint chain.
  std::vector<Eigen::Vector3d> waypoints;
  if (!resamplePath(path, waypoints)) {
    frontier_fail_cycles_++;
    setActiveBranchStatus(BranchStatus::OPEN, true);
    prefer_local_expansion_ = false;
    local_expansion_node_id_ = -1;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Path resampling failed (fail_cycles=%d).", frontier_fail_cycles_);
    return;
  }

  // Step 9: Generate and publish polynomial trajectory.
  if (!buildAndPublishTrajectory(waypoints, true)) {
    frontier_fail_cycles_++;
    setActiveBranchStatus(BranchStatus::OPEN, true);
    prefer_local_expansion_ = false;
    local_expansion_node_id_ = -1;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
      "Trajectory generation failed (fail_cycles=%d).", frontier_fail_cycles_);
    return;
  }

  // Step 10: Publish selected path/debug markers.
  publishSelectedPath(path);
  publishPlanningVisualization(path, waypoints);

  // Step 11: Store planning state.
  active_path_points_ = path;
  planned_forward_hint_.setZero();
  for (size_t i = 1; i < path.size(); ++i) {
    Eigen::Vector3d dir = path[i] - path[i - 1];
    dir.z() = 0.0;
    if (dir.norm() > 0.3) {
      planned_forward_hint_ = dir.normalized();
      break;
    }
  }

  current_goal_ = route_points.back();
  has_current_goal_ = true;
  frontier_fail_cycles_ = 0;
  last_plan_time_ = now;

  RCLCPP_INFO(this->get_logger(),
    "Trajectory published: mode=%u reason=%s goal=(%.2f, %.2f, %.2f) route_pts=%zu waypoints=%zu",
    static_cast<unsigned>(mode_), reason.c_str(),
    current_goal_.x(), current_goal_.y(), current_goal_.z(), route_points.size(), waypoints.size());
}

void PathPlannerNode::onHeartbeatTimer()
{
  // Step 1: Build heartbeat state.
  if (planner_done_) {
    publishHeartbeat(static_cast<uint8_t>(statemachine_pkg::protocol::AnswerStates::DONE),
      "DONE_GRAPH_EXHAUSTED");
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

bool PathPlannerNode::chooseGoal(
  std::vector<Eigen::Vector3d> &route_points_out,
  std::string &reason_out)
{
  if (mode_ == PlannerMode::RETURN_HOME) {
    return chooseReturnGoal(route_points_out, reason_out);
  }

  if (mode_ != PlannerMode::EXPLORE) {
    return false;
  }

  return chooseExploreGoal(route_points_out, reason_out);
}

bool PathPlannerNode::chooseExploreGoal(
  std::vector<Eigen::Vector3d> &route_points_out,
  std::string &reason_out)
{
  if (graph_nodes_.empty()) {
    return false;
  }

  // Step 1: Refresh branch sets before selecting next target.
  for (size_t i = 0; i < graph_nodes_.size(); ++i) {
    refreshBranchesAroundNode(static_cast<int>(i));
  }

  // Step 2: Resolve planner graph start node.
  int start_node = current_node_id_;
  if (start_node < 0 || start_node >= static_cast<int>(graph_nodes_.size())) {
    start_node = findClosestGraphNode(current_position_, std::numeric_limits<double>::infinity());
  }
  if (start_node < 0) {
    return false;
  }

  // Step 2b: Build a multi-point forward preview and register preview branches.
  std::vector<Eigen::Vector3d> preview_centerline_points;
  std::vector<Eigen::Vector3d> preview_secondary_goals;
  Eigen::Vector3d preview_primary_goal = Eigen::Vector3d::Zero();
  const bool has_preview = buildForwardCenterlinePreview(
    start_node,
    preview_centerline_points,
    preview_primary_goal,
    preview_secondary_goals);
  int preview_branch_index = -1;

  auto upsert_start_branch = [&](const Eigen::Vector3d &goal, bool force_open) {
    if (start_node < 0 || start_node >= static_cast<int>(graph_nodes_.size())) {
      return -1;
    }
    auto &start = graph_nodes_[static_cast<size_t>(start_node)];
    if (!isSegmentSafe(start.position, goal)) {
      return -1;
    }

    int best_idx = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < start.branches.size(); ++i) {
      const double d = distance3(start.branches[i].goal, goal);
      if (d <= 1.4 * graph_branch_merge_dist_m_ && d < best_dist) {
        best_dist = d;
        best_idx = static_cast<int>(i);
      }
    }

    if (best_idx >= 0) {
      auto &branch = start.branches[static_cast<size_t>(best_idx)];
      branch.goal = goal;
      if (force_open || branch.status == BranchStatus::BLOCKED || branch.status == BranchStatus::VISITED) {
        branch.status = BranchStatus::OPEN;
      }
      return best_idx;
    }

    if (static_cast<int>(start.branches.size()) >= graph_max_frontiers_per_node_) {
      return -1;
    }

    GraphBranch branch;
    branch.goal = goal;
    branch.status = BranchStatus::OPEN;
    branch.target_node = -1;
    start.branches.push_back(branch);
    return static_cast<int>(start.branches.size()) - 1;
  };

  if (has_preview) {
    preview_branch_index = upsert_start_branch(preview_primary_goal, true);
    for (const auto &goal : preview_secondary_goals) {
      (void)upsert_start_branch(goal, false);
    }
    seedCenterlinePreviewNodes(start_node, preview_centerline_points);
    updateNodeClassification(start_node);
  }

  // Step 2c: Prefer direct forward-centerline execution whenever a valid preview exists.
  if (has_preview &&
      preview_branch_index >= 0 &&
      preview_branch_index < static_cast<int>(graph_nodes_[static_cast<size_t>(start_node)].branches.size()))
  {
    route_points_out.clear();
    route_points_out.reserve(preview_centerline_points.size());

    double accum_dist = 0.0;
    bool committed = false;
    Eigen::Vector3d previous = current_position_;
    for (const auto &p : preview_centerline_points) {
      const double step_dist = distance3(previous, p);
      if (step_dist < 0.1) {
        continue;
      }

      accum_dist += step_dist;
      previous = p;
      if (accum_dist + 1e-6 < centerline_min_commit_dist_m_) {
        continue;
      }

      route_points_out.push_back(p);
      committed = true;
      if (accum_dist >= centerline_execute_dist_m_) {
        break;
      }
    }

    if (!committed && !preview_centerline_points.empty()) {
      route_points_out.push_back(preview_centerline_points.back());
    }

    if (!route_points_out.empty()) {
      debug_candidate_points_.clear();
      debug_candidate_points_.reserve(
        preview_centerline_points.size() + preview_secondary_goals.size() + 1);
      debug_candidate_points_.push_back(preview_primary_goal);
      for (const auto &p : preview_centerline_points) {
        debug_candidate_points_.push_back(p);
      }
      for (const auto &p : preview_secondary_goals) {
        debug_candidate_points_.push_back(p);
      }

      auto &branch = graph_nodes_[static_cast<size_t>(start_node)].branches[static_cast<size_t>(preview_branch_index)];
      branch.status = BranchStatus::ASSIGNED;
      active_branch_node_id_ = start_node;
      active_branch_index_ = preview_branch_index;
      active_branch_goal_ = branch.goal;
      has_active_branch_goal_ = true;
      reason_out = "forward_centerline";
      return true;
    }
  }

  // Step 3: Evaluate all open frontiers in graph.
  struct Candidate {
    int node_id{-1};
    int branch_index{-1};
    std::vector<int> route_nodes;
    double cost{std::numeric_limits<double>::infinity()};
  };

  Candidate best;
  std::vector<std::pair<double, Eigen::Vector3d>> candidate_debug;
  candidate_debug.reserve(static_cast<size_t>(marker_max_candidates_));

  Eigen::Vector3d desired_dir = Eigen::Vector3d::Zero();
  const bool has_desired_dir = computeDesiredDirection(start_node, desired_dir);
  const double forward_obstacle_dist = has_desired_dir
    ? estimateForwardObstacleDistance(current_position_, desired_dir, 30.0)
    : 30.0;
  const double wall_pressure = std::clamp((12.0 - forward_obstacle_dist) / 12.0, 0.0, 1.0);
  const double dynamic_forward_weight = graph_forward_bias_weight_ * (1.0 - 0.35 * wall_pressure);

  Eigen::Vector3d steered_dir = desired_dir;
  if (has_desired_dir && wall_pressure > 1e-3) {
    const Eigen::Vector3d lateral(-desired_dir.y(), desired_dir.x(), 0.0);
    Eigen::Vector3d left_dir = desired_dir + 0.35 * lateral;
    Eigen::Vector3d right_dir = desired_dir - 0.35 * lateral;
    if (left_dir.norm() > 1e-3) {
      left_dir.normalize();
    }
    if (right_dir.norm() > 1e-3) {
      right_dir.normalize();
    }

    const double left_dist = estimateForwardObstacleDistance(current_position_, left_dir, 20.0);
    const double right_dist = estimateForwardObstacleDistance(current_position_, right_dir, 20.0);
    const double denom = std::max(0.1, left_dist + right_dist);
    const double side_balance = (left_dist - right_dist) / denom;
    const double steer_gain = 0.9 * wall_pressure;

    steered_dir = desired_dir + steer_gain * side_balance * lateral;
    steered_dir.z() = 0.0;
    if (steered_dir.norm() > 1e-3) {
      steered_dir.normalize();
    } else {
      steered_dir = desired_dir;
    }
  }

  auto push_debug_candidate = [&](double cost, const Eigen::Vector3d &p) {
    if (marker_max_candidates_ <= 0) {
      return;
    }
    candidate_debug.emplace_back(cost, p);
    std::sort(candidate_debug.begin(), candidate_debug.end(),
      [](const auto &a, const auto &b) { return a.first < b.first; });
    if (static_cast<int>(candidate_debug.size()) > marker_max_candidates_) {
      candidate_debug.pop_back();
    }
  };

  auto evaluate_branch = [&](int node_idx, int branch_idx) {
    if (node_idx < 0 || node_idx >= static_cast<int>(graph_nodes_.size())) {
      return;
    }
    auto &node = graph_nodes_[static_cast<size_t>(node_idx)];
    if (branch_idx < 0 || branch_idx >= static_cast<int>(node.branches.size())) {
      return;
    }

    const auto &branch = node.branches[static_cast<size_t>(branch_idx)];
    if (branch.status != BranchStatus::OPEN) {
      return;
    }

    if (node_idx == start_node) {
      const double dist_from_current = distance3(current_position_, branch.goal);
      if (dist_from_current < std::max(1.2, 0.75 * graph_explore_spacing_m_)) {
        return;
      }

      if (has_desired_dir) {
        Eigen::Vector3d local_dir = branch.goal - current_position_;
        local_dir.z() = 0.0;
        if (local_dir.norm() > 1e-3) {
          local_dir.normalize();
          const double align = steered_dir.dot(local_dir);
          const double min_align = has_preview ? -0.05 : -0.25;
          if (align < min_align) {
            return;
          }
        }
      }
    }

    std::vector<int> route_nodes;
    if (!computeShortestNodePath(start_node, node_idx, route_nodes)) {
      return;
    }

    const double route_cost = computeNodePathLength(route_nodes);
    const double branch_cost = distance3(node.position, branch.goal);
    const double revisit_cost = revisitPenalty(branch.goal);
    const double vertical_penalty = std::abs(branch.goal.z() - node.position.z());

    double forward_bonus = 0.0;
    double lateral_penalty = 0.0;
    if (has_desired_dir) {
      Eigen::Vector3d dir = branch.goal - current_position_;
      dir.z() = 0.0;
      if (dir.norm() > 1e-3) {
        dir.normalize();
        forward_bonus = std::max(0.0, steered_dir.dot(dir));
        const double cross = steered_dir.x() * dir.y() - steered_dir.y() * dir.x();
        lateral_penalty = std::abs(cross);
      }
    }

    const double node_visit_penalty = node.visited ? 0.3 : 0.0;
    const double total_cost =
      route_cost +
      branch_cost +
      graph_vertical_penalty_weight_ * vertical_penalty +
      graph_frontier_revisit_penalty_ * revisit_cost +
      graph_view_align_weight_ * lateral_penalty +
      node_visit_penalty -
      dynamic_forward_weight * forward_bonus;

    push_debug_candidate(total_cost, branch.goal);

    if (total_cost < best.cost) {
      best.cost = total_cost;
      best.node_id = node_idx;
      best.branch_index = branch_idx;
      best.route_nodes = std::move(route_nodes);
    }
  };

  // Step 3a: On EXPLORE entry, prefer a branch aligned with current view direction.
  bool candidate_found = false;
  if (prefer_view_aligned_entry_) {
    int entry_branch_idx = -1;
    if (chooseEntryBranchFromView(start_node, entry_branch_idx)) {
      evaluate_branch(start_node, entry_branch_idx);
      candidate_found = (best.node_id >= 0);
    }
    prefer_view_aligned_entry_ = false;
  }

  // Step 3b: After reaching a branch goal, prefer local continuation once.
  if (!candidate_found &&
      prefer_local_expansion_ &&
      local_expansion_node_id_ >= 0 &&
      local_expansion_node_id_ < static_cast<int>(graph_nodes_.size()))
  {
    const auto &local_node = graph_nodes_[static_cast<size_t>(local_expansion_node_id_)];
    for (size_t branch_idx = 0; branch_idx < local_node.branches.size(); ++branch_idx) {
      evaluate_branch(local_expansion_node_id_, static_cast<int>(branch_idx));
    }
    candidate_found = (best.node_id >= 0);
    prefer_local_expansion_ = false;
    local_expansion_node_id_ = -1;
  }

  // Step 3c: Fallback to global frontier search when no local continuation exists.
  if (!candidate_found) {
    for (size_t node_idx = 0; node_idx < graph_nodes_.size(); ++node_idx) {
      auto &node = graph_nodes_[node_idx];
      for (size_t branch_idx = 0; branch_idx < node.branches.size(); ++branch_idx) {
        evaluate_branch(static_cast<int>(node_idx), static_cast<int>(branch_idx));
      }
    }
  }

  debug_candidate_points_.clear();
  debug_candidate_points_.reserve(candidate_debug.size());
  for (const auto &entry : candidate_debug) {
    debug_candidate_points_.push_back(entry.second);
  }

  if (best.node_id < 0 || best.branch_index < 0) {
    return false;
  }

  // Step 4: Build route point chain (graph path + selected branch goal).
  route_points_out.clear();
  route_points_out.reserve(best.route_nodes.size() + 1);
  for (size_t i = 1; i < best.route_nodes.size(); ++i) {
    route_points_out.push_back(graph_nodes_[best.route_nodes[i]].position);
  }

  auto &selected_node = graph_nodes_[best.node_id];
  auto &selected_branch = selected_node.branches[best.branch_index];
  const bool selected_preview_branch =
    has_preview &&
    best.node_id == start_node &&
    preview_branch_index >= 0 &&
    best.branch_index == preview_branch_index;

  if (selected_preview_branch) {
    // Step 4a: Commit to a forward sub-route (10m+) and cap execution distance (~15m).
    double accum_dist = 0.0;
    bool committed = false;
    Eigen::Vector3d previous = current_position_;
    for (const auto &p : preview_centerline_points) {
      const double step_dist = distance3(previous, p);
      if (step_dist < 0.1) {
        continue;
      }

      accum_dist += step_dist;
      previous = p;
      if (accum_dist + 1e-6 < centerline_min_commit_dist_m_) {
        continue;
      }

      route_points_out.push_back(p);
      committed = true;
      if (accum_dist >= centerline_execute_dist_m_) {
        break;
      }
    }

    if (!committed && !preview_centerline_points.empty()) {
      route_points_out.push_back(preview_centerline_points.back());
    }
  } else {
    route_points_out.push_back(selected_branch.goal);
  }

  // Step 5: Mark selected branch as active.
  selected_branch.status = BranchStatus::ASSIGNED;
  active_branch_node_id_ = best.node_id;
  active_branch_index_ = best.branch_index;
  active_branch_goal_ = selected_branch.goal;
  has_active_branch_goal_ = true;

  reason_out = "graph_frontier";
  return !route_points_out.empty();
}

bool PathPlannerNode::chooseReturnGoal(
  std::vector<Eigen::Vector3d> &route_points_out,
  std::string &reason_out) const
{
  route_points_out.clear();

  // Step 1: Prefer graph-home routing when available.
  if (home_node_id_ >= 0 && home_node_id_ < static_cast<int>(graph_nodes_.size())) {
    int start_node = current_node_id_;
    if (start_node < 0 || start_node >= static_cast<int>(graph_nodes_.size())) {
      start_node = findClosestGraphNode(current_position_, std::numeric_limits<double>::infinity());
    }

    if (start_node >= 0) {
      std::vector<int> route_nodes;
      if (computeShortestNodePath(start_node, home_node_id_, route_nodes)) {
        for (size_t i = 1; i < route_nodes.size(); ++i) {
          route_points_out.push_back(graph_nodes_[route_nodes[i]].position);
        }
      }
    }

    if (route_points_out.empty()) {
      route_points_out.push_back(graph_nodes_[home_node_id_].position);
    }

    reason_out = "return_home_graph";
    return true;
  }

  // Step 2: Fallback to cached home pose.
  if (!has_home_pose_) {
    return false;
  }
  route_points_out.push_back(home_position_);
  reason_out = "return_home_pose";
  return true;
}

bool PathPlannerNode::buildPathFromRoute(
  const std::vector<Eigen::Vector3d> &route_points,
  std::vector<Eigen::Vector3d> &path_out) const
{
  if (route_points.empty()) {
    return false;
  }

  // Step 1: Plan each route segment and concatenate OMPL path pieces.
  path_out.clear();
  Eigen::Vector3d segment_start = current_position_;
  path_out.push_back(segment_start);

  bool added_any_segment = false;
  for (const auto &segment_goal : route_points) {
    if (distance3(segment_start, segment_goal) < 0.1) {
      continue;
    }

    std::vector<Eigen::Vector3d> segment_path;
    if (!planPathOmpl(segment_start, segment_goal, segment_path)) {
      return false;
    }
    if (segment_path.size() < 2) {
      return false;
    }

    for (size_t i = 1; i < segment_path.size(); ++i) {
      path_out.push_back(segment_path[i]);
    }

    segment_start = segment_goal;
    added_any_segment = true;
  }

  if (!added_any_segment) {
    return false;
  }

  return isPathSafe(path_out);
}

void PathPlannerNode::updateGraphModel()
{
  // Step 1: Create home node from command seed as soon as map/pose are valid.
  if (has_home_seed_ && home_node_id_ < 0) {
    const int idx = addOrMergeGraphNode(home_seed_position_, false, false);
    home_node_id_ = idx;
    graph_nodes_[idx].is_home = true;
    home_position_ = graph_nodes_[idx].position;
    has_home_pose_ = true;
  }

  // Step 2: Ensure graph has at least one node.
  if (graph_nodes_.empty()) {
    const Eigen::Vector3d first = has_home_seed_ ? home_seed_position_ : current_position_;
    const int idx = addOrMergeGraphNode(first, true, false);

    if (home_node_id_ < 0) {
      home_node_id_ = idx;
      graph_nodes_[idx].is_home = true;
      home_position_ = graph_nodes_[idx].position;
      has_home_pose_ = true;
    }

    previous_graph_node_id_ = -1;
    last_graph_node_id_ = idx;
    current_node_id_ = idx;
    refreshBranchesAroundNode(idx);
  }

  // Step 3: Detect if current position should become a new graph node.
  const double spacing =
    (mode_ == PlannerMode::EXPLORE) ? graph_explore_spacing_m_ : graph_backbone_spacing_m_;

  const int near_node = findClosestGraphNode(current_position_, graph_merge_dist_m_);
  const auto local_branch_goals = sampleBranchGoals(current_position_);
  const bool is_junction_candidate =
    static_cast<int>(local_branch_goals.size()) >= graph_min_junction_free_dirs_;

  bool should_insert = false;
  if (last_graph_node_id_ < 0 || last_graph_node_id_ >= static_cast<int>(graph_nodes_.size())) {
    should_insert = true;
  } else {
    const double dist_last = distance3(current_position_, graph_nodes_[last_graph_node_id_].position);
    if (dist_last >= spacing) {
      should_insert = true;
    }
    if (is_junction_candidate && dist_last >= 0.5 * graph_merge_dist_m_) {
      should_insert = true;
    }
  }

  int node_id = near_node;
  if (should_insert || near_node < 0) {
    node_id = addOrMergeGraphNode(current_position_, true, is_junction_candidate);
  }

  if (node_id >= 0 && node_id != last_graph_node_id_) {
    if (last_graph_node_id_ >= 0 && last_graph_node_id_ < static_cast<int>(graph_nodes_.size())) {
      connectGraphNodes(last_graph_node_id_, node_id);
      previous_graph_node_id_ = last_graph_node_id_;
    }
    last_graph_node_id_ = node_id;
  }

  if (node_id >= 0 && node_id < static_cast<int>(graph_nodes_.size())) {
    current_node_id_ = node_id;
    graph_nodes_[node_id].visited = true;
    graph_nodes_[node_id].is_junction = graph_nodes_[node_id].is_junction || is_junction_candidate;
    refreshBranchesAroundNode(node_id);

    for (int neighbor_id : graph_nodes_[node_id].neighbors) {
      refreshBranchesAroundNode(neighbor_id);
    }
  }

  if (home_node_id_ >= 0 && home_node_id_ < static_cast<int>(graph_nodes_.size())) {
    refreshBranchesAroundNode(home_node_id_);
  }
}

int PathPlannerNode::addOrMergeGraphNode(
  const Eigen::Vector3d &p,
  bool mark_visited,
  bool prefer_junction)
{
  Eigen::Vector3d candidate = p;
  (void)recenterToLocalMidpoint(candidate, graph_center_search_radius_m_);

  // Step 1: Merge with existing nearby node if possible.
  const int existing = findClosestGraphNode(candidate, graph_merge_dist_m_);
  if (existing >= 0) {
    auto &node = graph_nodes_[existing];
    if (!node.is_home) {
      node.position = 0.75 * node.position + 0.25 * candidate;
      (void)recenterToLocalMidpoint(node.position, std::max(0.8, 0.6 * graph_center_search_radius_m_));
    }
    node.visited = node.visited || mark_visited;
    node.is_junction = node.is_junction || prefer_junction;
    return existing;
  }

  // Step 2: Add a new node.
  GraphNode node;
  node.id = static_cast<int>(graph_nodes_.size());
  node.position = candidate;
  node.visited = mark_visited;
  node.is_junction = prefer_junction;
  graph_nodes_.push_back(node);

  return node.id;
}

void PathPlannerNode::connectGraphNodes(int a, int b)
{
  // Step 1: Validate node ids.
  if (a < 0 || b < 0 || a == b) {
    return;
  }
  if (a >= static_cast<int>(graph_nodes_.size()) || b >= static_cast<int>(graph_nodes_.size())) {
    return;
  }

  // Step 2: Add only safe edges.
  if (!isSegmentSafe(graph_nodes_[a].position, graph_nodes_[b].position)) {
    return;
  }

  auto add_unique_neighbor = [&](int from, int to) {
    auto &neighbors = graph_nodes_[from].neighbors;
    if (std::find(neighbors.begin(), neighbors.end(), to) == neighbors.end()) {
      neighbors.push_back(to);
    }
  };

  add_unique_neighbor(a, b);
  add_unique_neighbor(b, a);
}

int PathPlannerNode::findClosestGraphNode(const Eigen::Vector3d &p, double max_dist) const
{
  // Step 1: Handle empty graph.
  if (graph_nodes_.empty()) {
    return -1;
  }

  // Step 2: Scan nearest node.
  int best_id = -1;
  double best_dist = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < graph_nodes_.size(); ++i) {
    const double d = distance3(graph_nodes_[i].position, p);
    if (d < best_dist) {
      best_dist = d;
      best_id = static_cast<int>(i);
    }
  }

  if (best_id < 0 || best_dist > max_dist) {
    return -1;
  }
  return best_id;
}

void PathPlannerNode::refreshBranchesAroundNode(int node_id)
{
  // Step 1: Validate node id.
  if (node_id < 0 || node_id >= static_cast<int>(graph_nodes_.size())) {
    return;
  }

  auto &node = graph_nodes_[node_id];

  // Step 2: Sample currently free branch goals around this node.
  const auto sampled_goals = sampleBranchGoals(node.position);
  std::vector<bool> matched(node.branches.size(), false);
  std::vector<GraphBranch> merged_branches;
  merged_branches.reserve(node.branches.size() + sampled_goals.size());

  for (const auto &goal : sampled_goals) {
    bool toward_existing_neighbor = false;
    for (int neighbor_id : node.neighbors) {
      if (neighbor_id < 0 || neighbor_id >= static_cast<int>(graph_nodes_.size())) {
        continue;
      }
      if (distance3(goal, graph_nodes_[neighbor_id].position) <= graph_branch_merge_dist_m_ * 1.5) {
        toward_existing_neighbor = true;
        break;
      }
    }
    if (toward_existing_neighbor) {
      continue;
    }

    int matched_idx = -1;
    double matched_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < node.branches.size(); ++i) {
      const double d = distance3(goal, node.branches[i].goal);
      if (d <= graph_branch_merge_dist_m_ && d < matched_dist) {
        matched_dist = d;
        matched_idx = static_cast<int>(i);
      }
    }

    GraphBranch branch;
    if (matched_idx >= 0) {
      branch = node.branches[static_cast<size_t>(matched_idx)];
      branch.goal = goal;
      matched[static_cast<size_t>(matched_idx)] = true;
      const bool is_active_assigned =
        (node_id == active_branch_node_id_) &&
        has_active_branch_goal_ &&
        (distance3(branch.goal, active_branch_goal_) <= graph_branch_merge_dist_m_);
      if (branch.status == BranchStatus::ASSIGNED && !is_active_assigned) {
        branch.status = BranchStatus::OPEN;
      }
      if (branch.status == BranchStatus::BLOCKED) {
        branch.status = BranchStatus::OPEN;
      }
    } else {
      branch.goal = goal;
      branch.status = BranchStatus::OPEN;
      branch.target_node = -1;
    }

    merged_branches.push_back(branch);
  }

  // Step 3: Preserve unmatched branches when they are still relevant.
  for (size_t i = 0; i < node.branches.size(); ++i) {
    if (matched[i]) {
      continue;
    }

    GraphBranch branch = node.branches[i];
    if (branch.status == BranchStatus::VISITED) {
      merged_branches.push_back(branch);
      continue;
    }

    if (
      node_id == active_branch_node_id_ &&
      has_active_branch_goal_ &&
      distance3(branch.goal, active_branch_goal_) <= graph_branch_merge_dist_m_)
    {
      branch.status = BranchStatus::ASSIGNED;
      merged_branches.push_back(branch);
      continue;
    }

    branch.status = isSegmentSafe(node.position, branch.goal) ? BranchStatus::OPEN : BranchStatus::BLOCKED;
    merged_branches.push_back(branch);
  }

  // Step 4: De-duplicate branch list and keep strongest status.
  auto status_rank = [](BranchStatus status) {
    switch (status) {
      case BranchStatus::ASSIGNED:
        return 4;
      case BranchStatus::OPEN:
        return 3;
      case BranchStatus::VISITED:
        return 2;
      case BranchStatus::BLOCKED:
      default:
        return 1;
    }
  };

  std::vector<GraphBranch> deduped;
  deduped.reserve(merged_branches.size());
  for (const auto &branch : merged_branches) {
    int existing_idx = -1;
    for (size_t i = 0; i < deduped.size(); ++i) {
      if (distance3(branch.goal, deduped[i].goal) <= graph_branch_merge_dist_m_) {
        existing_idx = static_cast<int>(i);
        break;
      }
    }

    if (existing_idx < 0) {
      deduped.push_back(branch);
      continue;
    }

    if (status_rank(branch.status) > status_rank(deduped[static_cast<size_t>(existing_idx)].status)) {
      deduped[static_cast<size_t>(existing_idx)] = branch;
    }
  }

  // Step 5: Link branch goals to known nodes and refresh status.
  for (auto &branch : deduped) {
    const int target_node = findClosestGraphNode(branch.goal, graph_merge_dist_m_ * 1.4);
    if (target_node >= 0 && target_node != node_id) {
      branch.target_node = target_node;
      connectGraphNodes(node_id, target_node);
    } else {
      branch.target_node = -1;
    }

    if (!isSegmentSafe(node.position, branch.goal) && branch.status != BranchStatus::ASSIGNED) {
      branch.status = BranchStatus::BLOCKED;
    }
  }

  std::sort(deduped.begin(), deduped.end(), [&](const GraphBranch &a, const GraphBranch &b) {
    return distance3(node.position, a.goal) < distance3(node.position, b.goal);
  });

  if (static_cast<int>(deduped.size()) > graph_max_frontiers_per_node_) {
    deduped.resize(static_cast<size_t>(graph_max_frontiers_per_node_));
  }

  node.branches = std::move(deduped);

  // Step 6: Keep active branch index synchronized.
  if (node_id == active_branch_node_id_) {
    active_branch_index_ = -1;
    if (has_active_branch_goal_) {
      for (size_t i = 0; i < node.branches.size(); ++i) {
        if (distance3(node.branches[i].goal, active_branch_goal_) <= graph_branch_merge_dist_m_) {
          active_branch_index_ = static_cast<int>(i);
          node.branches[i].status = BranchStatus::ASSIGNED;
          break;
        }
      }
    }
  }

  updateNodeClassification(node_id);
}

std::vector<Eigen::Vector3d> PathPlannerNode::sampleBranchGoals(const Eigen::Vector3d &origin) const
{
  std::vector<Eigen::Vector3d> goals;
  if (!octree_) {
    return goals;
  }

  // Step 1: Generate free-direction branch goals around node position.
  const double probe_dist = std::max(graph_junction_probe_dist_m_, 1.5 * wall_clearance_min_);
  const double goal_dist = std::max(graph_branch_goal_dist_m_, probe_dist);

  // Step 1a: Near the current node in EXPLORE mode, keep branch sampling mostly forward.
  bool use_forward_cone = false;
  Eigen::Vector3d desired_dir = Eigen::Vector3d::Zero();
  double min_forward_align = -1.0;
  if (mode_ == PlannerMode::EXPLORE) {
    int ref_node = -1;
    if (current_node_id_ >= 0 && current_node_id_ < static_cast<int>(graph_nodes_.size())) {
      const double near_dist = std::max(1.0, 1.8 * graph_merge_dist_m_);
      if (distance3(graph_nodes_[static_cast<size_t>(current_node_id_)].position, origin) <= near_dist) {
        ref_node = current_node_id_;
      }
    }
    if (ref_node < 0) {
      ref_node = findClosestGraphNode(origin, std::max(2.0, 2.0 * graph_merge_dist_m_));
    }

    if (ref_node >= 0 && computeDesiredDirection(ref_node, desired_dir)) {
      use_forward_cone = true;
      const double front_obstacle_dist = estimateForwardObstacleDistance(origin, desired_dir, 18.0);
      const double wall_pressure = std::clamp((8.0 - front_obstacle_dist) / 8.0, 0.0, 1.0);
      // Open tunnel -> strict forward, near obstacle -> allow side escape.
      min_forward_align = 0.45 - 0.55 * wall_pressure;
    }
  }

  for (const auto &dir_raw : kProbeDirections) {
    Eigen::Vector3d dir = dir_raw;
    if (dir.norm() < 1e-6) {
      continue;
    }
    dir.normalize();

    if (use_forward_cone) {
      Eigen::Vector3d dir_xy(dir.x(), dir.y(), 0.0);
      if (dir_xy.norm() < 1e-3) {
        continue;
      }
      dir_xy.normalize();
      const double align = dir_xy.dot(desired_dir);
      if (align < min_forward_align) {
        continue;
      }
    }

    const Eigen::Vector3d probe = origin + probe_dist * dir;
    if (!isSegmentSafe(origin, probe)) {
      continue;
    }

    Eigen::Vector3d goal = origin + goal_dist * dir;
    if (!projectGoalToFree(goal)) {
      continue;
    }
    if (!recenterToLocalMidpoint(goal, std::max(0.8, 0.7 * graph_center_search_radius_m_))) {
      continue;
    }
    if (!isSegmentSafe(origin, goal)) {
      continue;
    }

    bool duplicate = false;
    for (const auto &existing : goals) {
      if (distance3(existing, goal) <= graph_branch_merge_dist_m_) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) {
      goals.push_back(goal);
    }
  }

  return goals;
}

bool PathPlannerNode::recenterToLocalMidpoint(Eigen::Vector3d &point_in_out, double search_radius) const
{
  if (!octree_) {
    return false;
  }

  // Step 1: Start with input point if it is already safe.
  Eigen::Vector3d best = point_in_out;
  double best_score = -std::numeric_limits<double>::infinity();
  if (isPointSafe(best)) {
    best_score = estimateClearance(best, clearance_probe_radius_m_);
  }

  // Step 2: Search locally for a safer/more central point.
  const double res = octree_->getResolution();
  const double step = std::max(res, 0.2);
  const double radius = std::max(step, search_radius);
  const double max_vertical_shift = 0.8;

  for (double r = 0.0; r <= radius + 1e-6; r += step) {
    for (const auto &dir : kProbeDirections) {
      if (std::abs(dir.z()) > 0.35) {
        continue;
      }
      Eigen::Vector3d candidate = point_in_out + r * dir;
      if (!projectGoalToFree(candidate)) {
        continue;
      }
      if (!isPointSafe(candidate)) {
        continue;
      }
      const double dz = std::abs(candidate.z() - point_in_out.z());
      if (dz > max_vertical_shift) {
        continue;
      }

      const double clearance = estimateClearance(candidate, clearance_probe_radius_m_);
      const double score = clearance - 0.8 * dz;
      if (score > best_score) {
        best = candidate;
        best_score = score;
      }
    }
  }

  if (!std::isfinite(best_score)) {
    return false;
  }

  point_in_out = best;
  return true;
}

bool PathPlannerNode::extractSliceComponentCenters(
  const Eigen::Vector3d &slice_center,
  const Eigen::Vector3d &lateral_axis,
  const Eigen::Vector3d &vertical_axis,
  std::vector<Eigen::Vector3d> &component_centers_out) const
{
  component_centers_out.clear();
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

  const double half_extent = std::max(centerline_slice_half_extent_m_, wall_clearance_min_ + 0.4);
  const double step = std::max(centerline_slice_step_m_, octree_->getResolution());
  const int half_cells = std::max(1, static_cast<int>(std::ceil(half_extent / step)));
  const int size = 2 * half_cells + 1;

  const auto idx_of = [size](int x, int y) {
    return y * size + x;
  };

  std::vector<uint8_t> free_mask(static_cast<size_t>(size * size), 0U);
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      const double u = static_cast<double>(x - half_cells) * step;
      const double v = static_cast<double>(y - half_cells) * step;
      const Eigen::Vector3d p = slice_center + u * lateral + v * vertical;
      if (isPointSafe(p)) {
        free_mask[static_cast<size_t>(idx_of(x, y))] = 1U;
      }
    }
  }

  std::vector<uint8_t> visited(static_cast<size_t>(size * size), 0U);
  for (int y0 = 0; y0 < size; ++y0) {
    for (int x0 = 0; x0 < size; ++x0) {
      const int start_idx = idx_of(x0, y0);
      if (free_mask[static_cast<size_t>(start_idx)] == 0U ||
          visited[static_cast<size_t>(start_idx)] != 0U)
      {
        continue;
      }

      std::queue<std::pair<int, int>> queue;
      queue.emplace(x0, y0);
      visited[static_cast<size_t>(start_idx)] = 1U;

      int count = 0;
      double sum_u = 0.0;
      double sum_v = 0.0;

      while (!queue.empty()) {
        const auto [x, y] = queue.front();
        queue.pop();

        const double u = static_cast<double>(x - half_cells) * step;
        const double v = static_cast<double>(y - half_cells) * step;
        sum_u += u;
        sum_v += v;
        count++;

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
              visited[static_cast<size_t>(nidx)] != 0U)
          {
            continue;
          }

          visited[static_cast<size_t>(nidx)] = 1U;
          queue.emplace(nx, ny);
        }
      }

      if (count < centerline_min_component_cells_) {
        continue;
      }

      Eigen::Vector3d centroid = slice_center +
        (sum_u / static_cast<double>(count)) * lateral +
        (sum_v / static_cast<double>(count)) * vertical;
      if (!projectGoalToFree(centroid)) {
        continue;
      }
      if (!isPointSafe(centroid)) {
        continue;
      }

      component_centers_out.push_back(centroid);
    }
  }

  std::sort(component_centers_out.begin(), component_centers_out.end(), [&](const auto &a, const auto &b) {
    return distance3(a, slice_center) < distance3(b, slice_center);
  });

  return !component_centers_out.empty();
}

bool PathPlannerNode::buildForwardCenterlinePreview(
  int start_node,
  std::vector<Eigen::Vector3d> &centerline_points_out,
  Eigen::Vector3d &primary_goal_out,
  std::vector<Eigen::Vector3d> &secondary_goals_out) const
{
  centerline_points_out.clear();
  secondary_goals_out.clear();
  primary_goal_out = Eigen::Vector3d::Zero();

  if (start_node < 0 || start_node >= static_cast<int>(graph_nodes_.size())) {
    return false;
  }

  Eigen::Vector3d forward = Eigen::Vector3d::Zero();
  if (!computeDesiredDirection(start_node, forward)) {
    return false;
  }

  Eigen::Vector3d lateral(-forward.y(), forward.x(), 0.0);
  if (lateral.norm() < 1e-3) {
    return false;
  }
  lateral.normalize();
  const Eigen::Vector3d vertical = Eigen::Vector3d::UnitZ();

  const auto &start = graph_nodes_[static_cast<size_t>(start_node)];
  Eigen::Vector3d previous = start.position;
  if (distance3(current_position_, start.position) <= std::max(1.5, graph_merge_dist_m_)) {
    previous = current_position_;
  }

  int split_slices = 0;
  for (int i = 0; i < centerline_horizon_points_; ++i) {
    const Eigen::Vector3d predicted = previous + centerline_step_m_ * forward;

    std::vector<Eigen::Vector3d> centers;
    if (!extractSliceComponentCenters(predicted, lateral, vertical, centers)) {
      break;
    }

    double best_score = -std::numeric_limits<double>::infinity();
    Eigen::Vector3d best_center = Eigen::Vector3d::Zero();
    for (const auto &candidate : centers) {
      if (!isSegmentSafe(previous, candidate)) {
        continue;
      }
      Eigen::Vector3d step_dir = candidate - previous;
      const double step_len = step_dir.norm();
      if (step_len < 0.3) {
        continue;
      }
      step_dir /= step_len;

      const double align = forward.dot(step_dir);
      const double side_shift = std::abs((candidate - predicted).dot(lateral));
      const double dz = std::abs(candidate.z() - previous.z());
      const double clearance = estimateClearance(
        candidate, wall_clearance_min_ + std::max(0.8, graph_center_search_radius_m_));
      const double score =
        2.6 * align +
        0.08 * clearance -
        0.35 * side_shift -
        0.30 * dz;

      if (score > best_score) {
        best_score = score;
        best_center = candidate;
      }
    }

    if (!std::isfinite(best_score)) {
      break;
    }

    if (centerline_points_out.empty() ||
        distance3(centerline_points_out.back(), best_center) >= 0.4 * centerline_step_m_)
    {
      centerline_points_out.push_back(best_center);
      previous = best_center;
    }

    int added_secondary = 0;
    for (const auto &candidate : centers) {
      if (distance3(candidate, best_center) <= 1.2 * graph_branch_merge_dist_m_) {
        continue;
      }
      if (!isSegmentSafe(start.position, candidate)) {
        continue;
      }

      bool duplicate = false;
      for (const auto &existing : secondary_goals_out) {
        if (distance3(existing, candidate) <= graph_branch_merge_dist_m_) {
          duplicate = true;
          break;
        }
      }
      if (duplicate) {
        continue;
      }

      secondary_goals_out.push_back(candidate);
      added_secondary++;
      if (added_secondary >= centerline_max_secondary_goals_) {
        break;
      }
    }
    if (added_secondary > 0) {
      split_slices++;
    }
  }

  if (centerline_points_out.empty()) {
    return false;
  }

  // Keep branch hints only when a split appears consistently.
  if (split_slices < 2) {
    secondary_goals_out.clear();
  }

  primary_goal_out = centerline_points_out.back();
  return true;
}

void PathPlannerNode::seedCenterlinePreviewNodes(
  int start_node,
  const std::vector<Eigen::Vector3d> &centerline_points)
{
  // Step 1: Validate inputs.
  if (start_node < 0 || start_node >= static_cast<int>(graph_nodes_.size())) {
    return;
  }
  if (centerline_points.empty()) {
    return;
  }

  // Step 2: Insert a sparse forward backbone from centerline preview points.
  int previous_node = start_node;
  Eigen::Vector3d previous_position = graph_nodes_[static_cast<size_t>(start_node)].position;
  const double min_seed_step = std::max(1.0, 0.75 * graph_explore_spacing_m_);
  double accum_dist = 0.0;

  for (size_t i = 0; i < centerline_points.size(); ++i) {
    const Eigen::Vector3d &point = centerline_points[i];
    const double step = distance3(previous_position, point);
    if (step < 0.1) {
      continue;
    }
    if (!isSegmentSafe(previous_position, point)) {
      break;
    }

    accum_dist += step;
    const bool last_point = (i + 1 == centerline_points.size());
    if (accum_dist + 1e-6 < min_seed_step && !last_point) {
      continue;
    }

    const int node_id = addOrMergeGraphNode(point, false, false);
    if (node_id < 0) {
      continue;
    }

    if (node_id != previous_node) {
      connectGraphNodes(previous_node, node_id);
      previous_node = node_id;
    }
    previous_position = graph_nodes_[static_cast<size_t>(previous_node)].position;
    accum_dist = 0.0;
  }

  // Step 3: Refresh local branch model for seeded nodes.
  refreshBranchesAroundNode(start_node);
  if (previous_node >= 0 && previous_node < static_cast<int>(graph_nodes_.size())) {
    refreshBranchesAroundNode(previous_node);
  }
}

bool PathPlannerNode::chooseEntryBranchFromView(int start_node, int &branch_index_out)
{
  branch_index_out = -1;
  if (start_node < 0 || start_node >= static_cast<int>(graph_nodes_.size())) {
    return false;
  }

  auto &node = graph_nodes_[static_cast<size_t>(start_node)];
  if (node.branches.empty()) {
    return false;
  }

  Eigen::Vector3d forward_dir = Eigen::Vector3d::Zero();
  if (!computeDesiredDirection(start_node, forward_dir)) {
    return false;
  }

  const Eigen::Vector3d origin = node.position;
  const Eigen::Vector3d lateral(-forward_dir.y(), forward_dir.x(), 0.0);
  const double ring_dist = std::max(2.5, graph_branch_goal_dist_m_);
  const double ring_radius = std::max(1.0, graph_center_search_radius_m_);
  const double offset_step = std::max(0.3, octree_ ? octree_->getResolution() : 0.3);

  // Step 1: Determine free windows on the forward ring and keep the best one.
  struct FreeWindow
  {
    double offset_begin{0.0};
    double offset_end{0.0};
    double weighted_offset_sum{0.0};
    double weight_sum{0.0};
  };

  std::vector<FreeWindow> windows;
  bool window_open = false;
  FreeWindow active_window;

  auto close_window = [&]() {
    if (window_open) {
      windows.push_back(active_window);
      window_open = false;
      active_window = FreeWindow{};
    }
  };

  for (double off = -ring_radius; off <= ring_radius + 1e-6; off += offset_step) {
    Eigen::Vector3d probe = origin + ring_dist * forward_dir + off * lateral;
    probe.z() = origin.z();
    if (!projectGoalToFree(probe)) {
      close_window();
      continue;
    }
    if (std::abs(probe.z() - origin.z()) > 0.8) {
      close_window();
      continue;
    }
    if (!isSegmentSafe(origin, probe)) {
      close_window();
      continue;
    }

    const double clearance = estimateClearance(
      probe, std::max(wall_clearance_min_ + 1.0, graph_center_search_radius_m_));
    const double weight = std::max(0.1, clearance);

    if (!window_open) {
      window_open = true;
      active_window.offset_begin = off;
      active_window.offset_end = off;
    } else {
      active_window.offset_end = off;
    }
    active_window.weight_sum += weight;
    active_window.weighted_offset_sum += weight * off;
  }
  close_window();

  if (windows.empty()) {
    return false;
  }

  double center_offset = 0.0;
  double best_window_score = -std::numeric_limits<double>::infinity();
  for (const auto &window : windows) {
    const double span = std::max(0.0, window.offset_end - window.offset_begin);
    const double weighted_center = (window.weight_sum > 1e-6)
      ? (window.weighted_offset_sum / window.weight_sum)
      : (0.5 * (window.offset_begin + window.offset_end));
    const double score = span - 0.25 * std::abs(weighted_center);
    if (score > best_window_score) {
      best_window_score = score;
      center_offset = weighted_center;
    }
  }

  Eigen::Vector3d ring_center = origin + ring_dist * forward_dir + center_offset * lateral;
  ring_center.z() = origin.z();
  if (!isPointSafe(ring_center)) {
    Eigen::Vector3d adjusted_center = ring_center;
    if (projectGoalToFree(adjusted_center) &&
        std::abs(adjusted_center.z() - origin.z()) <= 0.5 &&
        isSegmentSafe(origin, adjusted_center))
    {
      ring_center = adjusted_center;
    }
  }

  // Step 2: Select OPEN branch closest to ring midpoint while still forward-oriented.
  double best_score = -std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < node.branches.size(); ++i) {
    const auto &branch = node.branches[i];
    if (branch.status != BranchStatus::OPEN) {
      continue;
    }

    Eigen::Vector3d dir = branch.goal - origin;
    dir.z() = 0.0;
    if (dir.norm() < 1e-3) {
      continue;
    }
    dir.normalize();

    const double align = forward_dir.dot(dir);
    const double dz = std::abs(branch.goal.z() - origin.z());
    const double dist_to_center = std::hypot(
      branch.goal.x() - ring_center.x(),
      branch.goal.y() - ring_center.y());
    const double score =
      graph_view_align_weight_ * align -
      0.35 * dist_to_center -
      graph_vertical_penalty_weight_ * dz;

    if (score > best_score) {
      best_score = score;
      branch_index_out = static_cast<int>(i);
    }
  }

  if (branch_index_out < 0) {
    return false;
  }

  // Require at least weak forward alignment for entry decision.
  return best_score > -0.1;
}

bool PathPlannerNode::computeDesiredDirection(int start_node, Eigen::Vector3d &dir_out) const
{
  dir_out = Eigen::Vector3d::Zero();
  Eigen::Vector3d continuity = Eigen::Vector3d::Zero();
  bool has_continuity = false;

  // Step 1: Prefer current route/goal direction to preserve path continuity.
  if (has_current_goal_) {
    Eigen::Vector3d toward_goal = current_goal_ - current_position_;
    toward_goal.z() = 0.0;
    if (toward_goal.norm() > 0.3) {
      continuity = toward_goal.normalized();
      has_continuity = true;
    }
  }

  if (!has_continuity && planned_forward_hint_.norm() > 1e-3) {
    continuity = planned_forward_hint_.normalized();
    has_continuity = true;
  }

  // Step 2: Fallback to graph progression when no active continuity exists.
  if (!has_continuity &&
      start_node >= 0 &&
      start_node < static_cast<int>(graph_nodes_.size()) &&
      previous_graph_node_id_ >= 0 &&
      previous_graph_node_id_ < static_cast<int>(graph_nodes_.size()) &&
      previous_graph_node_id_ != start_node)
  {
    Eigen::Vector3d from_previous = graph_nodes_[static_cast<size_t>(start_node)].position -
      graph_nodes_[static_cast<size_t>(previous_graph_node_id_)].position;
    from_previous.z() = 0.0;
    if (from_previous.norm() > 1e-3) {
      continuity = from_previous.normalized();
      has_continuity = true;
    }
  }

  // Step 3: Use heading hints only as a weak reference (not as primary direction).
  Eigen::Vector3d reference = Eigen::Vector3d::Zero();
  if (has_continuity) {
    reference = continuity;
  } else if (!heading_hint_.isZero(1e-6)) {
    reference = heading_hint_;
  } else if (!body_forward_hint_.isZero(1e-6)) {
    reference = body_forward_hint_;
  }
  reference.z() = 0.0;
  if (reference.norm() > 1e-3) {
    reference.normalize();
  } else {
    reference = Eigen::Vector3d::Zero();
  }

  // Step 4: Estimate tunnel escape direction from free depth in XY plane.
  Eigen::Vector3d escape = Eigen::Vector3d::Zero();
  bool has_escape = false;
  const double max_probe_dist = std::max(25.0, 2.0 * graph_branch_goal_dist_m_);
  constexpr int kYawSamples = 24;
  constexpr double kPi = 3.14159265358979323846;
  double best_score = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < kYawSamples; ++i) {
    const double yaw = (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(kYawSamples);
    Eigen::Vector3d cand(std::cos(yaw), std::sin(yaw), 0.0);
    if (!reference.isZero(1e-6) && cand.dot(reference) < -0.35) {
      continue;
    }

    const double free_depth = estimateForwardObstacleDistance(current_position_, cand, max_probe_dist);
    const double align = reference.isZero(1e-6) ? 0.0 : cand.dot(reference);
    const double score =
      free_depth +
      4.0 * std::max(0.0, align) -
      1.2 * std::max(0.0, -align);

    if (score > best_score) {
      best_score = score;
      escape = cand;
      has_escape = true;
    }
  }

  // Step 5: Blend continuity with tunnel depth direction.
  if (has_continuity && has_escape) {
    const double continuity_depth =
      estimateForwardObstacleDistance(current_position_, continuity, max_probe_dist);
    const double wall_pressure = std::clamp((12.0 - continuity_depth) / 12.0, 0.0, 1.0);
    const double blend = 0.15 + 0.55 * wall_pressure;
    dir_out = (1.0 - blend) * continuity + blend * escape;
  } else if (has_continuity) {
    dir_out = continuity;
  } else if (has_escape) {
    dir_out = escape;
  } else if (!reference.isZero(1e-6)) {
    dir_out = reference;
  } else {
    return false;
  }

  dir_out.z() = 0.0;
  if (dir_out.norm() < 1e-3) {
    return false;
  }
  dir_out.normalize();
  return true;
}

double PathPlannerNode::estimateForwardObstacleDistance(
  const Eigen::Vector3d &origin,
  const Eigen::Vector3d &dir_xy,
  double max_dist) const
{
  if (!octree_) {
    return max_dist;
  }

  Eigen::Vector3d dir = dir_xy;
  dir.z() = 0.0;
  if (dir.norm() < 1e-3) {
    return max_dist;
  }
  dir.normalize();

  const double res = octree_->getResolution();
  const double step = std::max(0.2, res);
  const std::array<double, 3> z_offsets = {0.0, 0.4, -0.4};

  for (double d = step; d <= max_dist + 1e-6; d += step) {
    for (double dz : z_offsets) {
      const Eigen::Vector3d q = origin + d * dir + Eigen::Vector3d(0.0, 0.0, dz);
      const octomap::OcTreeNode *node = octree_->search(q.x(), q.y(), q.z());
      if (node && octree_->isNodeOccupied(node)) {
        return d;
      }
    }
  }

  return max_dist;
}

void PathPlannerNode::updateNodeClassification(int node_id)
{
  // Step 1: Validate node id.
  if (node_id < 0 || node_id >= static_cast<int>(graph_nodes_.size())) {
    return;
  }

  auto &node = graph_nodes_[node_id];

  // Step 2: Count active branches.
  int open_count = 0;
  for (const auto &branch : node.branches) {
    if (branch.status == BranchStatus::OPEN || branch.status == BranchStatus::ASSIGNED) {
      open_count++;
    }
  }

  // Step 3: Derive class labels from topology.
  node.is_junction = (open_count >= 2) || (node.neighbors.size() >= 3);
  node.is_dead_end = node.visited && (open_count == 0) && (node.neighbors.size() <= 1);
}

bool PathPlannerNode::computeShortestNodePath(
  int start_node,
  int goal_node,
  std::vector<int> &path_nodes_out) const
{
  path_nodes_out.clear();

  // Step 1: Validate indices.
  if (start_node < 0 || goal_node < 0) {
    return false;
  }
  if (start_node >= static_cast<int>(graph_nodes_.size()) || goal_node >= static_cast<int>(graph_nodes_.size())) {
    return false;
  }

  // Step 2: Handle trivial path.
  if (start_node == goal_node) {
    path_nodes_out.push_back(start_node);
    return true;
  }

  // Step 3: Dijkstra shortest path.
  const size_t n = graph_nodes_.size();
  std::vector<double> dist(n, std::numeric_limits<double>::infinity());
  std::vector<int> prev(n, -1);

  using QueueEntry = std::pair<double, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;

  dist[static_cast<size_t>(start_node)] = 0.0;
  queue.emplace(0.0, start_node);

  while (!queue.empty()) {
    const auto [cost, node_id] = queue.top();
    queue.pop();

    if (cost > dist[static_cast<size_t>(node_id)] + 1e-9) {
      continue;
    }
    if (node_id == goal_node) {
      break;
    }

    for (int neighbor : graph_nodes_[static_cast<size_t>(node_id)].neighbors) {
      if (neighbor < 0 || neighbor >= static_cast<int>(n)) {
        continue;
      }

      const double edge_cost =
        distance3(graph_nodes_[static_cast<size_t>(node_id)].position, graph_nodes_[static_cast<size_t>(neighbor)].position);
      const double next_cost = cost + edge_cost;
      if (next_cost < dist[static_cast<size_t>(neighbor)]) {
        dist[static_cast<size_t>(neighbor)] = next_cost;
        prev[static_cast<size_t>(neighbor)] = node_id;
        queue.emplace(next_cost, neighbor);
      }
    }
  }

  if (!std::isfinite(dist[static_cast<size_t>(goal_node)])) {
    return false;
  }

  // Step 4: Reconstruct route from predecessor table.
  std::vector<int> reverse_path;
  for (int at = goal_node; at >= 0; at = prev[static_cast<size_t>(at)]) {
    reverse_path.push_back(at);
    if (at == start_node) {
      break;
    }
  }

  if (reverse_path.empty() || reverse_path.back() != start_node) {
    return false;
  }

  path_nodes_out.assign(reverse_path.rbegin(), reverse_path.rend());
  return true;
}

double PathPlannerNode::computeNodePathLength(const std::vector<int> &path_nodes) const
{
  if (path_nodes.size() < 2) {
    return 0.0;
  }

  double total = 0.0;
  for (size_t i = 1; i < path_nodes.size(); ++i) {
    const int a = path_nodes[i - 1];
    const int b = path_nodes[i];
    if (a < 0 || b < 0) {
      continue;
    }
    if (a >= static_cast<int>(graph_nodes_.size()) || b >= static_cast<int>(graph_nodes_.size())) {
      continue;
    }
    total += distance3(graph_nodes_[static_cast<size_t>(a)].position, graph_nodes_[static_cast<size_t>(b)].position);
  }

  return total;
}

bool PathPlannerNode::hasOpenFrontiers() const
{
  for (const auto &node : graph_nodes_) {
    for (const auto &branch : node.branches) {
      if (branch.status == BranchStatus::OPEN || branch.status == BranchStatus::ASSIGNED) {
        return true;
      }
    }
  }
  return false;
}

void PathPlannerNode::setActiveBranchStatus(BranchStatus status, bool clear_active_state)
{
  // Step 1: Resolve active node.
  if (active_branch_node_id_ < 0 || active_branch_node_id_ >= static_cast<int>(graph_nodes_.size())) {
    if (clear_active_state) {
      active_branch_node_id_ = -1;
      active_branch_index_ = -1;
      has_active_branch_goal_ = false;
    }
    return;
  }

  auto &node = graph_nodes_[static_cast<size_t>(active_branch_node_id_)];

  // Step 2: Resolve active branch by index or by goal proximity.
  int branch_index = active_branch_index_;
  if (branch_index < 0 || branch_index >= static_cast<int>(node.branches.size())) {
    branch_index = -1;
    if (has_active_branch_goal_) {
      for (size_t i = 0; i < node.branches.size(); ++i) {
        if (distance3(node.branches[i].goal, active_branch_goal_) <= graph_branch_merge_dist_m_) {
          branch_index = static_cast<int>(i);
          break;
        }
      }
    }
  }

  // Step 3: Apply requested status when branch exists.
  if (branch_index >= 0 && branch_index < static_cast<int>(node.branches.size())) {
    node.branches[static_cast<size_t>(branch_index)].status = status;
  }
  updateNodeClassification(active_branch_node_id_);

  // Step 4: Optionally clear active state bookkeeping.
  if (clear_active_state) {
    active_branch_node_id_ = -1;
    active_branch_index_ = -1;
    has_active_branch_goal_ = false;
  }
}

void PathPlannerNode::markActiveBranchVisited()
{
  // Step 1: Validate active branch owner.
  if (active_branch_node_id_ < 0 || active_branch_node_id_ >= static_cast<int>(graph_nodes_.size())) {
    active_branch_node_id_ = -1;
    active_branch_index_ = -1;
    has_active_branch_goal_ = false;
    return;
  }

  auto &node = graph_nodes_[active_branch_node_id_];

  // Step 2: Resolve branch index using cached index or cached goal.
  int branch_index = active_branch_index_;
  if (branch_index < 0 || branch_index >= static_cast<int>(node.branches.size())) {
    branch_index = -1;
    if (has_active_branch_goal_) {
      for (size_t i = 0; i < node.branches.size(); ++i) {
        if (distance3(node.branches[i].goal, active_branch_goal_) <= graph_branch_merge_dist_m_) {
          branch_index = static_cast<int>(i);
          break;
        }
      }
    }
  }

  // Step 3: Mark branch visited.
  if (branch_index >= 0 && branch_index < static_cast<int>(node.branches.size())) {
    node.branches[static_cast<size_t>(branch_index)].status = BranchStatus::VISITED;
  }

  updateNodeClassification(active_branch_node_id_);

  // Step 4: Reset active branch state.
  active_branch_node_id_ = -1;
  active_branch_index_ = -1;
  has_active_branch_goal_ = false;
}

bool PathPlannerNode::smoothPathCorners(std::vector<Eigen::Vector3d> &path_in_out) const
{
  if (path_in_out.size() < 3 || ompl_corner_smoothing_passes_ <= 0) {
    return true;
  }

  constexpr double kDegToRad = 0.01745329251994329576923690768489;
  const double min_cos = std::cos(ompl_max_turn_deg_ * kDegToRad);

  std::vector<Eigen::Vector3d> path = path_in_out;
  for (int pass = 0; pass < ompl_corner_smoothing_passes_; ++pass) {
    bool changed_any = false;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
      const Eigen::Vector3d prev = path[i - 1];
      const Eigen::Vector3d curr = path[i];
      const Eigen::Vector3d next = path[i + 1];

      Eigen::Vector3d in = curr - prev;
      Eigen::Vector3d out = next - curr;
      const double in_len = in.norm();
      const double out_len = out.norm();
      if (in_len < 1e-3 || out_len < 1e-3) {
        continue;
      }

      in /= in_len;
      out /= out_len;
      const double cos_turn = in.dot(out);
      if (cos_turn >= min_cos) {
        continue;
      }

      Eigen::Vector3d target = 0.5 * (prev + next);
      Eigen::Vector3d candidate = (1.0 - ompl_corner_blend_) * curr + ompl_corner_blend_ * target;
      if (!projectGoalToFree(candidate)) {
        continue;
      }
      if (!isPointSafe(candidate)) {
        continue;
      }
      if (!isSegmentSafe(prev, candidate) || !isSegmentSafe(candidate, next)) {
        continue;
      }

      path[i] = candidate;
      changed_any = true;
    }

    if (!changed_any) {
      break;
    }
  }

  if (!isPathSafe(path)) {
    return false;
  }

  path_in_out = std::move(path);
  return true;
}

bool PathPlannerNode::centerPathInTunnel(std::vector<Eigen::Vector3d> &path_in_out) const
{
  if (path_in_out.size() < 3) {
    return true;
  }

  std::vector<Eigen::Vector3d> path = path_in_out;
  const double search_radius = std::max(1.2, 1.5 * graph_center_search_radius_m_);
  for (int pass = 0; pass < 3; ++pass) {
    bool changed_any = false;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
      Eigen::Vector3d candidate = path[i];
      if (!recenterToLocalMidpoint(candidate, search_radius)) {
        continue;
      }
      if (distance3(candidate, path[i]) < 0.05) {
        continue;
      }
      if (!isSegmentSafe(path[i - 1], candidate) || !isSegmentSafe(candidate, path[i + 1])) {
        continue;
      }
      path[i] = candidate;
      changed_any = true;
    }
    if (!changed_any) {
      break;
    }
  }

  if (!isPathSafe(path)) {
    return false;
  }

  path_in_out = std::move(path);
  return true;
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

  // Keep replans anchored to current pose: reject large start projection jumps.
  const double start_shift = distance3(start_safe, start);
  const double max_start_shift_m = std::max(0.8, 4.0 * octree_->getResolution());
  if (start_shift > max_start_shift_m) {
    return false;
  }

  // Keep goal centered, but never pull start away from current pose.
  (void)recenterToLocalMidpoint(goal_safe, std::max(0.8, graph_center_search_radius_m_));

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

  // Step 3: Configure OMPL setup and state validity checker.
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
  if (simplified_points.size() >= 3) {
    (void)smoothPathCorners(simplified_points);
    (void)centerPathInTunnel(simplified_points);
  }
  if (simplified_points.size() >= 2 && isPathSafe(simplified_points)) {
    path_out = std::move(simplified_points);
  } else {
    if (raw_points.size() >= 3) {
      std::vector<Eigen::Vector3d> raw_smoothed = raw_points;
      if (smoothPathCorners(raw_smoothed)) {
        (void)centerPathInTunnel(raw_smoothed);
      }
      if (isPathSafe(raw_smoothed)) {
        raw_points = std::move(raw_smoothed);
      }
    }
    path_out = std::move(raw_points);
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Simplified path violated safety check, fallback to raw OMPL path.");
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

  // Step 1: Build points with adaptive spacing.
  const double span = std::max(0.1, waypoint_dist_max_ - waypoint_dist_min_);
  Eigen::Vector3d prev_dir = Eigen::Vector3d::Zero();
  bool has_prev_dir = false;

  waypoints_out.push_back(path_in.front());
  auto push_waypoint_centered = [&](const Eigen::Vector3d &sample) {
    if (waypoints_out.empty()) {
      waypoints_out.push_back(sample);
      return;
    }

    Eigen::Vector3d centered = sample;
    const double recenter_radius = std::max(0.6, 0.5 * graph_center_search_radius_m_);
    const bool centered_ok =
      recenterToLocalMidpoint(centered, recenter_radius) &&
      isSegmentSafe(waypoints_out.back(), centered);

    const Eigen::Vector3d chosen = centered_ok ? centered : sample;
    if (!isSegmentSafe(waypoints_out.back(), chosen)) {
      return;
    }

    if (distance3(chosen, waypoints_out.back()) >= waypoint_dist_min_ * 0.95) {
      waypoints_out.push_back(chosen);
    }
  };

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
      push_waypoint_centered(sample);
      dist += step;
    }
    prev_dir = seg_dir;
    has_prev_dir = true;
  }

  // Step 2: Ensure final goal is included.
  if (distance3(waypoints_out.back(), path_in.back()) > 1e-3) {
    waypoints_out.push_back(path_in.back());
  }

  // Step 3: Enforce maximum segment length.
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

  // Step 1a: Stabilize replan start velocity by keeping only aligned forward component.
  Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
  Eigen::Vector3d first_dir = Eigen::Vector3d::Zero();
  for (size_t i = 1; i < waypoints.size(); ++i) {
    const Eigen::Vector3d diff = waypoints[i] - current_position_;
    if (diff.norm() > 0.2) {
      first_dir = diff.normalized();
      break;
    }
  }

  const double current_speed = current_velocity_.norm();
  if (current_speed > 0.05 && first_dir.norm() > 1e-3) {
    const Eigen::Vector3d vel_dir = current_velocity_ / current_speed;
    const double align = vel_dir.dot(first_dir);
    if (align >= traj_start_align_cos_min_) {
      const double projected_speed = std::max(0.0, align) * current_speed;
      const double max_start_speed = traj_start_speed_max_ratio_ * max_v_;
      const double start_speed = std::clamp(projected_speed, 0.0, max_start_speed);
      v0 = start_speed * first_dir;
    }
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
  for (double &t : segment_times) {
    t *= traj_time_scale_;
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

  const double influence_radius = std::max(graph_explore_spacing_m_, 2.0 * waypoint_dist_max_);
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

double PathPlannerNode::distanceToPath(
  const Eigen::Vector3d &p,
  const std::vector<Eigen::Vector3d> &path_points)
{
  if (path_points.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (path_points.size() == 1) {
    return distance3(p, path_points.front());
  }

  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < path_points.size(); ++i) {
    const Eigen::Vector3d a = path_points[i - 1];
    const Eigen::Vector3d b = path_points[i];
    const Eigen::Vector3d ab = b - a;
    const double ab2 = ab.squaredNorm();
    if (ab2 < 1e-9) {
      best = std::min(best, distance3(p, a));
      continue;
    }

    const double t = std::clamp((p - a).dot(ab) / ab2, 0.0, 1.0);
    const Eigen::Vector3d proj = a + t * ab;
    best = std::min(best, distance3(p, proj));
  }

  return best;
}

bool PathPlannerNode::pathProgressMetrics(
  const Eigen::Vector3d &p,
  const std::vector<Eigen::Vector3d> &path_points,
  double &distance_out,
  double &progress_ratio_out,
  double &remaining_dist_out)
{
  distance_out = std::numeric_limits<double>::infinity();
  progress_ratio_out = 0.0;
  remaining_dist_out = std::numeric_limits<double>::infinity();

  if (path_points.size() < 2) {
    return false;
  }

  std::vector<double> cumulative(path_points.size(), 0.0);
  for (size_t i = 1; i < path_points.size(); ++i) {
    cumulative[i] = cumulative[i - 1] + distance3(path_points[i - 1], path_points[i]);
  }
  const double total_len = cumulative.back();
  if (total_len < 1e-6) {
    distance_out = distance3(p, path_points.front());
    progress_ratio_out = 1.0;
    remaining_dist_out = 0.0;
    return true;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  double best_progress_len = 0.0;
  for (size_t i = 1; i < path_points.size(); ++i) {
    const Eigen::Vector3d a = path_points[i - 1];
    const Eigen::Vector3d b = path_points[i];
    const Eigen::Vector3d ab = b - a;
    const double ab2 = ab.squaredNorm();
    if (ab2 < 1e-9) {
      const double d = distance3(p, a);
      if (d < best_dist) {
        best_dist = d;
        best_progress_len = cumulative[i - 1];
      }
      continue;
    }

    const double t = std::clamp((p - a).dot(ab) / ab2, 0.0, 1.0);
    const Eigen::Vector3d proj = a + t * ab;
    const double d = distance3(p, proj);
    if (d < best_dist) {
      best_dist = d;
      best_progress_len = cumulative[i - 1] + t * std::sqrt(ab2);
    }
  }

  distance_out = best_dist;
  progress_ratio_out = std::clamp(best_progress_len / total_len, 0.0, 1.0);
  remaining_dist_out = std::max(0.0, total_len - best_progress_len);
  return true;
}

bool PathPlannerNode::isFollowingActivePath() const
{
  if (active_path_points_.size() < 2) {
    return false;
  }

  double dist_to_path = std::numeric_limits<double>::infinity();
  double progress_ratio = 0.0;
  double remaining_dist = std::numeric_limits<double>::infinity();
  if (!pathProgressMetrics(
      current_position_, active_path_points_, dist_to_path, progress_ratio, remaining_dist))
  {
    return false;
  }
  if (dist_to_path > active_path_follow_dist_m_) {
    return false;
  }

  // Reject stalled/near-goal situations from being treated as healthy path progress.
  if (has_current_goal_ && distance3(current_position_, current_goal_) <= 1.5 * goal_reached_radius_m_) {
    return false;
  }

  return true;
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

  // Step 1: Publish only currently selected path.
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

  auto graph_nodes_marker = make_marker(
    5, "graph_nodes", visualization_msgs::msg::Marker::SPHERE_LIST,
    0.22F, 0.22F, 0.22F, 0.8F, 0.4F, 0.2F, 0.9F);
  for (const auto &node : graph_nodes_) {
    graph_nodes_marker.points.push_back(toPointMsg(node.position));
  }
  marker_array.markers.push_back(graph_nodes_marker);

  auto graph_edges = make_marker(
    6, "graph_edges", visualization_msgs::msg::Marker::LINE_LIST,
    0.06F, 0.0F, 0.0F, 0.9F, 0.8F, 0.2F, 0.8F);
  std::set<std::pair<int, int>> edge_set;
  for (const auto &node : graph_nodes_) {
    for (int neighbor : node.neighbors) {
      if (neighbor < 0 || neighbor >= static_cast<int>(graph_nodes_.size())) {
        continue;
      }
      const int a = std::min(node.id, neighbor);
      const int b = std::max(node.id, neighbor);
      if (!edge_set.insert({a, b}).second) {
        continue;
      }
      graph_edges.points.push_back(toPointMsg(graph_nodes_[static_cast<size_t>(a)].position));
      graph_edges.points.push_back(toPointMsg(graph_nodes_[static_cast<size_t>(b)].position));
    }
  }
  marker_array.markers.push_back(graph_edges);

  auto frontier_points = make_marker(
    7, "frontier_goals", visualization_msgs::msg::Marker::SPHERE_LIST,
    0.26F, 0.26F, 0.26F, 0.05F, 0.95F, 0.95F, 0.9F);
  for (const auto &node : graph_nodes_) {
    for (const auto &branch : node.branches) {
      if (branch.status == BranchStatus::OPEN || branch.status == BranchStatus::ASSIGNED) {
        frontier_points.points.push_back(toPointMsg(branch.goal));
      }
    }
  }
  marker_array.markers.push_back(frontier_points);

  if (ompl_path.size() >= 2) {
    const size_t mid_index = std::min(ompl_path.size() - 1, std::max<size_t>(1, ompl_path.size() / 2));
    auto midpoint_marker = make_marker(
      8, "ompl_midpoint", visualization_msgs::msg::Marker::SPHERE,
      0.42F, 0.42F, 0.42F, 0.2F, 1.0F, 0.2F, 0.95F);
    midpoint_marker.pose.position = toPointMsg(ompl_path[mid_index]);
    midpoint_marker.pose.orientation.w = 1.0;
    marker_array.markers.push_back(midpoint_marker);
  }

  marker_pub_->publish(marker_array);
}

bool PathPlannerNode::shouldReportDone() const
{
  if (mode_ != PlannerMode::EXPLORE) {
    return false;
  }

  if (hasOpenFrontiers()) {
    return false;
  }

  if (frontier_fail_cycles_ < max_frontier_fail_cycles_) {
    return false;
  }

  return !graph_nodes_.empty();
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
