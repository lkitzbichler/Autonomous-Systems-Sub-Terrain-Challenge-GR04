#include "pathplanner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

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
        frontier_fail_cycles_ = 0;
        debug_candidate_points_.clear();
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
        frontier_fail_cycles_ = 0;
        debug_candidate_points_.clear();
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

  // Step 1: Prefer true frontiers first.
  if (chooseExploreGoal(goal_out, reason_out)) {
    return true;
  }

  // Step 2: Fallback to free-space sweep if frontier extraction fails.
  return chooseFallbackGoal(goal_out, reason_out);
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
  marker_array.markers.reserve(6);

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
