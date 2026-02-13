#include "path_planning_pkg/path_planner_node.hpp"

#include <octomap_msgs/conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

PathPlannerNode::PathPlannerNode() : rclcpp::Node("path_planner") {
  command_topic_ =
      declare_parameter<std::string>("command_topic", "statemachine/command");
  octomap_topic_ =
      declare_parameter<std::string>("octomap_topic", "octomap_binary");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/current_state_est");
  trajectory_topic_ =
      declare_parameter<std::string>("trajectory_topic", "trajectory");
  markers_topic_ =
      declare_parameter<std::string>("markers_topic", "path_planner/markers");
  world_frame_ = declare_parameter<std::string>("world_frame", "world");
  waypoint_done_topic_ =
      declare_parameter<std::string>("waypoint_done_topic", "basic_waypoint/done");
  actual_path_topic_ =
      declare_parameter<std::string>("actual_path_topic", "path_planner/actual_path");
  planned_path_topic_ =
      declare_parameter<std::string>("planned_path_topic", "path_planner/planned_path");

  auto_start_ = declare_parameter<bool>("auto_start", true);
  visualize_ = declare_parameter<bool>("visualize", true);
  replan_while_moving_ = declare_parameter<bool>("replan_while_moving", false);
  start_on_waypoint_done_ =
      declare_parameter<bool>("start_on_waypoint_done", false);
  publish_actual_path_ =
      declare_parameter<bool>("publish_actual_path", true);
  publish_planned_path_ =
      declare_parameter<bool>("publish_planned_path", true);

  tick_rate_hz_ = declare_parameter<double>("tick_rate_hz", 1.0);
  max_v_ = declare_parameter<double>("max_v", 1.5);
  max_a_ = declare_parameter<double>("max_a", 1.0);
  goal_reached_tolerance_ =
      declare_parameter<double>("goal_reached_tolerance", 0.5);
  replan_interval_s_ = declare_parameter<double>("replan_interval_s", 2.0);
  replan_min_remaining_ =
      declare_parameter<double>("replan_min_remaining", 0.0);
  frontier_max_distance_ =
      declare_parameter<double>("frontier_max_distance", 30.0);
  min_goal_distance_ = declare_parameter<double>("min_goal_distance", 1.0);
  frontier_score_distance_weight_ =
      declare_parameter<double>("frontier_score_distance_weight", 1.0);
  frontier_score_size_weight_ =
      declare_parameter<double>("frontier_score_size_weight", 1.0);
  forward_bias_weight_ =
      declare_parameter<double>("forward_bias_weight", 0.0);
  min_forward_cos_ = declare_parameter<double>("min_forward_cos", -1.0);
  lateral_bias_weight_ = declare_parameter<double>("lateral_bias_weight", 0.0);
  forward_speed_threshold_ =
      declare_parameter<double>("forward_speed_threshold", 0.2);
  use_velocity_heading_ =
      declare_parameter<bool>("use_velocity_heading", false);
  invert_forward_dir_ =
      declare_parameter<bool>("invert_forward_dir", false);
  use_yaw_ = declare_parameter<bool>("use_yaw", true);
  yaw_speed_threshold_ =
      declare_parameter<double>("yaw_speed_threshold", 0.2);
  yaw_offset_ = declare_parameter<double>("yaw_offset", 0.0);
  backtrack_enabled_ = declare_parameter<bool>("backtrack_enabled", true);
  backtrack_min_distance_ =
      declare_parameter<double>("backtrack_min_distance", 3.0);
  backtrack_stack_size_ =
      declare_parameter<int>("backtrack_stack_size", 50);
  ompl_planning_time_ =
      declare_parameter<double>("ompl_planning_time", 1.0);
  ompl_range_ = declare_parameter<double>("ompl_range", 5.0);
  ompl_goal_bias_ = declare_parameter<double>("ompl_goal_bias", 0.05);
  ompl_resolution_ = declare_parameter<double>("ompl_resolution", 0.02);
  ompl_simplify_ = declare_parameter<bool>("ompl_simplify", true);
  ompl_allow_unknown_ = declare_parameter<bool>("ompl_allow_unknown", true);
  clearance_radius_ = declare_parameter<double>("clearance_radius", 0.0);
  path_simplify_distance_ =
      declare_parameter<double>("path_simplify_distance", 1.0);
  use_line_of_sight_prune_ =
      declare_parameter<bool>("use_line_of_sight_prune", true);
  line_of_sight_step_ =
      declare_parameter<double>("line_of_sight_step", 0.5);
  actual_path_min_distance_ =
      declare_parameter<double>("actual_path_min_distance", 0.0);
  planned_path_min_distance_ =
      declare_parameter<double>("planned_path_min_distance", 0.0);

  frontier_min_cluster_size_ =
      declare_parameter<int>("frontier_min_cluster_size", 5);
  max_frontier_nodes_ = declare_parameter<int>("max_frontier_nodes", 50000);

  command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic_, 10,
      std::bind(&PathPlannerNode::onCommand, this, std::placeholders::_1));

  octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, 10,
      std::bind(&PathPlannerNode::onOctomap, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&PathPlannerNode::onOdom, this, std::placeholders::_1));

  if (start_on_waypoint_done_) {
    waypoint_done_sub_ = create_subscription<std_msgs::msg::Bool>(
        waypoint_done_topic_, rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&PathPlannerNode::onWaypointDone, this, std::placeholders::_1));
  }

  trajectory_pub_ =
      create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(
          trajectory_topic_, 10);
  markers_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, 1);
  if (publish_actual_path_) {
    actual_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        actual_path_topic_,
        rclcpp::QoS(1).transient_local().reliable());
    actual_path_.header.frame_id = world_frame_;
  }
  if (publish_planned_path_) {
    planned_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        planned_path_topic_,
        rclcpp::QoS(1).transient_local().reliable());
    planned_path_history_.header.frame_id = world_frame_;
  }

  if (tick_rate_hz_ > 0.0) {
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / tick_rate_hz_),
        std::bind(&PathPlannerNode::tick, this));
  }

  exploring_ = auto_start_;
  last_plan_time_ = get_clock()->now();

  RCLCPP_INFO(get_logger(),
              "path_planner ready (octomap='%s', odom='%s', trajectory='%s')",
              octomap_topic_.c_str(), odom_topic_.c_str(),
              trajectory_topic_.c_str());
}

void PathPlannerNode::onCommand(const std_msgs::msg::String::SharedPtr msg) {
  // Simple string command interface (start/stop/replan).
  std::string cmd = msg->data;
  std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);

  if (cmd == "start" || cmd == "explore" || cmd == "resume") {
    exploring_ = true;
    force_replan_ = true;
  } else if (cmd == "stop" || cmd == "pause") {
    exploring_ = false;
  } else if (cmd == "replan") {
    force_replan_ = true;
  }

  RCLCPP_INFO(get_logger(), "command: '%s' (exploring=%s)", cmd.c_str(),
              exploring_ ? "true" : "false");
}

void PathPlannerNode::onWaypointDone(
    const std_msgs::msg::Bool::SharedPtr msg) {
  waypoint_done_ = msg->data;
}

void PathPlannerNode::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg) {
  // Cache the latest Octomap and update bounds.
  std::unique_ptr<octomap::AbstractOcTree> tree;
  if (msg->binary) {
    tree.reset(octomap_msgs::binaryMsgToMap(*msg));
  } else {
    tree.reset(octomap_msgs::fullMsgToMap(*msg));
  }

  if (!tree) {
    RCLCPP_WARN(get_logger(), "Failed to convert Octomap message.");
    return;
  }

  auto *oc_tree = dynamic_cast<octomap::OcTree *>(tree.get());
  if (!oc_tree) {
    RCLCPP_WARN(get_logger(), "Octomap message is not an OcTree.");
    return;
  }

  std::lock_guard<std::mutex> lock(map_mutex_);
  map_.reset(static_cast<octomap::OcTree *>(tree.release()));
  have_map_ = true;

  double min_x = 0.0, min_y = 0.0, min_z = 0.0;
  double max_x = 0.0, max_y = 0.0, max_z = 0.0;
  map_->getMetricMin(min_x, min_y, min_z);
  map_->getMetricMax(max_x, max_y, max_z);
  map_min_ = octomap::point3d(min_x, min_y, min_z);
  map_max_ = octomap::point3d(max_x, max_y, max_z);
  have_bounds_ = true;
}

void PathPlannerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Update pose/velocity and derive a forward direction for scoring/yaw.
  current_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z;
  current_velocity_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
      msg->twist.twist.linear.z;
  have_odom_ = true;

  // --- Actual path ---
  if (publish_actual_path_ && actual_path_pub_) {
    bool append = true;
    if (have_actual_path_ && actual_path_min_distance_ > 0.0) {
      const double dist = (current_position_ - last_path_position_).norm();
      append = dist >= actual_path_min_distance_;
    }
    if (!have_actual_path_ || append) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = msg->header.stamp;
      pose.header.frame_id = world_frame_;
      pose.pose = msg->pose.pose;
      actual_path_.poses.push_back(pose);
      last_path_position_ = current_position_;
      have_actual_path_ = true;
    }
    actual_path_.header.stamp = msg->header.stamp;
    actual_path_pub_->publish(actual_path_);
  }

  const Eigen::Vector3d vel(current_velocity_.x(), current_velocity_.y(), 0.0);
  if (use_velocity_heading_ && vel.head<2>().norm() > forward_speed_threshold_) {
    forward_dir_ = vel.normalized();
    if (invert_forward_dir_) {
      forward_dir_ *= -1.0;
    }
    return;
  }

  const auto &q = msg->pose.pose.orientation;
  // Derive yaw from quaternion (Z-up world).
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  forward_dir_ = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
  if (invert_forward_dir_) {
    forward_dir_ *= -1.0;
  }
  last_yaw_ = yaw + yaw_offset_;
}

void PathPlannerNode::tick() {
  // --- Planning loop ---
  // Main planning loop: select frontier -> plan path -> publish trajectory.
  if (!exploring_) {
    if (start_on_waypoint_done_ && waypoint_done_) {
      exploring_ = true;
      force_replan_ = true;
      RCLCPP_INFO(get_logger(), "basic_waypoint/done -> start planning");
    } else {
      return;
    }
  }
  if (!have_map_ || !have_odom_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Waiting for map and odom...");
    return;
  }

  const rclcpp::Time now = get_clock()->now();
  if (last_plan_time_.get_clock_type() != now.get_clock_type()) {
    last_plan_time_ = now;
  }

  if (has_goal_) {
    // Cache reached goals for optional backtracking.
    const double dist = (current_position_ - goal_position_).norm();
    if (dist < goal_reached_tolerance_) {
      if (backtrack_enabled_) {
        if (backtrack_stack_.empty() ||
            (goal_position_ - backtrack_stack_.back()).norm() >
                backtrack_min_distance_) {
          // Keep a bounded stack of reached goals for fallback routing.
          backtrack_stack_.push_back(goal_position_);
          if (static_cast<int>(backtrack_stack_.size()) > backtrack_stack_size_) {
            backtrack_stack_.pop_front();
          }
        }
      }
      has_goal_ = false;
      force_replan_ = true;
    }
  }

  const bool interval_elapsed =
      (now - last_plan_time_).seconds() > replan_interval_s_;

  if (has_goal_ && !force_replan_ &&
      !(replan_while_moving_ && interval_elapsed)) {
    return;
  }

  if (!interval_elapsed && !force_replan_ && has_goal_) {
    return;
  }

  if (has_goal_ && replan_while_moving_ && interval_elapsed &&
      replan_min_remaining_ > 0.0) {
    const double dist_remaining =
        (current_position_ - goal_position_).norm();
    if (dist_remaining > replan_min_remaining_) {
      return;
    }
  }

  // Build frontier clusters in the known free space.
  std::vector<FrontierCluster> clusters;
  if (!computeFrontierClusters(&clusters) || clusters.empty()) {
    // If no frontiers remain, try backtracking to a previous goal.
    if (backtrack_enabled_) {
      Eigen::Vector3d target;
      if (popBacktrackTarget(&target)) {
        if (planToPosition(target, clusters)) {
          return;
        }
      }
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "No frontiers found.");
    return;
  }

  // Select the best frontier candidate as next goal.
  octomap::OcTreeKey goal_key;
  if (!selectFrontierGoal(clusters, &goal_key)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Failed to select frontier goal.");
    return;
  }

  std::shared_ptr<octomap::OcTree> map;
  octomap::point3d min;
  octomap::point3d max;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
    min = map_min_;
    max = map_max_;
  }
  if (!map) {
    return;
  }

  const octomap::point3d goal_coord = map->keyToCoord(goal_key);
  goal_position_ =
      Eigen::Vector3d(goal_coord.x(), goal_coord.y(), goal_coord.z());

  Eigen::Vector3d start = current_position_;
  const octomap::point3d start_coord(start.x(), start.y(), start.z());
  if (!coordInBounds(start_coord, min, max)) {
    RCLCPP_WARN(get_logger(), "Current position outside Octomap bounds.");
    return;
  }
  if (!isFreeCoordInMap(map.get(), start_coord, min, max, clearance_radius_)) {
    octomap::OcTreeKey start_key;
    if (map->coordToKeyChecked(start_coord, start_key) &&
        findNearestFreeKey(start_key, &start_key)) {
      // Snap start to the closest free cell when current voxel is invalid.
      const octomap::point3d free_coord = map->keyToCoord(start_key);
      start = Eigen::Vector3d(free_coord.x(), free_coord.y(), free_coord.z());
    } else {
      RCLCPP_WARN(get_logger(), "Could not find free start cell.");
      return;
    }
  }

  std::vector<Eigen::Vector3d> path;
  if (!planPathOmpl(start, goal_position_, &path)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "OMPL failed to find a path.");
    return;
  }

  path = simplifyPointPath(path);

  mav_trajectory_generation::Trajectory trajectory;
  if (!buildTrajectoryFromPath(path, &trajectory)) {
    RCLCPP_WARN(get_logger(), "Failed to build trajectory.");
    return;
  }
  publishTrajectory(trajectory);
  if (publish_planned_path_) {
    publishPlannedPath(path);
  }

  goal_position_ = path.back();
  has_goal_ = true;
  force_replan_ = false;
  last_plan_time_ = now;

  if (visualize_) {
    publishVisualization(path, clusters, &goal_key);
  }
}

bool PathPlannerNode::planToPosition(
    const Eigen::Vector3d &target,
    const std::vector<FrontierCluster> &clusters) {
  // --- Backtracking plan ---
  std::shared_ptr<octomap::OcTree> map;
  octomap::point3d min;
  octomap::point3d max;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
    min = map_min_;
    max = map_max_;
  }
  if (!map) {
    return false;
  }

  Eigen::Vector3d start = current_position_;
  const octomap::point3d start_coord(start.x(), start.y(), start.z());
  if (!coordInBounds(start_coord, min, max)) {
    return false;
  }
  if (!isFreeCoordInMap(map.get(), start_coord, min, max, clearance_radius_)) {
    octomap::OcTreeKey start_key;
    if (map->coordToKeyChecked(start_coord, start_key) &&
        findNearestFreeKey(start_key, &start_key)) {
      const octomap::point3d free_coord = map->keyToCoord(start_key);
      start = Eigen::Vector3d(free_coord.x(), free_coord.y(), free_coord.z());
    } else {
      return false;
    }
  }

  Eigen::Vector3d goal = target;
  const octomap::point3d goal_coord(goal.x(), goal.y(), goal.z());
  if (!coordInBounds(goal_coord, min, max)) {
    return false;
  }
  if (!isFreeCoordInMap(map.get(), goal_coord, min, max, clearance_radius_)) {
    octomap::OcTreeKey goal_key;
    if (map->coordToKeyChecked(goal_coord, goal_key) &&
        findNearestFreeKey(goal_key, &goal_key)) {
      const octomap::point3d free_coord = map->keyToCoord(goal_key);
      goal = Eigen::Vector3d(free_coord.x(), free_coord.y(), free_coord.z());
    } else {
      return false;
    }
  }

  std::vector<Eigen::Vector3d> path;
  if (!planPathOmpl(start, goal, &path)) {
    return false;
  }

  path = simplifyPointPath(path);
  if (path.size() < 2) {
    return false;
  }

  mav_trajectory_generation::Trajectory trajectory;
  if (!buildTrajectoryFromPath(path, &trajectory)) {
    return false;
  }
  publishTrajectory(trajectory);
  if (publish_planned_path_) {
    publishPlannedPath(path);
  }

  goal_position_ = path.back();
  has_goal_ = true;
  force_replan_ = false;
  last_plan_time_ = get_clock()->now();

  if (visualize_) {
    octomap::OcTreeKey goal_key;
    if (map->coordToKeyChecked(
            octomap::point3d(path.back().x(), path.back().y(), path.back().z()),
            goal_key)) {
      publishVisualization(path, clusters, &goal_key);
    } else {
      publishVisualization(path, clusters, nullptr);
    }
  }

  RCLCPP_INFO(get_logger(), "Backtracking to prior goal.");
  return true;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
