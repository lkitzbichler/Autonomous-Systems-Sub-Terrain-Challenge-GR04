#include "pathplanner.h"

#include <octomap_msgs/conversions.h>

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/ros_conversions.h>
#include <mav_trajectory_generation/timing.h>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>

namespace {

bool coordInBounds(const octomap::point3d &coord, const octomap::point3d &min,
                   const octomap::point3d &max) {
  return coord.x() >= min.x() && coord.x() <= max.x() &&
         coord.y() >= min.y() && coord.y() <= max.y() &&
         coord.z() >= min.z() && coord.z() <= max.z();
}

bool isFreeCoordInMap(const octomap::OcTree *tree,
                      const octomap::point3d &coord,
                      const octomap::point3d &min,
                      const octomap::point3d &max,
                      double clearance_radius) {
  if (!tree) {
    return false;
  }
  if (!coordInBounds(coord, min, max)) {
    return false;
  }

  auto *node = tree->search(coord);
  if (!node || tree->isNodeOccupied(node)) {
    return false;
  }

  if (clearance_radius <= 1e-6) {
    return true;
  }

  const double res = tree->getResolution();
  const int steps = static_cast<int>(std::ceil(clearance_radius / res));
  for (int dx = -steps; dx <= steps; ++dx) {
    for (int dy = -steps; dy <= steps; ++dy) {
      for (int dz = -steps; dz <= steps; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        octomap::point3d c(coord.x() + dx * res, coord.y() + dy * res,
                           coord.z() + dz * res);
        if (!coordInBounds(c, min, max)) {
          return false;
        }
        auto *n = tree->search(c);
        if (n && tree->isNodeOccupied(n)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool isFreeKeyInMap(const octomap::OcTree *tree, const octomap::OcTreeKey &key,
                    const octomap::point3d &min, const octomap::point3d &max,
                    double clearance_radius) {
  if (!tree) {
    return false;
  }
  const octomap::point3d coord = tree->keyToCoord(key);
  return isFreeCoordInMap(tree, coord, min, max, clearance_radius);
}

bool isFrontierKeyInMap(const octomap::OcTree *tree,
                        const octomap::OcTreeKey &key,
                        const octomap::point3d &min,
                        const octomap::point3d &max) {
  if (!tree) {
    return false;
  }
  const double res = tree->getResolution();
  const octomap::point3d coord = tree->keyToCoord(key);
  const int dirs[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};
  for (const auto &d : dirs) {
    octomap::point3d ncoord(coord.x() + d[0] * res, coord.y() + d[1] * res,
                            coord.z() + d[2] * res);
    if (!coordInBounds(ncoord, min, max)) {
      return true;
    }
    auto *n = tree->search(ncoord);
    if (!n) {
      return true;
    }
  }
  return false;
}

} // namespace

std::size_t PathPlannerNode::KeyHash::operator()(const octomap::OcTreeKey &key) const noexcept {
  const uint64_t packed =
      (static_cast<uint64_t>(key.k[0]) << 32) |
      (static_cast<uint64_t>(key.k[1]) << 16) |
      static_cast<uint64_t>(key.k[2]);
  return std::hash<uint64_t>{}(packed);
}

bool PathPlannerNode::KeyEq::operator()(const octomap::OcTreeKey &a,
                                       const octomap::OcTreeKey &b) const noexcept {
  return a.k[0] == b.k[0] && a.k[1] == b.k[1] && a.k[2] == b.k[2];
}

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

  auto_start_ = declare_parameter<bool>("auto_start", true);
  visualize_ = declare_parameter<bool>("visualize", true);
  replan_while_moving_ = declare_parameter<bool>("replan_while_moving", false);
  start_on_waypoint_done_ =
      declare_parameter<bool>("start_on_waypoint_done", false);

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
  clearance_radius_ = declare_parameter<double>("clearance_radius", 0.0);
  path_simplify_distance_ =
      declare_parameter<double>("path_simplify_distance", 1.0);
  use_line_of_sight_prune_ =
      declare_parameter<bool>("use_line_of_sight_prune", true);
  line_of_sight_step_ =
      declare_parameter<double>("line_of_sight_step", 0.5);

  frontier_min_cluster_size_ =
      declare_parameter<int>("frontier_min_cluster_size", 5);
  max_frontier_nodes_ = declare_parameter<int>("max_frontier_nodes", 50000);
  max_astar_nodes_ = declare_parameter<int>("max_astar_nodes", 200000);

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

  last_map_stamp_ = msg->header.stamp;
}

void PathPlannerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_odom_ = *msg;
  current_position_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
      msg->pose.pose.position.z;
  current_velocity_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
      msg->twist.twist.linear.z;
  have_odom_ = true;

  const Eigen::Vector3d vel(current_velocity_.x(), current_velocity_.y(), 0.0);
  if (use_velocity_heading_ && vel.head<2>().norm() > forward_speed_threshold_) {
    forward_dir_ = vel.normalized();
    if (invert_forward_dir_) {
      forward_dir_ *= -1.0;
    }
    return;
  }

  const auto &q = msg->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  forward_dir_ = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
  if (invert_forward_dir_) {
    forward_dir_ *= -1.0;
  }
  last_yaw_ = yaw + yaw_offset_;
}

void PathPlannerNode::tick() {
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
    const double dist = (current_position_ - goal_position_).norm();
    if (dist < goal_reached_tolerance_) {
      if (backtrack_enabled_) {
        if (backtrack_stack_.empty() ||
            (goal_position_ - backtrack_stack_.back()).norm() >
                backtrack_min_distance_) {
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

  std::vector<FrontierCluster> clusters;
  if (!computeFrontierClusters(&clusters) || clusters.empty()) {
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

  octomap::OcTreeKey start_key;
  if (!map->coordToKeyChecked(
          octomap::point3d(current_position_.x(), current_position_.y(),
                           current_position_.z()),
          start_key)) {
    RCLCPP_WARN(get_logger(), "Current position outside Octomap bounds.");
    return;
  }

  if (!isFreeKeyInMap(map.get(), start_key, min, max, clearance_radius_)) {
    if (!findNearestFreeKey(start_key, &start_key)) {
      RCLCPP_WARN(get_logger(), "Could not find free start cell.");
      return;
    }
  }

  std::vector<octomap::OcTreeKey> path_keys;
  if (!planPathAStar(start_key, goal_key, &path_keys)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "A* failed to find a path.");
    return;
  }

  std::vector<Eigen::Vector3d> path = simplifyPath(path_keys);
  if (path.size() < 2) {
    RCLCPP_WARN(get_logger(), "Path too short after simplification.");
    return;
  }

  mav_trajectory_generation::Trajectory trajectory;
  if (!buildTrajectoryFromPath(path, &trajectory)) {
    RCLCPP_WARN(get_logger(), "Failed to build trajectory.");
    return;
  }
  publishTrajectory(trajectory);

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

  octomap::OcTreeKey start_key;
  if (!map->coordToKeyChecked(
          octomap::point3d(current_position_.x(), current_position_.y(),
                           current_position_.z()),
          start_key)) {
    return false;
  }

  if (!isFreeKeyInMap(map.get(), start_key, min, max, clearance_radius_)) {
    if (!findNearestFreeKey(start_key, &start_key)) {
      return false;
    }
  }

  octomap::OcTreeKey goal_key;
  if (!map->coordToKeyChecked(
          octomap::point3d(target.x(), target.y(), target.z()), goal_key)) {
    return false;
  }

  if (!isFreeKeyInMap(map.get(), goal_key, min, max, clearance_radius_)) {
    if (!findNearestFreeKey(goal_key, &goal_key)) {
      return false;
    }
  }

  std::vector<octomap::OcTreeKey> path_keys;
  if (!planPathAStar(start_key, goal_key, &path_keys)) {
    return false;
  }

  std::vector<Eigen::Vector3d> path = simplifyPath(path_keys);
  if (path.size() < 2) {
    return false;
  }

  mav_trajectory_generation::Trajectory trajectory;
  if (!buildTrajectoryFromPath(path, &trajectory)) {
    return false;
  }
  publishTrajectory(trajectory);

  goal_position_ = path.back();
  has_goal_ = true;
  force_replan_ = false;
  last_plan_time_ = get_clock()->now();

  if (visualize_) {
    publishVisualization(path, clusters, &goal_key);
  }

  RCLCPP_INFO(get_logger(), "Backtracking to prior goal.");
  return true;
}

bool PathPlannerNode::popBacktrackTarget(Eigen::Vector3d *target) {
  while (!backtrack_stack_.empty()) {
    const Eigen::Vector3d candidate = backtrack_stack_.back();
    backtrack_stack_.pop_back();
    if ((candidate - current_position_).norm() >= backtrack_min_distance_) {
      *target = candidate;
      return true;
    }
  }
  return false;
}

bool PathPlannerNode::computeFrontierClusters(
    std::vector<FrontierCluster> *clusters) {
  clusters->clear();

  std::shared_ptr<octomap::OcTree> map;
  octomap::point3d min;
  octomap::point3d max;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
    min = map_min_;
    max = map_max_;
  }
  if (!map || !have_bounds_) {
    return false;
  }

  octomap::OcTreeKey start_key;
  if (!map->coordToKeyChecked(
          octomap::point3d(current_position_.x(), current_position_.y(),
                           current_position_.z()),
          start_key)) {
    return false;
  }

  if (!isFreeKeyInMap(map.get(), start_key, min, max, clearance_radius_)) {
    if (!findNearestFreeKey(start_key, &start_key)) {
      return false;
    }
  }

  const double res = map->getResolution();
  const int dirs[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};

  std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> visited;
  std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> frontier_cells;

  std::queue<octomap::OcTreeKey> queue;
  queue.push(start_key);
  visited.insert(start_key);

  const octomap::point3d start_coord = map->keyToCoord(start_key);

  while (!queue.empty() && static_cast<int>(visited.size()) < max_frontier_nodes_) {
    const octomap::OcTreeKey key = queue.front();
    queue.pop();

    const octomap::point3d coord = map->keyToCoord(key);
    if ((coord - start_coord).norm() > frontier_max_distance_) {
      continue;
    }

    if (isFrontierKeyInMap(map.get(), key, min, max)) {
      frontier_cells.insert(key);
    }

    for (const auto &d : dirs) {
      octomap::point3d ncoord(coord.x() + d[0] * res, coord.y() + d[1] * res,
                              coord.z() + d[2] * res);
      if (!coordInBounds(ncoord, min, max)) {
        continue;
      }
      octomap::OcTreeKey nkey;
      if (!map->coordToKeyChecked(ncoord, nkey)) {
        continue;
      }
      if (visited.find(nkey) != visited.end()) {
        continue;
      }
      if (!isFreeKeyInMap(map.get(), nkey, min, max, clearance_radius_)) {
        continue;
      }
      visited.insert(nkey);
      queue.push(nkey);
    }
  }

  if (frontier_cells.empty()) {
    return false;
  }

  std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> frontier_visited;
  for (const auto &cell : frontier_cells) {
    if (frontier_visited.find(cell) != frontier_visited.end()) {
      continue;
    }

    FrontierCluster cluster;
    std::queue<octomap::OcTreeKey> cqueue;
    cqueue.push(cell);
    frontier_visited.insert(cell);

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

    while (!cqueue.empty()) {
      const octomap::OcTreeKey ck = cqueue.front();
      cqueue.pop();
      cluster.cells.push_back(ck);
      const octomap::point3d ccoord = map->keyToCoord(ck);
      centroid += Eigen::Vector3d(ccoord.x(), ccoord.y(), ccoord.z());

      const octomap::point3d base = map->keyToCoord(ck);
      for (const auto &d : dirs) {
        octomap::point3d ncoord(base.x() + d[0] * res, base.y() + d[1] * res,
                                base.z() + d[2] * res);
        octomap::OcTreeKey nkey;
        if (!map->coordToKeyChecked(ncoord, nkey)) {
          continue;
        }
        if (frontier_cells.find(nkey) == frontier_cells.end()) {
          continue;
        }
        if (frontier_visited.find(nkey) != frontier_visited.end()) {
          continue;
        }
        frontier_visited.insert(nkey);
        cqueue.push(nkey);
      }
    }

    if (static_cast<int>(cluster.cells.size()) < frontier_min_cluster_size_) {
      continue;
    }
    centroid /= static_cast<double>(cluster.cells.size());
    cluster.centroid = centroid;
    clusters->push_back(cluster);
  }

  return !clusters->empty();
}

bool PathPlannerNode::selectFrontierGoal(
    const std::vector<FrontierCluster> &clusters,
    octomap::OcTreeKey *goal_key) {
  if (clusters.empty()) {
    return false;
  }

  std::shared_ptr<octomap::OcTree> map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
  }
  if (!map) {
    return false;
  }

  double best_score = -std::numeric_limits<double>::infinity();
  double best_score_any = -std::numeric_limits<double>::infinity();
  octomap::OcTreeKey best_key;
  octomap::OcTreeKey best_key_any;
  bool found = false;
  bool found_any = false;

  for (const auto &cluster : clusters) {
    if (cluster.cells.empty()) {
      continue;
    }
    const double dist = (cluster.centroid - current_position_).norm();
    if (dist < min_goal_distance_) {
      continue;
    }
    const double size = static_cast<double>(cluster.cells.size());
    const double forward_score = forward_bias_weight_ * forwardDot(cluster.centroid);
    const double lateral_penalty = lateral_bias_weight_ * lateralDistance(cluster.centroid);
    const double score = frontier_score_size_weight_ * size -
                         frontier_score_distance_weight_ * dist +
                         forward_score - lateral_penalty;

    double best_cell_dist = std::numeric_limits<double>::infinity();
    octomap::OcTreeKey candidate_key;
    bool candidate_found = false;
    for (const auto &cell : cluster.cells) {
      const octomap::point3d coord = map->keyToCoord(cell);
      const double d =
          (Eigen::Vector3d(coord.x(), coord.y(), coord.z()) - current_position_)
              .norm();
      if (d < min_goal_distance_) {
        continue;
      }
      if (d < best_cell_dist) {
        best_cell_dist = d;
        candidate_key = cell;
        candidate_found = true;
      }
    }
    if (!candidate_found) {
      continue;
    }

    if (score > best_score_any) {
      best_score_any = score;
      best_key_any = candidate_key;
      found_any = true;
    }

    if (min_forward_cos_ > -1.0 && forwardDot(cluster.centroid) < min_forward_cos_) {
      continue;
    }

    if (score > best_score) {
      best_score = score;
      best_key = candidate_key;
      found = true;
    }
  }

  if (!found && found_any) {
    best_key = best_key_any;
    found = true;
  }
  if (!found) {
    return false;
  }
  if (min_goal_distance_ > 0.0) {
    const octomap::point3d coord = map->keyToCoord(best_key);
    const double d =
        (Eigen::Vector3d(coord.x(), coord.y(), coord.z()) - current_position_)
            .norm();
    if (d < min_goal_distance_) {
      return false;
    }
  }
  *goal_key = best_key;
  return true;
}

double PathPlannerNode::forwardDot(const Eigen::Vector3d &target) const {
  const Eigen::Vector3d dir = target - current_position_;
  const double norm = dir.head<2>().norm();
  if (norm < 1e-6) {
    return 0.0;
  }
  const Eigen::Vector3d dir_xy(dir.x() / norm, dir.y() / norm, 0.0);
  return forward_dir_.dot(dir_xy);
}

double PathPlannerNode::lateralDistance(const Eigen::Vector3d &target) const {
  const Eigen::Vector3d dir = target - current_position_;
  const double norm = dir.head<2>().norm();
  if (norm < 1e-6) {
    return 0.0;
  }
  const Eigen::Vector3d dir_xy(dir.x() / norm, dir.y() / norm, 0.0);
  const double forward = std::max(-1.0, std::min(1.0, forward_dir_.dot(dir_xy)));
  const double sin_angle = std::sqrt(std::max(0.0, 1.0 - forward * forward));
  return norm * sin_angle;
}

bool PathPlannerNode::planPathAStar(const octomap::OcTreeKey &start_key,
                                   const octomap::OcTreeKey &goal_key,
                                   std::vector<octomap::OcTreeKey> *path_out) {
  path_out->clear();
  if (KeyEq{}(start_key, goal_key)) {
    return false;
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
    return false;
  }

  const double res = map->getResolution();

  auto heuristic = [&](const octomap::OcTreeKey &key) {
    const octomap::point3d coord = map->keyToCoord(key);
    const octomap::point3d gcoord = map->keyToCoord(goal_key);
    return (coord - gcoord).norm();
  };

  struct Node {
    octomap::OcTreeKey key;
    double f = 0.0;
    double g = 0.0;
  };
  auto cmp = [](const Node &a, const Node &b) { return a.f > b.f; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);

  std::unordered_map<octomap::OcTreeKey, double, KeyHash, KeyEq> gscore;
  std::unordered_map<octomap::OcTreeKey, octomap::OcTreeKey, KeyHash, KeyEq>
      came_from;
  std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> closed;

  gscore[start_key] = 0.0;
  open.push(Node{start_key, heuristic(start_key), 0.0});

  const int dirs[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};

  int expanded = 0;
  while (!open.empty() && expanded < max_astar_nodes_) {
    const Node current = open.top();
    open.pop();

    if (closed.find(current.key) != closed.end()) {
      continue;
    }
    closed.insert(current.key);
    ++expanded;

    if (KeyEq{}(current.key, goal_key)) {
      octomap::OcTreeKey key = current.key;
      path_out->push_back(key);
      while (!KeyEq{}(key, start_key)) {
        key = came_from[key];
        path_out->push_back(key);
      }
      std::reverse(path_out->begin(), path_out->end());
      return true;
    }

    const octomap::point3d coord = map->keyToCoord(current.key);
    for (const auto &d : dirs) {
      octomap::point3d ncoord(coord.x() + d[0] * res, coord.y() + d[1] * res,
                              coord.z() + d[2] * res);
      if (!coordInBounds(ncoord, min, max)) {
        continue;
      }
      octomap::OcTreeKey nkey;
      if (!map->coordToKeyChecked(ncoord, nkey)) {
        continue;
      }
      if (closed.find(nkey) != closed.end()) {
        continue;
      }
      if (!isFreeKeyInMap(map.get(), nkey, min, max, clearance_radius_)) {
        continue;
      }
      const double step_cost =
          res * std::sqrt(static_cast<double>(d[0] * d[0] + d[1] * d[1] +
                                              d[2] * d[2]));
      const double tentative = gscore[current.key] + step_cost;
      auto it = gscore.find(nkey);
      if (it == gscore.end() || tentative < it->second) {
        gscore[nkey] = tentative;
        came_from[nkey] = current.key;
        const double f = tentative + heuristic(nkey);
        open.push(Node{nkey, f, tentative});
      }
    }
  }

  return false;
}

std::vector<Eigen::Vector3d> PathPlannerNode::simplifyPath(
    const std::vector<octomap::OcTreeKey> &path_keys) const {
  std::vector<Eigen::Vector3d> path;
  if (path_keys.empty()) {
    return path;
  }

  std::shared_ptr<octomap::OcTree> map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
  }
  if (!map) {
    return path;
  }

  path.reserve(path_keys.size());
  for (const auto &key : path_keys) {
    const octomap::point3d coord = map->keyToCoord(key);
    path.emplace_back(coord.x(), coord.y(), coord.z());
  }

  std::vector<Eigen::Vector3d> simplified;
  simplified.push_back(path.front());
  double accum = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    accum += (path[i] - simplified.back()).norm();
    if (accum >= path_simplify_distance_ || i + 1 == path.size()) {
      simplified.push_back(path[i]);
      accum = 0.0;
    }
  }

  if (simplified.size() < 2 && path.size() >= 2) {
    simplified.push_back(path.back());
  }

  if (use_line_of_sight_prune_) {
    return prunePathLineOfSight(simplified);
  }

  return simplified;
}

bool PathPlannerNode::isSegmentFree(const Eigen::Vector3d &start,
                                    const Eigen::Vector3d &goal) const {
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

  const Eigen::Vector3d delta = goal - start;
  const double length = delta.norm();
  if (length <= 1e-6) {
    return true;
  }

  const double step = std::max(0.05, line_of_sight_step_);
  const int steps = static_cast<int>(std::ceil(length / step));
  const Eigen::Vector3d dir = delta / length;

  for (int i = 0; i <= steps; ++i) {
    const Eigen::Vector3d p = start + dir * (static_cast<double>(i) * step);
    const octomap::point3d coord(p.x(), p.y(), p.z());
    if (!isFreeCoordInMap(map.get(), coord, min, max, clearance_radius_)) {
      return false;
    }
  }

  return true;
}

std::vector<Eigen::Vector3d> PathPlannerNode::prunePathLineOfSight(
    const std::vector<Eigen::Vector3d> &path) const {
  if (path.size() <= 2) {
    return path;
  }

  std::vector<Eigen::Vector3d> pruned;
  pruned.reserve(path.size());
  size_t i = 0;
  pruned.push_back(path.front());

  while (i + 1 < path.size()) {
    size_t best = i + 1;
    for (size_t j = path.size() - 1; j > i; --j) {
      if (isSegmentFree(path[i], path[j])) {
        best = j;
        break;
      }
    }
    pruned.push_back(path[best]);
    i = best;
  }

  return pruned;
}

bool PathPlannerNode::buildTrajectoryFromPath(
    const std::vector<Eigen::Vector3d> &path,
    mav_trajectory_generation::Trajectory *trajectory) {
  if (!trajectory || path.size() < 2) {
    return false;
  }

  std::vector<Eigen::Vector3d> waypoints = path;
  if (waypoints.size() == 2) {
    const Eigen::Vector3d mid = 0.5 * (waypoints.front() + waypoints.back());
    waypoints.insert(waypoints.begin() + 1, mid);
  }

  const int dimension = use_yaw_ ? 4 : 3;
  mav_trajectory_generation::Vertex::Vector vertices;

  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  if (use_yaw_) {
    double fallback_yaw = last_yaw_;
    const double vel_xy =
        std::hypot(current_velocity_.x(), current_velocity_.y());
    if (vel_xy > yaw_speed_threshold_) {
      fallback_yaw =
          std::atan2(current_velocity_.y(), current_velocity_.x()) + yaw_offset_;
    }
    const std::vector<double> yaw_values =
        computeYawFromPath(waypoints, fallback_yaw);

    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    Eigen::Vector4d start_pos_yaw;
    start_pos_yaw << waypoints.front().x(), waypoints.front().y(),
        waypoints.front().z(), yaw_values.front();
    start.makeStartOrEnd(start_pos_yaw, derivative_to_optimize);
    Eigen::Vector4d start_vel_yaw;
    start_vel_yaw << current_velocity_.x(), current_velocity_.y(),
        current_velocity_.z(), 0.0;
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        start_vel_yaw);
    vertices.push_back(start);

    for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
      mav_trajectory_generation::Vertex middle(dimension);
      Eigen::Vector4d pos_yaw;
      pos_yaw << waypoints[i].x(), waypoints[i].y(), waypoints[i].z(),
          yaw_values[i];
      middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                           pos_yaw);
      vertices.push_back(middle);
    }

    Eigen::Vector4d end_pos_yaw;
    end_pos_yaw << waypoints.back().x(), waypoints.back().y(),
        waypoints.back().z(), yaw_values.back();
    end.makeStartOrEnd(end_pos_yaw, derivative_to_optimize);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      Eigen::Vector4d::Zero());
    vertices.push_back(end);
  } else {
    mav_trajectory_generation::Vertex start(dimension), end(dimension);
    start.makeStartOrEnd(waypoints.front(), derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        current_velocity_);
    vertices.push_back(start);

    for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
      mav_trajectory_generation::Vertex middle(dimension);
      middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                           waypoints[i]);
      vertices.push_back(middle);
    }

    end.makeStartOrEnd(waypoints.back(), derivative_to_optimize);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      Eigen::Vector3d::Zero());
    vertices.push_back(end);
  }

  std::vector<double> segment_times =
      mav_trajectory_generation::estimateSegmentTimes(vertices, max_v_, max_a_);
  if (segment_times.size() + 1 != vertices.size()) {
    RCLCPP_WARN(get_logger(), "Invalid segment times (%zu) for %zu vertices.",
                segment_times.size(), vertices.size());
    return false;
  }

  mav_trajectory_generation::NonlinearOptimizationParameters parameters;
  const int N = 10;
  mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension,
                                                                    parameters);
  opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
  opt.addMaximumMagnitudeConstraint(
      mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
  opt.optimize();
  opt.getTrajectory(trajectory);

  return true;
}

std::vector<double> PathPlannerNode::computeYawFromPath(
    const std::vector<Eigen::Vector3d> &waypoints,
    double fallback_yaw) const {
  const size_t n = waypoints.size();
  std::vector<double> yaw_values(n, fallback_yaw);
  if (n < 2) {
    return yaw_values;
  }

  double last_yaw = fallback_yaw;
  for (size_t i = 0; i + 1 < n; ++i) {
    const Eigen::Vector3d delta = waypoints[i + 1] - waypoints[i];
    const double dx = delta.x();
    const double dy = delta.y();
    const double norm_xy = std::hypot(dx, dy);
    if (norm_xy > 1e-3) {
      last_yaw = std::atan2(dy, dx) + yaw_offset_;
    }
    yaw_values[i] = last_yaw;
  }
  yaw_values.back() = last_yaw;
  return yaw_values;
}

bool PathPlannerNode::publishTrajectory(
    const mav_trajectory_generation::Trajectory &trajectory) {
  mav_planning_msgs::msg::PolynomialTrajectory4D msg;
  if (!mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                      &msg)) {
    RCLCPP_WARN(get_logger(), "Failed to convert trajectory to 4D message.");
    return false;
  }
  msg.header.frame_id = world_frame_;
  msg.header.stamp = get_clock()->now();
  trajectory_pub_->publish(msg);
  return true;
}

void PathPlannerNode::publishVisualization(
    const std::vector<Eigen::Vector3d> &path,
    const std::vector<FrontierCluster> &clusters,
    const octomap::OcTreeKey *goal_key) {
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = world_frame_;
  path_marker.header.stamp = get_clock()->now();
  path_marker.ns = "path";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.scale.x = 0.05;
  path_marker.color.r = 0.1f;
  path_marker.color.g = 0.9f;
  path_marker.color.b = 0.1f;
  path_marker.color.a = 1.0f;
  for (const auto &p : path) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    path_marker.points.push_back(pt);
  }
  markers.markers.push_back(path_marker);

  visualization_msgs::msg::Marker frontier_marker;
  frontier_marker.header.frame_id = world_frame_;
  frontier_marker.header.stamp = get_clock()->now();
  frontier_marker.ns = "frontiers";
  frontier_marker.id = 1;
  frontier_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  frontier_marker.action = visualization_msgs::msg::Marker::ADD;
  frontier_marker.scale.x = 0.4;
  frontier_marker.scale.y = 0.4;
  frontier_marker.scale.z = 0.4;
  frontier_marker.color.r = 0.2f;
  frontier_marker.color.g = 0.4f;
  frontier_marker.color.b = 1.0f;
  frontier_marker.color.a = 0.8f;
  for (const auto &cluster : clusters) {
    geometry_msgs::msg::Point pt;
    pt.x = cluster.centroid.x();
    pt.y = cluster.centroid.y();
    pt.z = cluster.centroid.z();
    frontier_marker.points.push_back(pt);
  }
  markers.markers.push_back(frontier_marker);

  if (goal_key) {
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = world_frame_;
    goal_marker.header.stamp = get_clock()->now();
    goal_marker.ns = "goal";
    goal_marker.id = 2;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.scale.x = 0.6;
    goal_marker.scale.y = 0.6;
    goal_marker.scale.z = 0.6;
    goal_marker.color.r = 1.0f;
    goal_marker.color.g = 0.2f;
    goal_marker.color.b = 0.2f;
    goal_marker.color.a = 0.9f;

    std::shared_ptr<octomap::OcTree> map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = map_;
    }
    if (map) {
      const octomap::point3d coord = map->keyToCoord(*goal_key);
      goal_marker.pose.position.x = coord.x();
      goal_marker.pose.position.y = coord.y();
      goal_marker.pose.position.z = coord.z();
      markers.markers.push_back(goal_marker);
    }
  }

  markers_pub_->publish(markers);
}

bool PathPlannerNode::isFreeCoord(const octomap::point3d &coord) const {
  std::shared_ptr<octomap::OcTree> map;
  octomap::point3d min;
  octomap::point3d max;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
    min = map_min_;
    max = map_max_;
  }
  return isFreeCoordInMap(map.get(), coord, min, max, clearance_radius_);
}

bool PathPlannerNode::isFreeKey(const octomap::OcTreeKey &key) const {
  std::shared_ptr<octomap::OcTree> map;
  octomap::point3d min;
  octomap::point3d max;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
    min = map_min_;
    max = map_max_;
  }
  return isFreeKeyInMap(map.get(), key, min, max, clearance_radius_);
}

bool PathPlannerNode::isFrontierKey(const octomap::OcTreeKey &key) const {
  std::shared_ptr<octomap::OcTree> map;
  octomap::point3d min;
  octomap::point3d max;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
    min = map_min_;
    max = map_max_;
  }
  return isFrontierKeyInMap(map.get(), key, min, max);
}

bool PathPlannerNode::findNearestFreeKey(const octomap::OcTreeKey &seed_key,
                                         octomap::OcTreeKey *free_key) const {
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

  const double res = map->getResolution();
  const int dirs[6][3] = {{1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
                          {0, -1, 0}, {0, 0, 1},  {0, 0, -1}};

  std::queue<octomap::OcTreeKey> queue;
  std::unordered_set<octomap::OcTreeKey, KeyHash, KeyEq> visited;
  queue.push(seed_key);
  visited.insert(seed_key);

  while (!queue.empty() &&
         static_cast<int>(visited.size()) < max_frontier_nodes_) {
    const octomap::OcTreeKey key = queue.front();
    queue.pop();
    if (isFreeKeyInMap(map.get(), key, min, max, clearance_radius_)) {
      *free_key = key;
      return true;
    }
    const octomap::point3d coord = map->keyToCoord(key);
    for (const auto &d : dirs) {
      octomap::point3d ncoord(coord.x() + d[0] * res, coord.y() + d[1] * res,
                              coord.z() + d[2] * res);
      if (!coordInBounds(ncoord, min, max)) {
        continue;
      }
      octomap::OcTreeKey nkey;
      if (!map->coordToKeyChecked(ncoord, nkey)) {
        continue;
      }
      if (visited.find(nkey) != visited.end()) {
        continue;
      }
      visited.insert(nkey);
      queue.push(nkey);
    }
  }

  return false;
}

octomap::point3d PathPlannerNode::keyToCoord(
    const octomap::OcTreeKey &key) const {
  std::shared_ptr<octomap::OcTree> map;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    map = map_;
  }
  if (!map) {
    return octomap::point3d(0.0, 0.0, 0.0);
  }
  return map->keyToCoord(key);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
