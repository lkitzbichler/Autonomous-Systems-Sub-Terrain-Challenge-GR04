#include "pathplanner.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <octomap_msgs/conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace {

double distance3(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double normalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
  geometry_msgs::msg::Quaternion q;
  const double half = yaw * 0.5;
  q.w = std::cos(half);
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  return q;
}

}  // namespace

PathPlannerNode::PathPlannerNode()
: rclcpp::Node("path_planner") {
  command_topic_ = declare_parameter<std::string>("command_topic", "statemachine/cmd/path_planning");
  frontier_topic_ = declare_parameter<std::string>("frontier_topic", "frontier_target");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "current_state_est");
  octomap_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_binary");
  trajectory_topic_ = declare_parameter<std::string>("trajectory_topic", "command/trajectory");
  ready_topic_ = declare_parameter<std::string>("ready_topic", "path_planning/ready");
  goal_reached_topic_ = declare_parameter<std::string>("goal_reached_topic", "path_planning/goal_reached");
  map_frame_ = declare_parameter<std::string>("map_frame", "world");

  auto_start_ = declare_parameter<bool>("auto_start", true);
  require_map_ = declare_parameter<bool>("require_map", true);
  hold_when_idle_ = declare_parameter<bool>("hold_when_idle", true);
  simplify_path_ = declare_parameter<bool>("simplify_path", true);
  treat_unknown_as_occupied_ = declare_parameter<bool>("treat_unknown_as_occupied", false);

  publish_rate_ = declare_parameter<double>("publish_rate", publish_rate_);
  planner_rate_ = declare_parameter<double>("planner_rate", planner_rate_);
  max_planning_time_ = declare_parameter<double>("max_planning_time", max_planning_time_);
  planner_range_ = declare_parameter<double>("planner_range", planner_range_);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", goal_tolerance_);
  waypoint_reached_dist_ = declare_parameter<double>("waypoint_reached_dist", waypoint_reached_dist_);
  lookahead_dist_ = declare_parameter<double>("lookahead_dist", lookahead_dist_);
  cruise_speed_ = declare_parameter<double>("cruise_speed", cruise_speed_);
  collision_radius_ = declare_parameter<double>("collision_radius", collision_radius_);
  occupancy_threshold_ = declare_parameter<double>("occupancy_threshold", occupancy_threshold_);
  map_bound_padding_ = declare_parameter<double>("map_bound_padding", map_bound_padding_);
  path_interpolation_resolution_ = declare_parameter<double>("path_interpolation_resolution", path_interpolation_resolution_);
  yaw_offset_ = declare_parameter<double>("yaw_offset", yaw_offset_);
  tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", tf_timeout_sec_);

  default_bounds_ = declare_parameter<std::vector<double>>("default_bounds", default_bounds_);
  home_position_ = declare_parameter<std::vector<double>>("home_position", home_position_);

  if (default_bounds_.size() != 6) {
    RCLCPP_WARN(get_logger(), "default_bounds must have 6 entries, using fallback.");
    default_bounds_ = {-50.0, 50.0, -50.0, 50.0, 0.0, 20.0};
  }
  if (home_position_.size() != 3) {
    RCLCPP_WARN(get_logger(), "home_position must have 3 entries, using fallback.");
    home_position_ = {-38.02, 10.0, 6.57};
  }

  if (publish_rate_ <= 0.0) {
    publish_rate_ = 20.0;
  }
  if (planner_rate_ <= 0.0) {
    planner_rate_ = 1.0;
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  command_sub_ = create_subscription<std_msgs::msg::UInt8>(
    command_topic_, 10,
    std::bind(&PathPlannerNode::onCommand, this, std::placeholders::_1));

  frontier_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    frontier_topic_, 10,
    std::bind(&PathPlannerNode::onFrontierTarget, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, 20,
    std::bind(&PathPlannerNode::onOdom, this, std::placeholders::_1));

  octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
    octomap_topic_, rclcpp::QoS(1).transient_local(),
    std::bind(&PathPlannerNode::onOctomap, this, std::placeholders::_1));

  traj_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    trajectory_topic_, 10);

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "planned_path", 10);

  ready_pub_ = create_publisher<std_msgs::msg::Bool>(ready_topic_, 10);
  goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>(goal_reached_topic_, 10);

  planner_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / planner_rate_),
    std::bind(&PathPlannerNode::planTimer, this));

  publish_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_),
    std::bind(&PathPlannerNode::publishTimer, this));

  ready_timer_ = create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&PathPlannerNode::readyTimer, this));

  RCLCPP_INFO(get_logger(), "path_planner ready (command: %s, frontier: %s)",
              command_topic_.c_str(), frontier_topic_.c_str());
}

void PathPlannerNode::onCommand(const std_msgs::msg::UInt8::SharedPtr msg) {
  if (!msg) {
    return;
  }

  const auto cmd = static_cast<Command>(msg->data);
  switch (cmd) {
    case Command::START:
      active_ = true;
      return_home_mode_ = false;
      if (have_goal_) {
        need_replan_ = true;
      }
      RCLCPP_INFO(get_logger(), "command START");
      break;
    case Command::RETURN_HOME: {
      geometry_msgs::msg::Point home;
      home.x = home_position_[0];
      home.y = home_position_[1];
      home.z = home_position_[2];
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        goal_position_ = home;
        have_goal_ = true;
      }
      active_ = true;
      return_home_mode_ = true;
      need_replan_ = true;
      RCLCPP_INFO(get_logger(), "command RETURN_HOME");
      break;
    }
    case Command::STOP:
    case Command::HOLD:
    case Command::LAND:
      active_ = false;
      return_home_mode_ = false;
      {
        std::lock_guard<std::mutex> lock(path_mutex_);
        path_active_ = false;
        path_points_.clear();
        path_index_ = 0;
      }
      RCLCPP_INFO(get_logger(), "command STOP/HOLD/LAND");
      break;
    default:
      break;
  }
}

void PathPlannerNode::onFrontierTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!msg) {
    return;
  }

  if (return_home_mode_) {
    return;
  }

  geometry_msgs::msg::PoseStamped goal_pose = *msg;
  const auto &frame = goal_pose.header.frame_id;
  if (!frame.empty() && frame != map_frame_) {
    try {
      const auto timeout = tf2::durationFromSec(tf_timeout_sec_);
      goal_pose = tf_buffer_->transform(goal_pose, map_frame_, timeout);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(),
                  "TF transform failed (%s -> %s): %s",
                  frame.c_str(), map_frame_.c_str(), ex.what());
      return;
    }
  }

  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_position_ = goal_pose.pose.position;
    have_goal_ = true;
  }

  if (auto_start_) {
    active_ = true;
  }

  need_replan_ = true;
}

void PathPlannerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!msg) {
    return;
  }

  std::lock_guard<std::mutex> lock(odom_mutex_);
  current_position_ = msg->pose.pose.position;
  current_velocity_ = msg->twist.twist.linear;
  have_odom_ = true;
}

void PathPlannerNode::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg) {
  if (!msg) {
    return;
  }

  octomap::AbstractOcTree *tree = nullptr;
  if (msg->binary) {
    tree = octomap_msgs::binaryMsgToMap(*msg);
  } else {
    tree = octomap_msgs::fullMsgToMap(*msg);
  }

  if (!tree) {
    RCLCPP_WARN(get_logger(), "Failed to convert octomap message.");
    return;
  }

  auto *octree = dynamic_cast<octomap::OcTree *>(tree);
  if (!octree) {
    RCLCPP_WARN(get_logger(), "Octomap message was not an OcTree.");
    delete tree;
    return;
  }

  std::shared_ptr<octomap::OcTree> tree_ptr(octree);
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    octree_ = tree_ptr;
    have_map_ = true;
    updateInflationOffsets(octree->getResolution());
  }
}

void PathPlannerNode::planTimer() {
  if (!active_) {
    return;
  }
  if (planning_ || !need_replan_) {
    return;
  }

  if (!have_odom_) {
    return;
  }
  if (require_map_ && !have_map_) {
    return;
  }
  if (!have_goal_) {
    return;
  }

  planning_ = true;
  const bool success = planPath();
  if (!success) {
    RCLCPP_WARN(get_logger(), "Failed to plan path.");
  }
  planning_ = false;
}

bool PathPlannerNode::planPath() {
  geometry_msgs::msg::Point start;
  geometry_msgs::msg::Point goal;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!have_odom_) {
      return false;
    }
    start = current_position_;
  }
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!have_goal_) {
      return false;
    }
    goal = goal_position_;
  }

  std::shared_ptr<octomap::OcTree> tree_snapshot;
  std::vector<octomap::point3d> offsets_snapshot;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    tree_snapshot = octree_;
    offsets_snapshot = inflation_offsets_;
  }

  if (require_map_ && !tree_snapshot) {
    return false;
  }

  double min_x = default_bounds_[0];
  double max_x = default_bounds_[1];
  double min_y = default_bounds_[2];
  double max_y = default_bounds_[3];
  double min_z = default_bounds_[4];
  double max_z = default_bounds_[5];

  if (tree_snapshot && tree_snapshot->size() > 0) {
    tree_snapshot->getMetricMin(min_x, min_y, min_z);
    tree_snapshot->getMetricMax(max_x, max_y, max_z);
  }

  min_x -= map_bound_padding_;
  min_y -= map_bound_padding_;
  min_z -= map_bound_padding_;
  max_x += map_bound_padding_;
  max_y += map_bound_padding_;
  max_z += map_bound_padding_;

  auto space = std::make_shared<ob::RealVectorStateSpace>(3);
  ob::RealVectorBounds bounds(3);
  bounds.setLow(0, min_x);
  bounds.setHigh(0, max_x);
  bounds.setLow(1, min_y);
  bounds.setHigh(1, max_y);
  bounds.setLow(2, min_z);
  bounds.setHigh(2, max_z);
  space->setBounds(bounds);

  auto si = std::make_shared<ob::SpaceInformation>(space);
  si->setStateValidityChecker(
    [this, tree_snapshot, offsets_snapshot](const ob::State *state) {
      if (!tree_snapshot) {
        return true;
      }
      const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
      return isCollisionFree(*tree_snapshot, offsets_snapshot,
                             s->values[0], s->values[1], s->values[2]);
    });
  si->setup();

  ob::ScopedState<> start_state(space);
  start_state[0] = start.x;
  start_state[1] = start.y;
  start_state[2] = start.z;

  ob::ScopedState<> goal_state(space);
  goal_state[0] = goal.x;
  goal_state[1] = goal.y;
  goal_state[2] = goal.z;

  if (tree_snapshot) {
    if (!isCollisionFree(*tree_snapshot, offsets_snapshot, start.x, start.y, start.z)) {
      RCLCPP_WARN(get_logger(), "Start state is in collision.");
    }
    if (!isCollisionFree(*tree_snapshot, offsets_snapshot, goal.x, goal.y, goal.z)) {
      RCLCPP_WARN(get_logger(), "Goal state is in collision.");
    }
  }

  auto pdef = std::make_shared<ob::ProblemDefinition>(si);
  pdef->setStartAndGoalStates(start_state, goal_state, goal_tolerance_);

  auto planner = std::make_shared<og::RRTConnect>(si);
  planner->setRange(planner_range_);
  planner->setProblemDefinition(pdef);
  planner->setup();

  const auto solved = planner->solve(ob::timedPlannerTerminationCondition(max_planning_time_));
  if (!solved) {
    return false;
  }

  auto path = std::dynamic_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
  if (!path) {
    return false;
  }

  if (simplify_path_) {
    og::PathSimplifier simplifier(si);
    simplifier.simplifyMax(*path);
  }

  if (path_interpolation_resolution_ > 0.0) {
    const double length = path->length();
    const int count = std::max(2, static_cast<int>(std::ceil(length / path_interpolation_resolution_)) + 1);
    path->interpolate(count);
  }

  std::vector<geometry_msgs::msg::Point> points;
  points.reserve(path->getStateCount());
  for (const auto *state : path->getStates()) {
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
    geometry_msgs::msg::Point p;
    p.x = s->values[0];
    p.y = s->values[1];
    p.z = s->values[2];
    points.push_back(p);
  }

  if (points.empty()) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_points_ = std::move(points);
    path_index_ = 0;
    path_active_ = true;
  }

  need_replan_ = false;
  publishPlannedPath();
  return true;
}

bool PathPlannerNode::isCollisionFree(const octomap::OcTree &tree,
                                      const std::vector<octomap::point3d> &offsets,
                                      double x, double y, double z) const {
  if (offsets.empty()) {
    return true;
  }

  for (const auto &off : offsets) {
    const double px = x + off.x();
    const double py = y + off.y();
    const double pz = z + off.z();
    auto *node = tree.search(px, py, pz);
    if (!node) {
      if (treat_unknown_as_occupied_) {
        return false;
      }
      continue;
    }
    if (node->getOccupancy() >= occupancy_threshold_) {
      return false;
    }
  }
  return true;
}

void PathPlannerNode::updateInflationOffsets(double resolution) {
  inflation_offsets_.clear();

  if (collision_radius_ <= 0.0) {
    inflation_offsets_.emplace_back(0.0, 0.0, 0.0);
    return;
  }

  const double step = (resolution > 1e-6) ? resolution : 0.1;
  const double radius = collision_radius_;
  const double radius_sq = radius * radius;

  for (double dx = -radius; dx <= radius; dx += step) {
    for (double dy = -radius; dy <= radius; dy += step) {
      for (double dz = -radius; dz <= radius; dz += step) {
        const double dist_sq = dx * dx + dy * dy + dz * dz;
        if (dist_sq <= radius_sq) {
          inflation_offsets_.emplace_back(dx, dy, dz);
        }
      }
    }
  }

  if (inflation_offsets_.empty()) {
    inflation_offsets_.emplace_back(0.0, 0.0, 0.0);
  }
}

void PathPlannerNode::publishTrajectoryPoint(const geometry_msgs::msg::Point &target,
                                             const geometry_msgs::msg::Vector3 &velocity,
                                             double yaw) {
  trajectory_msgs::msg::MultiDOFJointTrajectory msg;
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;

  yaw = normalizeAngle(yaw + yaw_offset_);

  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
  geometry_msgs::msg::Transform transform;
  transform.translation.x = target.x;
  transform.translation.y = target.y;
  transform.translation.z = target.z;
  transform.rotation = yawToQuaternion(yaw);
  point.transforms.push_back(transform);

  geometry_msgs::msg::Twist vel;
  vel.linear = velocity;
  point.velocities.push_back(vel);

  geometry_msgs::msg::Twist acc;
  acc.linear.x = 0.0;
  acc.linear.y = 0.0;
  acc.linear.z = 0.0;
  point.accelerations.push_back(acc);

  msg.points.push_back(point);
  traj_pub_->publish(msg);
}

void PathPlannerNode::publishHoldPosition() {
  geometry_msgs::msg::Point hold;
  geometry_msgs::msg::Vector3 vel;
  double yaw = last_yaw_;

  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!have_odom_) {
      return;
    }
    hold = current_position_;
    const double vxy = std::hypot(current_velocity_.x, current_velocity_.y);
    if (vxy > 1e-3) {
      yaw = std::atan2(current_velocity_.y, current_velocity_.x);
      last_yaw_ = yaw;
    }
  }

  vel.x = 0.0;
  vel.y = 0.0;
  vel.z = 0.0;
  publishTrajectoryPoint(hold, vel, yaw);
}

void PathPlannerNode::publishPlannedPath() {
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = now();
  path_msg.header.frame_id = map_frame_;

  std::vector<geometry_msgs::msg::Point> path_copy;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_copy = path_points_;
  }

  path_msg.poses.reserve(path_copy.size());
  for (const auto &p : path_copy) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position = p;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_pub_->publish(path_msg);
}

void PathPlannerNode::publishTimer() {
  geometry_msgs::msg::Point current;
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!have_odom_) {
      return;
    }
    current = current_position_;
  }

  if (!active_) {
    return;
  }

  std::vector<geometry_msgs::msg::Point> path_copy;
  size_t index = 0;
  bool path_active = false;
  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_copy = path_points_;
    index = path_index_;
    path_active = path_active_;
  }

  if (!path_active || path_copy.empty()) {
    if (hold_when_idle_) {
      publishHoldPosition();
    }
    return;
  }

  while (index < path_copy.size() && distance3(current, path_copy[index]) < waypoint_reached_dist_) {
    index++;
  }

  if (index >= path_copy.size()) {
    {
      std::lock_guard<std::mutex> lock(path_mutex_);
      path_index_ = index;
      path_active_ = false;
    }

    if (return_home_mode_) {
      std_msgs::msg::Bool done;
      done.data = true;
      goal_reached_pub_->publish(done);
      return_home_mode_ = false;
    }

    if (hold_when_idle_) {
      publishHoldPosition();
    }
    return;
  }

  {
    std::lock_guard<std::mutex> lock(path_mutex_);
    path_index_ = index;
  }

  size_t target_index = index;
  if (lookahead_dist_ > 0.0) {
    for (size_t i = index; i < path_copy.size(); ++i) {
      if (distance3(current, path_copy[i]) >= lookahead_dist_) {
        target_index = i;
        break;
      }
    }
  }

  const auto target = path_copy[target_index];
  const double dx = target.x - current.x;
  const double dy = target.y - current.y;
  const double dz = target.z - current.z;
  const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  geometry_msgs::msg::Vector3 vel;
  if (dist > 1e-3) {
    const double scale = cruise_speed_ / dist;
    vel.x = dx * scale;
    vel.y = dy * scale;
    vel.z = dz * scale;
  } else {
    vel.x = 0.0;
    vel.y = 0.0;
    vel.z = 0.0;
  }

  double yaw = last_yaw_;
  const double vxy = std::hypot(vel.x, vel.y);
  if (vxy > 1e-3) {
    yaw = std::atan2(vel.y, vel.x);
    last_yaw_ = yaw;
  }

  publishTrajectoryPoint(target, vel, yaw);
}

void PathPlannerNode::readyTimer() {
  bool ready = have_odom_ && (!require_map_ || have_map_);
  std_msgs::msg::Bool msg;
  msg.data = ready;
  ready_pub_->publish(msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
