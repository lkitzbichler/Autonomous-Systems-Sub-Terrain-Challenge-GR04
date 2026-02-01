#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>

#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/ros_conversions.h>

#include <Eigen/Dense>

#include <octomap/octomap.h>
#include <std_srvs/srv/empty.hpp>

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <mutex>
#include <queue>
#include <vector>

namespace {

constexpr uint8_t kUnknown = 0;
constexpr uint8_t kFree = 1;
constexpr uint8_t kOccupied = 2;
constexpr uint8_t kBlocked = 3;
constexpr double kPi = 3.14159265358979323846;

struct Grid3D {
  double resolution = 1.0;
  Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  int nx = 0;
  int ny = 0;
  int nz = 0;
  std::vector<uint8_t> base_state;  // unknown/free/occupied
  std::vector<uint8_t> plan_state;  // free/blocked (unknown stays unknown)

  inline bool valid(int ix, int iy, int iz) const {
    return ix >= 0 && iy >= 0 && iz >= 0 && ix < nx && iy < ny && iz < nz;
  }

  inline int index(int ix, int iy, int iz) const {
    return (iz * ny + iy) * nx + ix;
  }

  inline Eigen::Vector3d cellCenter(int ix, int iy, int iz) const {
    return origin + Eigen::Vector3d(
      (static_cast<double>(ix) + 0.5) * resolution,
      (static_cast<double>(iy) + 0.5) * resolution,
      (static_cast<double>(iz) + 0.5) * resolution);
  }
};

struct FrontierCluster {
  std::vector<int> indices;
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
};

struct StoredFrontier {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double gain = 0.0;
  int observations = 0;
  int fail_count = 0;
  bool visited = false;
  rclcpp::Time last_seen{0, 0, RCL_ROS_TIME};
};

struct AStarNode {
  int idx;
  double f;
};

struct AStarCompare {
  bool operator()(const AStarNode &a, const AStarNode &b) const {
    return a.f > b.f;
  }
};

}  // namespace

class ExplorationNode : public rclcpp::Node {
public:
  ExplorationNode()
  : rclcpp::Node("exploration_node")
  {
    octomap_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_binary");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "current_state_est");
    trajectory_topic_ = declare_parameter<std::string>("trajectory_topic", "trajectory");
    frame_id_ = declare_parameter<std::string>("frame_id", "world");

    local_radius_xy_ = declare_parameter<double>("local_radius_xy", 12.0);
    local_radius_z_ = declare_parameter<double>("local_radius_z", 6.0);
    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.0);  // 0 -> use octomap

    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.6);

    frontier_min_cluster_size_ = declare_parameter<int>("frontier_min_cluster_size", 6);
    frontier_gain_weight_ = declare_parameter<double>("frontier_gain_weight", 1.0);
    frontier_dist_weight_ = declare_parameter<double>("frontier_dist_weight", 1.0);
    info_gain_radius_ = declare_parameter<double>("info_gain_radius", 2.0);
    info_gain_unknown_weight_ = declare_parameter<double>("info_gain_unknown_weight", 1.0);
    cluster_size_weight_ = declare_parameter<double>("cluster_size_weight", 1.0);
    outside_unknown_as_frontier_ = declare_parameter<bool>("outside_unknown_as_frontier", true);
    outside_unknown_as_gain_ = declare_parameter<bool>("outside_unknown_as_gain", true);
    allow_unknown_traversal_ = declare_parameter<bool>("allow_unknown_traversal", true);
    unknown_traversal_penalty_ = declare_parameter<double>("unknown_traversal_penalty", 2.0);
    forward_bias_weight_ = declare_parameter<double>("forward_bias_weight", 2.0);
    min_forward_cos_ = declare_parameter<double>("min_forward_cos", 0.0);
    lateral_bias_weight_ = declare_parameter<double>("lateral_bias_weight", 0.0);
    forward_speed_threshold_ = declare_parameter<double>("forward_speed_threshold", 0.2);
    use_velocity_heading_ = declare_parameter<bool>("use_velocity_heading", false);
    invert_forward_dir_ = declare_parameter<bool>("invert_forward_dir", false);
    escape_fail_threshold_ = declare_parameter<int>("escape_fail_threshold", 3);
    escape_backoff_ = declare_parameter<double>("escape_backoff", 2.0);
    escape_up_ = declare_parameter<double>("escape_up", 0.5);
    goal_clearance_ = declare_parameter<double>("goal_clearance", 0.8);

    waypoint_spacing_ = declare_parameter<double>("waypoint_spacing", 1.5);
    max_v_ = declare_parameter<double>("max_v", 1.5);
    max_a_ = declare_parameter<double>("max_a", 1.0);
    use_yaw_ = declare_parameter<bool>("use_yaw", true);
    yaw_speed_threshold_ = declare_parameter<double>("yaw_speed_threshold", 0.2);
    yaw_offset_ = declare_parameter<double>("yaw_offset", 0.0);

    replan_period_sec_ = declare_parameter<double>("replan_period_sec", 1.0);

    lantern_topic_ = declare_parameter<std::string>("lantern_topic", "detected_lanterns");
    required_lanterns_ = declare_parameter<int>("required_lanterns", 4);
    stop_on_lanterns_ = declare_parameter<bool>("stop_on_lanterns", true);

    goal_history_size_ = declare_parameter<int>("goal_history_size", 5);
    goal_history_radius_ = declare_parameter<double>("goal_history_radius", 2.0);
    global_frontier_merge_radius_ = declare_parameter<double>("global_frontier_merge_radius", 2.0);
    global_frontier_ttl_sec_ = declare_parameter<double>("global_frontier_ttl_sec", 0.0);
    goal_reached_distance_ = declare_parameter<double>("goal_reached_distance", 1.0);
    max_goal_failures_ = declare_parameter<int>("max_goal_failures", 3);
    subgoal_margin_ = declare_parameter<double>("subgoal_margin", 0.5);
    min_goal_distance_ = declare_parameter<double>("min_goal_distance", 2.0);
    min_path_length_ = declare_parameter<double>("min_path_length", 2.0);

    start_immediately_ = declare_parameter<bool>("start_immediately", false);
    start_distance_ = declare_parameter<double>("start_distance", 1.0);
    start_speed_ = declare_parameter<double>("start_speed", 0.2);
    entrance_position_ = declare_parameter<std::vector<double>>("entrance_position", std::vector<double>{-400.0, 7.0, 15.0});
    start_on_waypoint_done_ = declare_parameter<bool>("start_on_waypoint_done", true);
    waypoint_done_topic_ = declare_parameter<std::string>("waypoint_done_topic", "basic_waypoint/done");

    publish_markers_ = declare_parameter<bool>("publish_markers", true);

    if (entrance_position_.size() != 3) {
      RCLCPP_WARN(get_logger(), "entrance_position must have 3 values, using default.");
      entrance_position_ = {-400.0, 7.0, 15.0};
    }

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, 5,
      std::bind(&ExplorationNode::onOctomap, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&ExplorationNode::onOdom, this, std::placeholders::_1));

    auto done_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    waypoint_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      waypoint_done_topic_, done_qos,
      std::bind(&ExplorationNode::onWaypointDone, this, std::placeholders::_1));

    lantern_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      lantern_topic_, 10,
      std::bind(&ExplorationNode::onLanterns, this, std::placeholders::_1));

    traj_pub_ = create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(
      trajectory_topic_, 10);

    stop_sampling_client_ = create_client<std_srvs::srv::Empty>("stop_sampling");

    if (publish_markers_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "exploration_markers", 5);
    }

    const auto period = std::chrono::duration<double>(replan_period_sec_);
    timer_ = create_wall_timer(period, std::bind(&ExplorationNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "exploration_node ready (octomap: %s, odom: %s)",
                octomap_topic_.c_str(), odom_topic_.c_str());
  }

private:
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> tree;
    if (msg->binary) {
      tree.reset(octomap_msgs::binaryMsgToMap(*msg));
    } else {
      tree.reset(octomap_msgs::fullMsgToMap(*msg));
    }

    if (!tree) {
      RCLCPP_WARN(get_logger(), "Failed to deserialize Octomap");
      return;
    }

    auto *octree = dynamic_cast<octomap::OcTree*>(tree.release());
    if (!octree) {
      RCLCPP_WARN(get_logger(), "Octomap is not an OcTree");
      return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    octree_.reset(octree);
    have_map_ = true;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    have_odom_ = true;

    const Eigen::Vector3d vel(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);
    if (use_velocity_heading_ && vel.head<2>().norm() > forward_speed_threshold_) {
      forward_dir_ = Eigen::Vector3d(vel.x(), vel.y(), 0.0).normalized();
      if (invert_forward_dir_) {
        forward_dir_ *= -1.0;
      }
      return;
    }

    const auto &q = msg->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    forward_dir_ = Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0).normalized();
    if (invert_forward_dir_) {
      forward_dir_ *= -1.0;
    }
  }

  void onLanterns(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    last_lantern_count_ = static_cast<int>(msg->poses.size());
    if (stop_on_lanterns_ && last_lantern_count_ >= required_lanterns_) {
      exploration_done_ = true;
    }
  }

  void onWaypointDone(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (waypoint_done_ != msg->data) {
      waypoint_done_ = msg->data;
      RCLCPP_INFO(get_logger(), "basic_waypoint/done -> %s", waypoint_done_ ? "true" : "false");
    }
  }

  void onTimer()
  {
    if (!have_map_ || !have_odom_) {
      return;
    }

    if (exploration_done_) {
      if (!done_stop_requested_) {
        requestStopSampling();
        done_stop_requested_ = true;
        RCLCPP_INFO(get_logger(), "Exploration completed (lanterns=%d).", last_lantern_count_);
      }
      return;
    }

    if (!exploration_started_) {
      const bool start_by_waypoint = start_on_waypoint_done_ ? waypoint_done_ : false;
      const bool start_by_entrance = start_on_waypoint_done_ ? false : isAtEntrance();
      if (start_immediately_ || start_by_waypoint || start_by_entrance) {
        exploration_started_ = true;
        RCLCPP_INFO(get_logger(), "Exploration started.");
      } else {
        return;
      }
    }

    const auto now = this->now();
    if ((now - last_plan_time_).seconds() < replan_period_sec_) {
      return;
    }

    std::shared_ptr<octomap::OcTree> map_copy;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map_copy = octree_;
    }

    if (!map_copy) {
      return;
    }

    Eigen::Vector3d current_pos(
      last_odom_.pose.pose.position.x,
      last_odom_.pose.pose.position.y,
      last_odom_.pose.pose.position.z);
    Eigen::Vector3d current_vel(
      last_odom_.twist.twist.linear.x,
      last_odom_.twist.twist.linear.y,
      last_odom_.twist.twist.linear.z);

    Grid3D grid;
    if (!buildLocalGrid(*map_copy, current_pos, grid)) {
      return;
    }

    std::vector<FrontierCluster> clusters = extractFrontiers(grid);
    updateGlobalFrontiers(clusters, grid, now);
    pruneGlobalFrontiers(now);
    if (have_active_goal_ && active_goal_index_ >= 0 &&
        static_cast<size_t>(active_goal_index_) >= global_frontiers_.size()) {
      active_goal_index_ = -1;
    }

    if (have_active_goal_ && isGoalReached(current_pos)) {
      markFrontierVisited(active_goal_index_);
      have_active_goal_ = false;
      active_goal_index_ = -1;
    }

    if (!have_active_goal_) {
      active_goal_index_ = selectFrontierIndex(current_pos);
      if (active_goal_index_ >= 0) {
        active_goal_ = global_frontiers_[static_cast<size_t>(active_goal_index_)].position;
        have_active_goal_ = true;
      } else if (clusters.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No frontiers available.");
        return;
      } else {
        const auto fallback_cluster = selectBestFrontier(clusters, grid, current_pos);
        if (fallback_cluster.indices.empty()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No suitable frontier cluster.");
          return;
        }
        active_goal_ = fallback_cluster.centroid;
        have_active_goal_ = true;
        active_goal_index_ = -1;
      }
    }

    const Eigen::Vector3d goal = projectGoalToGrid(current_pos, active_goal_, grid);
    if ((current_pos - goal).norm() < min_goal_distance_) {
      markFrontierVisited(active_goal_index_);
      have_active_goal_ = false;
      active_goal_index_ = -1;
      return;
    }

    std::vector<Eigen::Vector3d> path;
    if (!planPath(grid, current_pos, goal, path)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Planning failed.");
      markFrontierFailed(active_goal_index_);
      have_active_goal_ = false;
      active_goal_index_ = -1;
      consecutive_plan_failures_ += 1;
      if (shouldAttemptEscape()) {
        if (publishEscape(grid, current_pos, current_vel)) {
          last_plan_time_ = now;
          return;
        }
      }
      return;
    }

    std::vector<Eigen::Vector3d> waypoints = downsamplePath(path, waypoint_spacing_);
    if (waypoints.size() < 2) {
      RCLCPP_WARN(get_logger(), "Path too short to generate trajectory.");
      markFrontierFailed(active_goal_index_);
      have_active_goal_ = false;
      active_goal_index_ = -1;
      consecutive_plan_failures_ += 1;
      if (shouldAttemptEscape()) {
        if (publishEscape(grid, current_pos, current_vel)) {
          last_plan_time_ = now;
          return;
        }
      }
      return;
    }
    if ((waypoints.back() - waypoints.front()).norm() < min_path_length_) {
      RCLCPP_WARN(get_logger(), "Path length below minimum, skipping goal.");
      markFrontierFailed(active_goal_index_);
      have_active_goal_ = false;
      active_goal_index_ = -1;
      consecutive_plan_failures_ += 1;
      if (shouldAttemptEscape()) {
        if (publishEscape(grid, current_pos, current_vel)) {
          last_plan_time_ = now;
          return;
        }
      }
      return;
    }

    double fallback_yaw = last_yaw_;
    const double vel_xy = std::hypot(current_vel.x(), current_vel.y());
    if (vel_xy > yaw_speed_threshold_) {
      fallback_yaw = std::atan2(current_vel.y(), current_vel.x());
    }

    mav_trajectory_generation::Trajectory trajectory;
    if (!generateTrajectory(current_pos, current_vel, waypoints, fallback_yaw, &trajectory)) {
      RCLCPP_WARN(get_logger(), "Trajectory generation failed.");
      return;
    }

    if (use_yaw_) {
      const auto yaw_values = computeYawFromPath(waypoints, fallback_yaw);
      if (!yaw_values.empty()) {
        last_yaw_ = yaw_values.back();
      }
    }

    publishTrajectory(trajectory);
    publishMarkers(path, clusters, goal);
    rememberGoal(active_goal_);
    consecutive_plan_failures_ = 0;

    last_plan_time_ = now;
  }

  bool isAtEntrance() const
  {
    const Eigen::Vector3d current_pos(
      last_odom_.pose.pose.position.x,
      last_odom_.pose.pose.position.y,
      last_odom_.pose.pose.position.z);
    const Eigen::Vector3d current_vel(
      last_odom_.twist.twist.linear.x,
      last_odom_.twist.twist.linear.y,
      last_odom_.twist.twist.linear.z);
    const Eigen::Vector3d entrance(entrance_position_[0], entrance_position_[1], entrance_position_[2]);

    const double dist = (current_pos - entrance).norm();
    const double speed = current_vel.norm();
    return dist <= start_distance_ && speed <= start_speed_;
  }

  bool buildLocalGrid(const octomap::OcTree &tree, const Eigen::Vector3d &center, Grid3D &grid)
  {
    const double resolution = (grid_resolution_ > 0.0) ? grid_resolution_ : tree.getResolution();
    if (resolution <= 0.0) {
      return false;
    }

    const int nx = static_cast<int>(std::ceil(2.0 * local_radius_xy_ / resolution));
    const int ny = static_cast<int>(std::ceil(2.0 * local_radius_xy_ / resolution));
    const int nz = static_cast<int>(std::ceil(2.0 * local_radius_z_ / resolution));
    if (nx <= 0 || ny <= 0 || nz <= 0) {
      return false;
    }

    grid.resolution = resolution;
    grid.nx = nx;
    grid.ny = ny;
    grid.nz = nz;
    grid.origin = center - Eigen::Vector3d(local_radius_xy_, local_radius_xy_, local_radius_z_);

    const size_t total = static_cast<size_t>(nx) * ny * nz;
    grid.base_state.assign(total, kUnknown);
    grid.plan_state.assign(total, kUnknown);

    for (int ix = 0; ix < nx; ++ix) {
      for (int iy = 0; iy < ny; ++iy) {
        for (int iz = 0; iz < nz; ++iz) {
          const Eigen::Vector3d pos = grid.cellCenter(ix, iy, iz);
          octomap::OcTreeNode *node = tree.search(pos.x(), pos.y(), pos.z());
          uint8_t state = kUnknown;
          if (node) {
            state = tree.isNodeOccupied(node) ? kOccupied : kFree;
          }
          const int idx = grid.index(ix, iy, iz);
          grid.base_state[idx] = state;
          grid.plan_state[idx] = state;
        }
      }
    }

    inflateObstacles(grid);
    return true;
  }

  void inflateObstacles(Grid3D &grid)
  {
    const int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / grid.resolution));
    if (inflation_cells <= 0) {
      for (size_t i = 0; i < grid.plan_state.size(); ++i) {
        if (grid.plan_state[i] == kOccupied) {
          grid.plan_state[i] = kBlocked;
        }
      }
      return;
    }

    std::vector<Eigen::Vector3i> offsets;
    offsets.reserve((2 * inflation_cells + 1) * (2 * inflation_cells + 1) * (2 * inflation_cells + 1));
    const double max_dist = inflation_radius_ + 1e-6;
    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
      for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
        for (int dz = -inflation_cells; dz <= inflation_cells; ++dz) {
          const double dist = std::sqrt(
            static_cast<double>(dx * dx + dy * dy + dz * dz)) * grid.resolution;
          if (dist <= max_dist) {
            offsets.emplace_back(dx, dy, dz);
          }
        }
      }
    }

    std::vector<uint8_t> inflated = grid.plan_state;
    for (int ix = 0; ix < grid.nx; ++ix) {
      for (int iy = 0; iy < grid.ny; ++iy) {
        for (int iz = 0; iz < grid.nz; ++iz) {
          const int idx = grid.index(ix, iy, iz);
          if (grid.plan_state[idx] != kOccupied) {
            continue;
          }
          for (const auto &off : offsets) {
            const int nx = ix + off.x();
            const int ny = iy + off.y();
            const int nz = iz + off.z();
            if (!grid.valid(nx, ny, nz)) {
              continue;
            }
            const int nidx = grid.index(nx, ny, nz);
            if (inflated[nidx] == kFree) {
              inflated[nidx] = kBlocked;
            }
            if (inflated[nidx] == kOccupied) {
              inflated[nidx] = kBlocked;
            }
          }
        }
      }
    }

    grid.plan_state.swap(inflated);
  }

  std::vector<FrontierCluster> extractFrontiers(const Grid3D &grid)
  {
    const int total = grid.nx * grid.ny * grid.nz;
    std::vector<uint8_t> visited(static_cast<size_t>(total), 0);
    std::vector<FrontierCluster> clusters;

    auto isFrontier = [&grid, this](int ix, int iy, int iz) {
      const int idx = grid.index(ix, iy, iz);
      if (grid.base_state[idx] != kFree) {
        return false;
      }
      static const int dx[6] = {1, -1, 0, 0, 0, 0};
      static const int dy[6] = {0, 0, 1, -1, 0, 0};
      static const int dz[6] = {0, 0, 0, 0, 1, -1};
      for (int k = 0; k < 6; ++k) {
        const int nx = ix + dx[k];
        const int ny = iy + dy[k];
        const int nz = iz + dz[k];
        if (!grid.valid(nx, ny, nz)) {
          if (outside_unknown_as_frontier_) {
            return true;
          }
          continue;
        }
        const int nidx = grid.index(nx, ny, nz);
        if (grid.base_state[nidx] == kUnknown) {
          return true;
        }
      }
      return false;
    };

    for (int ix = 0; ix < grid.nx; ++ix) {
      for (int iy = 0; iy < grid.ny; ++iy) {
        for (int iz = 0; iz < grid.nz; ++iz) {
          const int idx = grid.index(ix, iy, iz);
          if (visited[idx]) {
            continue;
          }
          if (!isFrontier(ix, iy, iz)) {
            continue;
          }

          FrontierCluster cluster;
          std::queue<Eigen::Vector3i> q;
          q.emplace(ix, iy, iz);
          visited[idx] = 1;

          while (!q.empty()) {
            const auto cell = q.front();
            q.pop();
            const int cidx = grid.index(cell.x(), cell.y(), cell.z());
            cluster.indices.push_back(cidx);
            cluster.centroid += grid.cellCenter(cell.x(), cell.y(), cell.z());

            for (int dx = -1; dx <= 1; ++dx) {
              for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                  if (dx == 0 && dy == 0 && dz == 0) {
                    continue;
                  }
                  const int nx = cell.x() + dx;
                  const int ny = cell.y() + dy;
                  const int nz = cell.z() + dz;
                  if (!grid.valid(nx, ny, nz)) {
                    continue;
                  }
                  const int nidx = grid.index(nx, ny, nz);
                  if (visited[nidx]) {
                    continue;
                  }
                  if (!isFrontier(nx, ny, nz)) {
                    continue;
                  }
                  visited[nidx] = 1;
                  q.emplace(nx, ny, nz);
                }
              }
            }
          }

          if (static_cast<int>(cluster.indices.size()) >= frontier_min_cluster_size_) {
            cluster.centroid /= static_cast<double>(cluster.indices.size());
            clusters.push_back(cluster);
          }
        }
      }
    }

    return clusters;
  }

  double computeClusterGain(const Grid3D &grid, const FrontierCluster &cluster) const
  {
    double gain = 0.0;
    if (info_gain_radius_ > 0.0) {
      gain += info_gain_unknown_weight_ *
        static_cast<double>(countUnknownInRadius(grid, cluster.centroid, info_gain_radius_));
    }
    if (cluster_size_weight_ > 0.0) {
      gain += cluster_size_weight_ * static_cast<double>(cluster.indices.size());
    }
    return gain;
  }

  int countUnknownInRadius(const Grid3D &grid, const Eigen::Vector3d &center, double radius) const
  {
    if (radius <= 0.0) {
      return 0;
    }
    Eigen::Vector3i center_idx;
    if (!worldToGrid(grid, center, center_idx)) {
      return 0;
    }
    const int r = static_cast<int>(std::ceil(radius / grid.resolution));
    const double r_sq = radius * radius;
    int count = 0;
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        for (int dz = -r; dz <= r; ++dz) {
          const double dist_sq = static_cast<double>(dx * dx + dy * dy + dz * dz) *
                                 grid.resolution * grid.resolution;
          if (dist_sq > r_sq) {
            continue;
          }
          const int nx = center_idx.x() + dx;
          const int ny = center_idx.y() + dy;
          const int nz = center_idx.z() + dz;
          if (!grid.valid(nx, ny, nz)) {
            if (outside_unknown_as_gain_) {
              count += 1;
            }
            continue;
          }
          const int nidx = grid.index(nx, ny, nz);
          if (grid.base_state[nidx] == kUnknown) {
            count += 1;
          }
        }
      }
    }
    return count;
  }

  FrontierCluster selectBestFrontier(const std::vector<FrontierCluster> &clusters,
                                     const Grid3D &grid,
                                     const Eigen::Vector3d &current_pos)
  {
    FrontierCluster best;
    FrontierCluster best_fallback;
    double best_score_any = -std::numeric_limits<double>::infinity();
    double best_score_allowed = -std::numeric_limits<double>::infinity();
    bool found = false;

    for (const auto &cluster : clusters) {
      const double gain = computeClusterGain(grid, cluster);
      const double dist = (cluster.centroid - current_pos).norm();
      const double forward_score = forward_bias_weight_ * forwardDot(cluster.centroid, current_pos);
      const double lateral_penalty = lateral_bias_weight_ * lateralDistance(cluster.centroid, current_pos);
      const double score = frontier_gain_weight_ * gain - frontier_dist_weight_ * dist + forward_score - lateral_penalty;
      if (score > best_score_any) {
        best_score_any = score;
        best_fallback = cluster;
      }
      if (isNearRecentGoal(cluster.centroid) || forwardDot(cluster.centroid, current_pos) < min_forward_cos_) {
        continue;
      }
      if (!found || score > best_score_allowed) {
        best_score_allowed = score;
        best = cluster;
        found = true;
      }
    }

    return found ? best : best_fallback;
  }

  void updateGlobalFrontiers(const std::vector<FrontierCluster> &clusters,
                             const Grid3D &grid,
                             const rclcpp::Time &now)
  {
    for (const auto &cluster : clusters) {
      const Eigen::Vector3d pos = cluster.centroid;
      const double gain = computeClusterGain(grid, cluster);
      int match_index = findFrontierNear(pos, global_frontier_merge_radius_);
      if (match_index >= 0) {
        auto &frontier = global_frontiers_[static_cast<size_t>(match_index)];
        const double total = static_cast<double>(frontier.observations + 1);
        frontier.position = (frontier.position * frontier.observations + pos) / total;
        frontier.gain = std::max(frontier.gain, gain);
        frontier.observations += 1;
        frontier.last_seen = now;
      } else {
        StoredFrontier frontier;
        frontier.position = pos;
        frontier.gain = gain;
        frontier.observations = 1;
        frontier.fail_count = 0;
        frontier.visited = false;
        frontier.last_seen = now;
        global_frontiers_.push_back(frontier);
      }
    }
  }

  void pruneGlobalFrontiers(const rclcpp::Time &now)
  {
    if (global_frontiers_.empty()) {
      return;
    }
    auto keep = [&](const StoredFrontier &f) {
      if (f.fail_count >= max_goal_failures_) {
        return false;
      }
      if (f.visited) {
        return false;
      }
      if (global_frontier_ttl_sec_ > 0.0) {
        const double age = (now - f.last_seen).seconds();
        if (age > global_frontier_ttl_sec_) {
          return false;
        }
      }
      return true;
    };

    std::vector<StoredFrontier> filtered;
    filtered.reserve(global_frontiers_.size());
    for (const auto &f : global_frontiers_) {
      if (keep(f)) {
        filtered.push_back(f);
      }
    }
    global_frontiers_.swap(filtered);
  }

  int findFrontierNear(const Eigen::Vector3d &pos, double radius) const
  {
    if (global_frontiers_.empty()) {
      return -1;
    }
    const double radius_sq = radius * radius;
    int best_idx = -1;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < global_frontiers_.size(); ++i) {
      const double dist_sq = (global_frontiers_[i].position - pos).squaredNorm();
      if (dist_sq <= radius_sq && dist_sq < best_dist) {
        best_dist = dist_sq;
        best_idx = static_cast<int>(i);
      }
    }
    return best_idx;
  }

  int selectFrontierIndex(const Eigen::Vector3d &current_pos) const
  {
    if (global_frontiers_.empty()) {
      return -1;
    }

    int best_idx = -1;
    int fallback_idx = -1;
    double best_score_allowed = -std::numeric_limits<double>::infinity();
    double best_score_any = -std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < global_frontiers_.size(); ++i) {
      const auto &frontier = global_frontiers_[i];
      if (frontier.visited || frontier.fail_count >= max_goal_failures_) {
        continue;
      }
      const double dist = (frontier.position - current_pos).norm();
      const double forward_score = forward_bias_weight_ * forwardDot(frontier.position, current_pos);
      const double lateral_penalty = lateral_bias_weight_ * lateralDistance(frontier.position, current_pos);
      const double score = frontier_gain_weight_ * frontier.gain - frontier_dist_weight_ * dist + forward_score - lateral_penalty;
      if (score > best_score_any) {
        best_score_any = score;
        fallback_idx = static_cast<int>(i);
      }
      if (isNearRecentGoal(frontier.position) || forwardDot(frontier.position, current_pos) < min_forward_cos_) {
        continue;
      }
      if (score > best_score_allowed) {
        best_score_allowed = score;
        best_idx = static_cast<int>(i);
      }
    }

    return (best_idx >= 0) ? best_idx : fallback_idx;
  }

  bool isGoalReached(const Eigen::Vector3d &current_pos) const
  {
    return (current_pos - active_goal_).norm() <= goal_reached_distance_;
  }

  double forwardDot(const Eigen::Vector3d &target, const Eigen::Vector3d &current) const
  {
    const Eigen::Vector3d dir = (target - current);
    const double norm = dir.head<2>().norm();
    if (norm < 1e-6) {
      return 0.0;
    }
    const Eigen::Vector3d dir_xy(dir.x() / norm, dir.y() / norm, 0.0);
    return forward_dir_.dot(dir_xy);
  }

  double lateralDistance(const Eigen::Vector3d &target, const Eigen::Vector3d &current) const
  {
    const Eigen::Vector3d dir = (target - current);
    const double norm = dir.head<2>().norm();
    if (norm < 1e-6) {
      return 0.0;
    }
    const Eigen::Vector3d dir_xy(dir.x() / norm, dir.y() / norm, 0.0);
    const double forward = std::max(-1.0, std::min(1.0, forward_dir_.dot(dir_xy)));
    const double sin_angle = std::sqrt(std::max(0.0, 1.0 - forward * forward));
    return norm * sin_angle;
  }

  bool shouldAttemptEscape() const
  {
    return escape_fail_threshold_ > 0 && consecutive_plan_failures_ >= escape_fail_threshold_;
  }

  bool publishEscape(const Grid3D &grid,
                     const Eigen::Vector3d &current_pos,
                     const Eigen::Vector3d &current_vel)
  {
    const Eigen::Vector3d retreat = current_pos - forward_dir_ * escape_backoff_ +
                                    Eigen::Vector3d(0.0, 0.0, escape_up_);
    const Eigen::Vector3d target = projectGoalToGrid(current_pos, retreat, grid);
    if ((target - current_pos).norm() < min_goal_distance_) {
      return false;
    }

    std::vector<Eigen::Vector3d> waypoints;
    waypoints.push_back(current_pos);
    waypoints.push_back(target);

    double fallback_yaw = last_yaw_;
    const double vel_xy = std::hypot(current_vel.x(), current_vel.y());
    if (vel_xy > yaw_speed_threshold_) {
      fallback_yaw = std::atan2(current_vel.y(), current_vel.x());
    }

    mav_trajectory_generation::Trajectory trajectory;
    if (!generateTrajectory(current_pos, current_vel, waypoints, fallback_yaw, &trajectory)) {
      return false;
    }

    publishTrajectory(trajectory);
    have_active_goal_ = false;
    active_goal_index_ = -1;
    consecutive_plan_failures_ = 0;
    RCLCPP_WARN(get_logger(), "Escape trajectory published.");
    return true;
  }

  void markFrontierVisited(int index)
  {
    if (index < 0 || static_cast<size_t>(index) >= global_frontiers_.size()) {
      return;
    }
    global_frontiers_[static_cast<size_t>(index)].visited = true;
  }

  void markFrontierFailed(int index)
  {
    if (index < 0 || static_cast<size_t>(index) >= global_frontiers_.size()) {
      return;
    }
    global_frontiers_[static_cast<size_t>(index)].fail_count += 1;
  }

  Eigen::Vector3d projectGoalToGrid(const Eigen::Vector3d &current_pos,
                                    const Eigen::Vector3d &target,
                                    const Grid3D &grid) const
  {
    const Eigen::Vector3d min_bound = grid.origin;
    const Eigen::Vector3d max_bound = grid.origin + Eigen::Vector3d(
      grid.nx * grid.resolution,
      grid.ny * grid.resolution,
      grid.nz * grid.resolution);

    auto inside = [&](const Eigen::Vector3d &p) {
      return (p.x() >= min_bound.x() && p.y() >= min_bound.y() && p.z() >= min_bound.z() &&
              p.x() <= max_bound.x() && p.y() <= max_bound.y() && p.z() <= max_bound.z());
    };

    Eigen::Vector3d clamped = target;
    if (!inside(target)) {
      const Eigen::Vector3d dir = target - current_pos;
      const double dir_norm = dir.norm();
      if (dir_norm < 1e-6) {
        return current_pos;
      }

      double t_min = std::numeric_limits<double>::infinity();
      for (int axis = 0; axis < 3; ++axis) {
        const double d = dir[axis];
        if (std::abs(d) < 1e-9) {
          continue;
        }
        const double bound = (d > 0.0) ? max_bound[axis] : min_bound[axis];
        const double t = (bound - current_pos[axis]) / d;
        if (t > 0.0 && t < t_min) {
          t_min = t;
        }
      }

      if (!std::isfinite(t_min)) {
        return current_pos;
      }

      const double t_margin = subgoal_margin_ / dir_norm;
      const double t = std::max(0.0, t_min - t_margin);
      clamped = current_pos + dir * t;
      clamped.x() = std::min(std::max(clamped.x(), min_bound.x()), max_bound.x());
      clamped.y() = std::min(std::max(clamped.y(), min_bound.y()), max_bound.y());
      clamped.z() = std::min(std::max(clamped.z(), min_bound.z()), max_bound.z());
    }

    Eigen::Vector3i goal_idx;
    if (worldToGrid(grid, clamped, goal_idx)) {
      const int gindex = grid.index(goal_idx.x(), goal_idx.y(), goal_idx.z());
      if (grid.plan_state[gindex] == kFree) {
        return clamped;
      }
    }

    const Eigen::Vector3d dir = clamped - current_pos;
    const double dir_norm = dir.norm();
    if (dir_norm < 1e-6) {
      return current_pos;
    }
    const Eigen::Vector3d dir_unit = dir / dir_norm;
    const double step = std::max(0.5 * grid.resolution, 0.1);

    Eigen::Vector3d last_free = current_pos;
    bool found = false;
    for (double t = 0.0; t <= dir_norm; t += step) {
      const Eigen::Vector3d p = current_pos + dir_unit * t;
      Eigen::Vector3i idx;
      if (!worldToGrid(grid, p, idx)) {
        break;
      }
      const int index = grid.index(idx.x(), idx.y(), idx.z());
      if (grid.plan_state[index] == kFree) {
        last_free = grid.cellCenter(idx.x(), idx.y(), idx.z());
        found = true;
        continue;
      }
      break;
    }

    if (!found) {
      return current_pos;
    }

    const Eigen::Vector3d backed = last_free - dir_unit * goal_clearance_;
    Eigen::Vector3i back_idx;
    if (worldToGrid(grid, backed, back_idx)) {
      const int bindex = grid.index(back_idx.x(), back_idx.y(), back_idx.z());
      if (grid.plan_state[bindex] == kFree) {
        return grid.cellCenter(back_idx.x(), back_idx.y(), back_idx.z());
      }
    }

    return last_free;
  }

  bool isNearRecentGoal(const Eigen::Vector3d &goal) const
  {
    if (goal_history_size_ <= 0 || goal_history_.empty()) {
      return false;
    }
    for (const auto &prev : goal_history_) {
      if ((goal - prev).norm() <= goal_history_radius_) {
        return true;
      }
    }
    return false;
  }

  void rememberGoal(const Eigen::Vector3d &goal)
  {
    if (goal_history_size_ <= 0) {
      return;
    }
    goal_history_.push_back(goal);
    while (static_cast<int>(goal_history_.size()) > goal_history_size_) {
      goal_history_.pop_front();
    }
  }

  bool planPath(const Grid3D &grid, const Eigen::Vector3d &start, const Eigen::Vector3d &goal,
                std::vector<Eigen::Vector3d> &path_out)
  {
    Eigen::Vector3i start_idx;
    Eigen::Vector3i goal_idx;
    if (!worldToGrid(grid, start, start_idx) || !worldToGrid(grid, goal, goal_idx)) {
      return false;
    }

    if (!ensureFreeCell(grid, start_idx)) {
      return false;
    }
    if (!ensureFreeCell(grid, goal_idx)) {
      return false;
    }

    const int total = grid.nx * grid.ny * grid.nz;
    std::vector<double> g_score(static_cast<size_t>(total), std::numeric_limits<double>::infinity());
    std::vector<int> came_from(static_cast<size_t>(total), -1);
    std::priority_queue<AStarNode, std::vector<AStarNode>, AStarCompare> open;

    const int start_index = grid.index(start_idx.x(), start_idx.y(), start_idx.z());
    const int goal_index = grid.index(goal_idx.x(), goal_idx.y(), goal_idx.z());

    g_score[start_index] = 0.0;
    open.push({start_index, heuristic(grid, start_idx, goal_idx)});

    std::vector<Eigen::Vector3i> neighbor_offsets;
    neighbor_offsets.reserve(26);
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          neighbor_offsets.emplace_back(dx, dy, dz);
        }
      }
    }

    std::vector<uint8_t> closed(static_cast<size_t>(total), 0);

    while (!open.empty()) {
      const int current = open.top().idx;
      open.pop();

      if (closed[current]) {
        continue;
      }
      closed[current] = 1;

      if (current == goal_index) {
        reconstructPath(grid, came_from, current, path_out);
        return true;
      }

      Eigen::Vector3i cidx;
      indexToGrid(grid, current, cidx);

      for (const auto &off : neighbor_offsets) {
        const int nx = cidx.x() + off.x();
        const int ny = cidx.y() + off.y();
        const int nz = cidx.z() + off.z();
        if (!grid.valid(nx, ny, nz)) {
          continue;
        }
        const int nindex = grid.index(nx, ny, nz);
        if (closed[nindex]) {
          continue;
        }
        const uint8_t plan_state = grid.plan_state[nindex];
        const uint8_t base_state = grid.base_state[nindex];
        const bool traversable = (plan_state == kFree) ||
          (allow_unknown_traversal_ && plan_state == kUnknown && base_state == kUnknown);
        if (!traversable) {
          continue;
        }

        double step_cost = grid.resolution * std::sqrt(
          static_cast<double>(off.x() * off.x() + off.y() * off.y() + off.z() * off.z()));
        if (plan_state == kUnknown && base_state == kUnknown) {
          step_cost *= unknown_traversal_penalty_;
        }
        const double tentative_g = g_score[current] + step_cost;

        if (tentative_g < g_score[nindex]) {
          g_score[nindex] = tentative_g;
          came_from[nindex] = current;
          const double f = tentative_g + heuristic(grid, Eigen::Vector3i(nx, ny, nz), goal_idx);
          open.push({nindex, f});
        }
      }
    }

    return false;
  }

  bool worldToGrid(const Grid3D &grid, const Eigen::Vector3d &pos, Eigen::Vector3i &idx) const
  {
    const Eigen::Vector3d rel = pos - grid.origin;
    idx.x() = static_cast<int>(std::floor(rel.x() / grid.resolution));
    idx.y() = static_cast<int>(std::floor(rel.y() / grid.resolution));
    idx.z() = static_cast<int>(std::floor(rel.z() / grid.resolution));
    return grid.valid(idx.x(), idx.y(), idx.z());
  }

  void indexToGrid(const Grid3D &grid, int index, Eigen::Vector3i &idx) const
  {
    const int plane = grid.nx * grid.ny;
    idx.z() = index / plane;
    const int rem = index % plane;
    idx.y() = rem / grid.nx;
    idx.x() = rem % grid.nx;
  }

  bool ensureFreeCell(const Grid3D &grid, Eigen::Vector3i &idx)
  {
    const int index = grid.index(idx.x(), idx.y(), idx.z());
    if (grid.plan_state[index] == kFree) {
      return true;
    }

    const int max_radius = 3;
    for (int r = 1; r <= max_radius; ++r) {
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy = -r; dy <= r; ++dy) {
          for (int dz = -r; dz <= r; ++dz) {
            const int nx = idx.x() + dx;
            const int ny = idx.y() + dy;
            const int nz = idx.z() + dz;
            if (!grid.valid(nx, ny, nz)) {
              continue;
            }
            const int nindex = grid.index(nx, ny, nz);
            if (grid.plan_state[nindex] == kFree) {
              idx = Eigen::Vector3i(nx, ny, nz);
              return true;
            }
          }
        }
      }
    }

    return false;
  }

  double heuristic(const Grid3D &grid, const Eigen::Vector3i &a, const Eigen::Vector3i &b) const
  {
    const double dx = static_cast<double>(a.x() - b.x());
    const double dy = static_cast<double>(a.y() - b.y());
    const double dz = static_cast<double>(a.z() - b.z());
    return grid.resolution * std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  void reconstructPath(const Grid3D &grid, const std::vector<int> &came_from,
                       int current, std::vector<Eigen::Vector3d> &path_out)
  {
    path_out.clear();
    int idx = current;
    while (idx >= 0) {
      Eigen::Vector3i grid_idx;
      indexToGrid(grid, idx, grid_idx);
      path_out.push_back(grid.cellCenter(grid_idx.x(), grid_idx.y(), grid_idx.z()));
      idx = came_from[idx];
    }
    std::reverse(path_out.begin(), path_out.end());
  }

  std::vector<Eigen::Vector3d> downsamplePath(const std::vector<Eigen::Vector3d> &path,
                                              double spacing) const
  {
    if (path.empty()) {
      return {};
    }

    std::vector<Eigen::Vector3d> result;
    result.push_back(path.front());

    double accum = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      accum += (path[i] - path[i - 1]).norm();
      if (accum >= spacing) {
        if ((path[i] - result.back()).norm() > 1e-3) {
          result.push_back(path[i]);
        }
        accum = 0.0;
      }
    }

    if ((result.back() - path.back()).norm() > 0.5 * spacing) {
      result.push_back(path.back());
    }

    if (result.size() < 2 && path.size() >= 2) {
      result = {path.front(), path.back()};
    }

    return result;
  }

  bool generateTrajectory(const Eigen::Vector3d &start_pos,
                          const Eigen::Vector3d &start_vel,
                          const std::vector<Eigen::Vector3d> &waypoints_in,
                          double fallback_yaw,
                          mav_trajectory_generation::Trajectory *trajectory)
  {
    std::vector<Eigen::Vector3d> waypoints = waypoints_in;
    if (waypoints.size() < 2) {
      return false;
    }
    if (waypoints.size() == 2) {
      const Eigen::Vector3d mid = 0.5 * (waypoints.front() + waypoints.back());
      waypoints.insert(waypoints.begin() + 1, mid);
    }

    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    const int dimension = use_yaw_ ? 4 : 3;
    mav_trajectory_generation::Vertex::Vector vertices;

    if (use_yaw_) {
      const std::vector<double> yaw_values = computeYawFromPath(waypoints, fallback_yaw);

      mav_trajectory_generation::Vertex start(dimension);
      Eigen::Vector4d start_pos_yaw;
      start_pos_yaw << start_pos.x(), start_pos.y(), start_pos.z(), yaw_values.front();
      start.makeStartOrEnd(start_pos_yaw, derivative_to_optimize);
      Eigen::Vector4d start_vel_yaw;
      start_vel_yaw << start_vel.x(), start_vel.y(), start_vel.z(), 0.0;
      start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_vel_yaw);
      vertices.push_back(start);

      for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
        mav_trajectory_generation::Vertex middle(dimension);
        Eigen::Vector4d pos_yaw;
        pos_yaw << waypoints[i].x(), waypoints[i].y(), waypoints[i].z(), yaw_values[i];
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, pos_yaw);
        vertices.push_back(middle);
      }

      mav_trajectory_generation::Vertex end(dimension);
      Eigen::Vector4d end_pos_yaw;
      end_pos_yaw << waypoints.back().x(), waypoints.back().y(), waypoints.back().z(), yaw_values.back();
      end.makeStartOrEnd(end_pos_yaw, derivative_to_optimize);
      end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector4d::Zero());
      vertices.push_back(end);
    } else {
      mav_trajectory_generation::Vertex start(dimension);
      start.makeStartOrEnd(start_pos, derivative_to_optimize);
      start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, start_vel);
      vertices.push_back(start);

      for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
        mav_trajectory_generation::Vertex middle(dimension);
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, waypoints[i]);
        vertices.push_back(middle);
      }

      mav_trajectory_generation::Vertex end(dimension);
      end.makeStartOrEnd(waypoints.back(), derivative_to_optimize);
      end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d::Zero());
      vertices.push_back(end);
    }

    std::vector<double> segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);
    if (segment_times.size() + 1 != vertices.size()) {
      RCLCPP_WARN(get_logger(), "Invalid segment times (%zu) for %zu vertices.",
                  segment_times.size(), vertices.size());
      return false;
    }

    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    const int N = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);

    opt.optimize();
    opt.getTrajectory(trajectory);
    return true;
  }

  std::vector<double> computeYawFromPath(const std::vector<Eigen::Vector3d> &waypoints,
                                         double fallback_yaw) const
  {
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
        last_yaw = std::atan2(dy, dx);
      }
      yaw_values[i] = last_yaw;
    }
    yaw_values.back() = yaw_values[n - 2];

    for (size_t i = 1; i < n; ++i) {
      double delta = yaw_values[i] - yaw_values[i - 1];
      while (delta > kPi) {
        yaw_values[i] -= 2.0 * kPi;
        delta = yaw_values[i] - yaw_values[i - 1];
      }
      while (delta < -kPi) {
        yaw_values[i] += 2.0 * kPi;
        delta = yaw_values[i] - yaw_values[i - 1];
      }
    }

    if (std::abs(yaw_offset_) > 1e-6) {
      for (auto &yaw : yaw_values) {
        yaw += yaw_offset_;
      }
    }

    return yaw_values;
  }

  void requestStopSampling()
  {
    if (!stop_sampling_client_) {
      return;
    }
    if (!stop_sampling_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(get_logger(), "stop_sampling service not available.");
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    (void)stop_sampling_client_->async_send_request(request);
  }

  void publishTrajectory(const mav_trajectory_generation::Trajectory &trajectory)
  {
    mav_planning_msgs::msg::PolynomialTrajectory4D msg;
    if (!mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg)) {
      RCLCPP_WARN(get_logger(), "Failed to convert trajectory to message.");
      return;
    }
    msg.header.frame_id = frame_id_;
    msg.header.stamp = this->now();
    traj_pub_->publish(msg);
  }

  void publishMarkers(const std::vector<Eigen::Vector3d> &path,
                      const std::vector<FrontierCluster> &clusters,
                      const Eigen::Vector3d &goal)
  {
    if (!publish_markers_ || !marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = frame_id_;
    path_marker.header.stamp = this->now();
    path_marker.ns = "exploration_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;
    path_marker.scale.x = 0.05;
    path_marker.color.r = 0.1f;
    path_marker.color.g = 0.8f;
    path_marker.color.b = 0.1f;
    path_marker.color.a = 1.0f;

    for (const auto &pt : path) {
      geometry_msgs::msg::Point p;
      p.x = pt.x();
      p.y = pt.y();
      p.z = pt.z();
      path_marker.points.push_back(p);
    }
    marker_array.markers.push_back(path_marker);

    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = frame_id_;
    goal_marker.header.stamp = this->now();
    goal_marker.ns = "exploration_goal";
    goal_marker.id = 1;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.scale.x = 0.5;
    goal_marker.scale.y = 0.5;
    goal_marker.scale.z = 0.5;
    goal_marker.color.r = 0.9f;
    goal_marker.color.g = 0.2f;
    goal_marker.color.b = 0.2f;
    goal_marker.color.a = 1.0f;
    goal_marker.pose.position.x = goal.x();
    goal_marker.pose.position.y = goal.y();
    goal_marker.pose.position.z = goal.z();
    marker_array.markers.push_back(goal_marker);

    int id = 2;
    for (const auto &cluster : clusters) {
      visualization_msgs::msg::Marker cluster_marker;
      cluster_marker.header.frame_id = frame_id_;
      cluster_marker.header.stamp = this->now();
      cluster_marker.ns = "exploration_frontiers";
      cluster_marker.id = id++;
      cluster_marker.type = visualization_msgs::msg::Marker::SPHERE;
      cluster_marker.action = visualization_msgs::msg::Marker::ADD;
      cluster_marker.scale.x = 0.3;
      cluster_marker.scale.y = 0.3;
      cluster_marker.scale.z = 0.3;
      cluster_marker.color.r = 0.2f;
      cluster_marker.color.g = 0.2f;
      cluster_marker.color.b = 0.9f;
      cluster_marker.color.a = 0.7f;
      cluster_marker.pose.position.x = cluster.centroid.x();
      cluster_marker.pose.position.y = cluster.centroid.y();
      cluster_marker.pose.position.z = cluster.centroid.z();
      marker_array.markers.push_back(cluster_marker);
    }

    marker_pub_->publish(marker_array);
  }

  std::string octomap_topic_;
  std::string odom_topic_;
  std::string trajectory_topic_;
  std::string frame_id_;

  double local_radius_xy_;
  double local_radius_z_;
  double grid_resolution_;
  double inflation_radius_;
  int frontier_min_cluster_size_;
  double frontier_gain_weight_;
  double frontier_dist_weight_;
  double info_gain_radius_;
  double info_gain_unknown_weight_;
  double cluster_size_weight_;
  bool outside_unknown_as_frontier_;
  bool outside_unknown_as_gain_;
  bool allow_unknown_traversal_;
  double unknown_traversal_penalty_;
  double forward_bias_weight_;
  double min_forward_cos_;
  double lateral_bias_weight_;
  double forward_speed_threshold_;
  bool use_velocity_heading_;
  bool invert_forward_dir_;
  int escape_fail_threshold_;
  double escape_backoff_;
  double escape_up_;
  int consecutive_plan_failures_ = 0;
  double goal_clearance_;
  double waypoint_spacing_;
  double max_v_;
  double max_a_;
  bool use_yaw_;
  double yaw_speed_threshold_;
  double yaw_offset_;
  double replan_period_sec_;

  std::string lantern_topic_;
  int required_lanterns_;
  bool stop_on_lanterns_;

  int goal_history_size_;
  double goal_history_radius_;
  double global_frontier_merge_radius_;
  double global_frontier_ttl_sec_;
  double goal_reached_distance_;
  int max_goal_failures_;
  double subgoal_margin_;
  double min_goal_distance_;
  double min_path_length_;

  bool start_immediately_;
  double start_distance_;
  double start_speed_;
  std::vector<double> entrance_position_;
  bool start_on_waypoint_done_;
  std::string waypoint_done_topic_;

  bool publish_markers_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waypoint_done_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lantern_sub_;
  rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_sampling_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry last_odom_;
  bool have_odom_ = false;
  bool have_map_ = false;
  bool exploration_started_ = false;
  bool exploration_done_ = false;
  bool done_stop_requested_ = false;
  int last_lantern_count_ = 0;
  bool waypoint_done_ = false;
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  double last_yaw_ = 0.0;
  Eigen::Vector3d forward_dir_ = Eigen::Vector3d::UnitX();
  bool have_active_goal_ = false;
  int active_goal_index_ = -1;
  Eigen::Vector3d active_goal_ = Eigen::Vector3d::Zero();

  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex map_mutex_;

  std::deque<Eigen::Vector3d> goal_history_;
  std::vector<StoredFrontier> global_frontiers_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationNode>());
  rclcpp::shutdown();
  return 0;
}
