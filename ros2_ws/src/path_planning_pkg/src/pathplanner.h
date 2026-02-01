#pragma once

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <mav_trajectory_generation/trajectory.h>

#include <Eigen/Dense>

#include <octomap/octomap.h>

#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode();

private:
  struct KeyHash {
    std::size_t operator()(const octomap::OcTreeKey &key) const noexcept;
  };
  struct KeyEq {
    bool operator()(const octomap::OcTreeKey &a,
                    const octomap::OcTreeKey &b) const noexcept;
  };

  struct FrontierCluster {
    std::vector<octomap::OcTreeKey> cells;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  };

  void onCommand(const std_msgs::msg::String::SharedPtr msg);
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onWaypointDone(const std_msgs::msg::Bool::SharedPtr msg);
  void tick();

  bool computeFrontierClusters(std::vector<FrontierCluster> *clusters);
  bool selectFrontierGoal(const std::vector<FrontierCluster> &clusters,
                          octomap::OcTreeKey *goal_key);
  bool planPathAStar(const octomap::OcTreeKey &start_key,
                     const octomap::OcTreeKey &goal_key,
                     std::vector<octomap::OcTreeKey> *path_out);
  bool buildTrajectoryFromPath(const std::vector<Eigen::Vector3d> &path,
                               mav_trajectory_generation::Trajectory *trajectory);
  bool publishTrajectory(const mav_trajectory_generation::Trajectory &trajectory);
  void publishVisualization(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<FrontierCluster> &clusters,
                            const octomap::OcTreeKey *goal_key);

  std::vector<double> computeYawFromPath(
      const std::vector<Eigen::Vector3d> &waypoints,
      double fallback_yaw) const;

  bool isFreeCoord(const octomap::point3d &coord) const;
  bool isFreeKey(const octomap::OcTreeKey &key) const;
  bool isFrontierKey(const octomap::OcTreeKey &key) const;
  bool findNearestFreeKey(const octomap::OcTreeKey &seed_key,
                          octomap::OcTreeKey *free_key) const;
  octomap::point3d keyToCoord(const octomap::OcTreeKey &key) const;

  double forwardDot(const Eigen::Vector3d &target) const;
  double lateralDistance(const Eigen::Vector3d &target) const;

  std::vector<Eigen::Vector3d> simplifyPath(
      const std::vector<octomap::OcTreeKey> &path_keys) const;
  bool isSegmentFree(const Eigen::Vector3d &start,
                     const Eigen::Vector3d &goal) const;
  std::vector<Eigen::Vector3d> prunePathLineOfSight(
      const std::vector<Eigen::Vector3d> &path) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr waypoint_done_sub_;

  rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr
      trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      markers_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  mutable std::mutex map_mutex_;
  std::shared_ptr<octomap::OcTree> map_;
  bool have_map_ = false;
  octomap::point3d map_min_ = octomap::point3d(0.0, 0.0, 0.0);
  octomap::point3d map_max_ = octomap::point3d(0.0, 0.0, 0.0);
  bool have_bounds_ = false;

  nav_msgs::msg::Odometry current_odom_;
  bool have_odom_ = false;
  Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d forward_dir_ = Eigen::Vector3d::UnitX();
  double last_yaw_ = 0.0;

  bool exploring_ = false;
  bool force_replan_ = false;
  bool has_goal_ = false;
  Eigen::Vector3d goal_position_ = Eigen::Vector3d::Zero();
  bool waypoint_done_ = false;

  rclcpp::Time last_plan_time_;
  rclcpp::Time last_map_stamp_;

  // Parameters
  std::string command_topic_;
  std::string octomap_topic_;
  std::string odom_topic_;
  std::string trajectory_topic_;
  std::string markers_topic_;
  std::string world_frame_;
  std::string waypoint_done_topic_;

  bool auto_start_ = true;
  bool visualize_ = true;
  bool replan_while_moving_ = false;
  bool start_on_waypoint_done_ = false;

  double tick_rate_hz_ = 1.0;
  double max_v_ = 1.5;
  double max_a_ = 1.0;
  double goal_reached_tolerance_ = 0.5;
  double replan_interval_s_ = 2.0;
  double frontier_max_distance_ = 30.0;
  double min_goal_distance_ = 1.0;
  double frontier_score_distance_weight_ = 1.0;
  double frontier_score_size_weight_ = 1.0;
  double forward_bias_weight_ = 0.0;
  double min_forward_cos_ = -1.0;
  double lateral_bias_weight_ = 0.0;
  double forward_speed_threshold_ = 0.2;
  bool use_velocity_heading_ = false;
  bool invert_forward_dir_ = false;
  bool use_yaw_ = true;
  double yaw_speed_threshold_ = 0.2;
  double yaw_offset_ = 0.0;
  double clearance_radius_ = 0.0;
  double path_simplify_distance_ = 1.0;
  bool use_line_of_sight_prune_ = true;
  double line_of_sight_step_ = 0.5;

  int frontier_min_cluster_size_ = 5;
  int max_frontier_nodes_ = 50000;
  int max_astar_nodes_ = 200000;
};
