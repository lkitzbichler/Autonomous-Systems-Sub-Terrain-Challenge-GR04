#pragma once

#include <memory>
#include <string>
#include <vector>
#include <deque>

#include <Eigen/Core>

#include <geometry_msgs/msg/pose_array.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <statemachine_pkg/msg/answer.hpp>
#include <statemachine_pkg/msg/command.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode();

private:
  struct FrontierCluster {
    std::vector<Eigen::Vector3d> cells;
    Eigen::Vector3d centroid{Eigen::Vector3d::Zero()};
  };

  enum class PlannerMode : uint8_t {
    IDLE = 0,
    EXPLORE = 1,
    RETURN_HOME = 2,
  };
  enum class ReturnPhase : uint8_t {
    INACTIVE = 0,
    TRACE_PATH = 1,
    RECOVERY_OMPL = 2,
    FINAL_HOME = 3,
    COMPLETE = 4,
  };

  // Core callbacks
  void onCommand(const statemachine_pkg::msg::Command::SharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg);

  // Timers
  void onPlannerTimer();
  void onHeartbeatTimer();
  void onParameterRefreshTimer();

  // Parameter handling
  void declareAndLoadParameters();
  void refreshDynamicParameters();

  // Planning pipeline
  bool chooseGoal(Eigen::Vector3d &goal_out, std::string &reason_out);
  bool chooseExploreGoal(Eigen::Vector3d &goal_out, std::string &reason_out);
  bool chooseFallbackGoal(Eigen::Vector3d &goal_out, std::string &reason_out);
  bool computeFrontierClusters(std::vector<FrontierCluster> &clusters_out) const;
  bool selectFrontierClusterGoal(
    const std::vector<FrontierCluster> &clusters,
    const Eigen::Vector3d &forward_hint,
    Eigen::Vector3d &goal_out,
    std::string &reason_out);
  bool chooseReturnGoal(Eigen::Vector3d &goal_out, std::string &reason_out);
  bool projectGoalToFree(Eigen::Vector3d &goal_in_out) const;
  bool popBacktrackGoal(Eigen::Vector3d &goal_out);

  bool planPathOmpl(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal,
    std::vector<Eigen::Vector3d> &path_out) const;

  bool resamplePath(
    const std::vector<Eigen::Vector3d> &path_in,
    std::vector<Eigen::Vector3d> &waypoints_out) const;
  bool compressTrajectoryWaypoints(
    const std::vector<Eigen::Vector3d> &waypoints_in,
    std::vector<Eigen::Vector3d> &waypoints_out) const;

  bool buildAndPublishTrajectory(
    const std::vector<Eigen::Vector3d> &waypoints,
    bool force_stop_at_end);

  // Map/geometry helpers
  bool hasValidMapAndPose() const;
  bool isPointKnownFree(const Eigen::Vector3d &p) const;
  bool isPointSafe(const Eigen::Vector3d &p) const;
  bool isSegmentSafe(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const;
  bool isPathSafe(const std::vector<Eigen::Vector3d> &path) const;
  int countUnknownNeighbors(const Eigen::Vector3d &p) const;
  int countKnownFreeNeighbors(const Eigen::Vector3d &p) const;
  double estimateClearance(const Eigen::Vector3d &p, double max_radius) const;
  double revisitPenalty(const Eigen::Vector3d &p) const;
  static double distance3(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

  // Mission helper logic
  bool hasReachedGoal() const;
  void storeVisitedGoal(const Eigen::Vector3d &goal);
  void storeBacktrackGoal(const Eigen::Vector3d &goal);
  bool shouldReportDone() const;
  void recordBreadcrumb(const Eigen::Vector3d &position);
  void resetReturnHomeState();
  void publishHeartbeat(uint8_t state, const std::string &info) const;
  void publishPlanningVisualization(
    const std::vector<Eigen::Vector3d> &ompl_path,
    const std::vector<Eigen::Vector3d> &waypoints);
  void publishSelectedPath(const std::vector<Eigen::Vector3d> &path_points);

  // ROS interfaces
  rclcpp::Subscription<statemachine_pkg::msg::Command>::SharedPtr command_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr lantern_sub_;

  rclcpp::Publisher<statemachine_pkg::msg::Answer>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr selected_path_pub_;

  rclcpp::TimerBase::SharedPtr planner_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr parameter_refresh_timer_;

  // Topics
  std::string command_topic_;
  std::string heartbeat_topic_;
  std::string odom_topic_;
  std::string octomap_topic_;
  std::string trajectory_topic_;
  std::string lantern_topic_;
  std::string marker_topic_;
  std::string selected_path_topic_;
  std::string viz_frame_id_;

  // Timing params
  double planner_period_sec_{1.0};
  double heartbeat_period_sec_{1.0};
  double parameter_refresh_period_sec_{10.0};

  // Dynamic flight/planning params
  double max_v_{2.2};
  double max_a_{1.6};
  double waypoint_dist_min_{1.5};
  double waypoint_dist_max_{3.5};
  double wall_clearance_min_{1.2};
  double path_check_step_m_{0.2};
  double clearance_probe_radius_m_{4.0};
  double goal_reached_radius_m_{0.9};
  double replan_timeout_sec_{4.5};
  double frontier_goal_dist_min_{3.0};
  double frontier_goal_dist_max_{12.0};
  double ompl_timeout_sec_{0.25};
  double ompl_range_m_{1.5};
  double bounds_padding_m_{1.0};
  int frontier_unknown_min_neighbors_{2};
  int frontier_known_free_min_neighbors_{8};
  int frontier_min_cluster_size_{5};
  int max_frontier_fail_cycles_{8};
  bool backtrack_enabled_{true};
  double backtrack_min_distance_m_{7.0};
  int backtrack_max_points_{60};
  double breadcrumb_min_spacing_m_{1.0};
  int breadcrumb_max_points_{2000};
  double return_goal_reached_radius_m_{0.9};
  int required_lantern_count_{5};
  double lantern_merge_dist_m_{0.2};
  double score_unknown_weight_{1.2};
  double score_clearance_weight_{1.0};
  double score_distance_weight_{0.5};
  double score_revisit_weight_{1.4};
  double score_forward_weight_{0.8};
  double heading_hint_min_speed_{0.25};
  double forward_min_dot_{-0.15};
  bool adaptive_waypoint_step_enabled_{true};
  double adaptive_step_clearance_range_m_{2.0};
  double adaptive_step_turn_weight_{0.7};
  double adaptive_step_min_ratio_{0.35};
  int trajectory_waypoint_target_count_{5};
  int marker_max_candidates_{120};

  // Runtime state
  PlannerMode mode_{PlannerMode::IDLE};
  bool planner_done_{false};
  bool has_current_pose_{false};
  bool has_home_pose_{false};
  bool has_octomap_{false};
  bool has_current_goal_{false};
  bool current_goal_from_backtrack_{false};
  ReturnPhase return_phase_{ReturnPhase::INACTIVE};
  int return_breadcrumb_index_{-1};

  int frontier_fail_cycles_{0};
  Eigen::Vector3d current_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d heading_hint_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d home_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_goal_{Eigen::Vector3d::Zero()};
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

  std::vector<Eigen::Vector3d> visited_goals_;
  std::vector<Eigen::Vector3d> tracked_lanterns_;
  std::vector<Eigen::Vector3d> debug_candidate_points_;
  std::deque<Eigen::Vector3d> backtrack_goals_;
  std::deque<Eigen::Vector3d> breadcrumbs_;

  std::shared_ptr<octomap::OcTree> octree_;
};
