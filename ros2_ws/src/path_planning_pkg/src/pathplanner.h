#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

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
  enum class PlannerMode : uint8_t {
    IDLE = 0,
    EXPLORE = 1,
    RETURN_HOME = 2,
  };

  enum class BranchStatus : uint8_t {
    OPEN = 0,
    ASSIGNED = 1,
    VISITED = 2,
    BLOCKED = 3,
  };

  struct GraphBranch {
    Eigen::Vector3d goal{Eigen::Vector3d::Zero()};
    BranchStatus status{BranchStatus::OPEN};
    int target_node{-1};
  };

  struct GraphNode {
    int id{-1};
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    std::vector<int> neighbors;
    std::vector<GraphBranch> branches;
    bool visited{false};
    bool is_home{false};
    bool is_junction{false};
    bool is_dead_end{false};
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

  // Mission planning pipeline
  bool chooseGoal(std::vector<Eigen::Vector3d> &route_points_out, std::string &reason_out);
  bool chooseExploreGoal(std::vector<Eigen::Vector3d> &route_points_out, std::string &reason_out);
  bool chooseReturnGoal(std::vector<Eigen::Vector3d> &route_points_out, std::string &reason_out) const;
  bool buildPathFromRoute(
    const std::vector<Eigen::Vector3d> &route_points,
    std::vector<Eigen::Vector3d> &path_out) const;
  bool isFollowingActivePath() const;
  static double distanceToPath(
    const Eigen::Vector3d &p,
    const std::vector<Eigen::Vector3d> &path_points);
  static bool pathProgressMetrics(
    const Eigen::Vector3d &p,
    const std::vector<Eigen::Vector3d> &path_points,
    double &distance_out,
    double &progress_ratio_out,
    double &remaining_dist_out);

  bool planPathOmpl(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal,
    std::vector<Eigen::Vector3d> &path_out) const;
  bool smoothPathCorners(std::vector<Eigen::Vector3d> &path_in_out) const;
  bool centerPathInTunnel(std::vector<Eigen::Vector3d> &path_in_out) const;

  bool resamplePath(
    const std::vector<Eigen::Vector3d> &path_in,
    std::vector<Eigen::Vector3d> &waypoints_out) const;

  bool buildAndPublishTrajectory(
    const std::vector<Eigen::Vector3d> &waypoints,
    bool force_stop_at_end);

  // Graph helpers
  void seedHomeNodeFromCommand(const geometry_msgs::msg::Point &target);
  void updateGraphModel();
  int addOrMergeGraphNode(const Eigen::Vector3d &p, bool mark_visited, bool prefer_junction);
  void connectGraphNodes(int a, int b);
  int findClosestGraphNode(const Eigen::Vector3d &p, double max_dist) const;
  void refreshBranchesAroundNode(int node_id);
  std::vector<Eigen::Vector3d> sampleBranchGoals(const Eigen::Vector3d &origin) const;
  bool recenterToLocalMidpoint(Eigen::Vector3d &point_in_out, double search_radius) const;
  bool extractSliceComponentCenters(
    const Eigen::Vector3d &slice_center,
    const Eigen::Vector3d &lateral_axis,
    const Eigen::Vector3d &vertical_axis,
    std::vector<Eigen::Vector3d> &component_centers_out) const;
  bool buildForwardCenterlinePreview(
    int start_node,
    std::vector<Eigen::Vector3d> &centerline_points_out,
    Eigen::Vector3d &primary_goal_out,
    std::vector<Eigen::Vector3d> &secondary_goals_out) const;
  void seedCenterlinePreviewNodes(
    int start_node,
    const std::vector<Eigen::Vector3d> &centerline_points);
  bool chooseEntryBranchFromView(int start_node, int &branch_index_out);
  bool computeDesiredDirection(int start_node, Eigen::Vector3d &dir_out) const;
  double estimateForwardObstacleDistance(
    const Eigen::Vector3d &origin,
    const Eigen::Vector3d &dir_xy,
    double max_dist) const;
  void updateNodeClassification(int node_id);
  bool computeShortestNodePath(int start_node, int goal_node, std::vector<int> &path_nodes_out) const;
  double computeNodePathLength(const std::vector<int> &path_nodes) const;
  bool hasOpenFrontiers() const;
  void setActiveBranchStatus(BranchStatus status, bool clear_active_state);
  void markActiveBranchVisited();

  // Map/geometry helpers
  bool hasValidMapAndPose() const;
  bool isPointKnownFree(const Eigen::Vector3d &p) const;
  bool isPointSafe(const Eigen::Vector3d &p) const;
  bool isSegmentSafe(const Eigen::Vector3d &a, const Eigen::Vector3d &b) const;
  bool isPathSafe(const std::vector<Eigen::Vector3d> &path) const;
  double estimateClearance(const Eigen::Vector3d &p, double max_radius) const;
  double revisitPenalty(const Eigen::Vector3d &p) const;
  bool projectGoalToFree(Eigen::Vector3d &goal_in_out) const;
  static double distance3(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

  // Mission helper logic
  bool hasReachedGoal() const;
  void storeVisitedGoal(const Eigen::Vector3d &goal);
  bool shouldReportDone() const;
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
  double active_path_follow_dist_m_{2.5};
  double active_path_replan_progress_ratio_{0.35};
  double active_path_replan_front_obstacle_m_{9.0};
  double ompl_timeout_sec_{0.25};
  double ompl_range_m_{1.5};
  double ompl_max_turn_deg_{80.0};
  int ompl_corner_smoothing_passes_{3};
  double ompl_corner_blend_{0.65};
  double bounds_padding_m_{1.0};
  int max_frontier_fail_cycles_{8};
  double lantern_merge_dist_m_{0.2};
  double heading_hint_min_speed_{0.25};
  bool adaptive_waypoint_step_enabled_{true};
  double adaptive_step_clearance_range_m_{2.0};
  double adaptive_step_turn_weight_{0.7};
  double adaptive_step_min_ratio_{0.35};
  double traj_start_speed_max_ratio_{0.35};
  double traj_start_align_cos_min_{0.55};
  double traj_time_scale_{1.35};
  int marker_max_candidates_{120};

  // Graph-specific parameters
  double graph_backbone_spacing_m_{5.0};
  double graph_explore_spacing_m_{2.8};
  double graph_merge_dist_m_{1.2};
  double graph_junction_probe_dist_m_{3.5};
  double graph_branch_goal_dist_m_{15.0};
  double graph_branch_merge_dist_m_{1.6};
  double graph_frontier_revisit_penalty_{1.2};
  double graph_forward_bias_weight_{0.8};
  double graph_vertical_penalty_weight_{1.0};
  double graph_view_align_weight_{1.2};
  double graph_center_search_radius_m_{2.2};
  int graph_min_junction_free_dirs_{3};
  int graph_max_frontiers_per_node_{8};
  int centerline_horizon_points_{8};
  double centerline_step_m_{5.0};
  double centerline_slice_half_extent_m_{5.0};
  double centerline_slice_step_m_{0.6};
  int centerline_min_component_cells_{4};
  int centerline_max_secondary_goals_{4};
  double centerline_execute_dist_m_{30.0};
  double centerline_min_commit_dist_m_{20.0};
  double body_yaw_correction_deg_{180.0};

  // Runtime state
  PlannerMode mode_{PlannerMode::IDLE};
  bool planner_done_{false};
  bool has_current_pose_{false};
  bool has_home_pose_{false};
  bool has_home_seed_{false};
  bool has_octomap_{false};
  bool has_current_goal_{false};

  int frontier_fail_cycles_{0};
  int home_node_id_{-1};
  int current_node_id_{-1};
  int previous_graph_node_id_{-1};
  int last_graph_node_id_{-1};
  int active_branch_node_id_{-1};
  int active_branch_index_{-1};
  bool has_active_branch_goal_{false};
  bool prefer_local_expansion_{false};
  bool prefer_view_aligned_entry_{false};
  int local_expansion_node_id_{-1};

  Eigen::Vector3d current_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_forward_hint_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d heading_hint_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d planned_forward_hint_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d home_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d home_seed_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_goal_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d active_branch_goal_{Eigen::Vector3d::Zero()};
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};

  std::vector<Eigen::Vector3d> visited_goals_;
  std::vector<Eigen::Vector3d> tracked_lanterns_;
  std::vector<Eigen::Vector3d> debug_candidate_points_;
  std::vector<Eigen::Vector3d> active_path_points_;
  std::vector<GraphNode> graph_nodes_;

  std::shared_ptr<octomap::OcTree> octree_;
};
