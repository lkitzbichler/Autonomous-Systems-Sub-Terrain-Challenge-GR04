#pragma once

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
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
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

  // Core callbacks
  void onCommand(const statemachine_pkg::msg::Command::SharedPtr msg);
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void onDepthImage(const sensor_msgs::msg::Image::SharedPtr msg);
  void onDepthCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

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
  bool chooseReturnGoal(Eigen::Vector3d &goal_out, std::string &reason_out) const;
  bool chooseCenterlineGoal(Eigen::Vector3d &goal_out, std::string &reason_out);
  bool projectGoalToFree(Eigen::Vector3d &goal_in_out) const;
  bool chooseMapEscapeDirection(
    const Eigen::Vector3d &preferred_xy,
    Eigen::Vector3d &direction_xy_out) const;
  bool computeForwardDirection(Eigen::Vector3d &forward_out);
  bool decodeDepthMeters(
    const sensor_msgs::msg::Image &img,
    int u,
    int v,
    float &depth_m_out) const;
  bool extractConnectedSliceCenters(
    const Eigen::Vector3d &slice_center,
    const Eigen::Vector3d &lateral_axis,
    const Eigen::Vector3d &vertical_axis,
    const Eigen::Vector3d &anchor_point,
    Eigen::Vector3d &main_center_out,
    std::vector<Eigen::Vector3d> &secondary_centers_out) const;

  bool planPathOmpl(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal,
    std::vector<Eigen::Vector3d> &path_out) const;

  bool resamplePath(
    const std::vector<Eigen::Vector3d> &path_in,
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
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;

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
  std::string depth_image_topic_;
  std::string depth_camera_info_topic_;
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
  int max_frontier_fail_cycles_{8};
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
  double depth_free_min_m_{1.2};
  double depth_max_m_{25.0};
  int depth_min_component_pixels_{80};
  double depth_hint_weight_{0.45};
  double depth_hint_timeout_sec_{0.8};
  double depth_hint_min_align_dot_{0.25};
  double depth_body_yaw_correction_deg_{180.0};
  double depth_far_percentile_{0.72};
  double depth_target_range_min_m_{2.8};
  double map_escape_lookahead_m_{6.0};
  double forward_smoothing_alpha_{0.78};
  double escape_blend_weight_{0.30};
  int centerline_horizon_points_{8};
  double centerline_step_m_{2.0};
  double centerline_slice_half_extent_m_{4.0};
  double centerline_slice_step_m_{0.5};
  int centerline_min_component_cells_{6};
  int centerline_max_secondary_goals_{4};
  double centerline_execute_dist_m_{16.0};
  double centerline_min_commit_dist_m_{10.0};
  int marker_max_candidates_{120};

  // Runtime state
  PlannerMode mode_{PlannerMode::IDLE};
  bool planner_done_{false};
  bool has_current_pose_{false};
  bool has_home_pose_{false};
  bool has_octomap_{false};
  bool has_current_goal_{false};

  int frontier_fail_cycles_{0};
  Eigen::Vector3d current_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d heading_hint_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d body_forward_hint_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d body_left_hint_{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d body_up_hint_{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d depth_forward_hint_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d smoothed_forward_hint_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d home_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d current_goal_{Eigen::Vector3d::Zero()};
  rclcpp::Time last_plan_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_depth_hint_time_{0, 0, RCL_ROS_TIME};

  std::vector<Eigen::Vector3d> visited_goals_;
  std::vector<Eigen::Vector3d> tracked_lanterns_;
  std::vector<Eigen::Vector3d> debug_candidate_points_;
  std::vector<Eigen::Vector3d> debug_centerline_points_;
  bool has_depth_hint_{false};
  bool has_smoothed_forward_hint_{false};
  bool depth_wall_blocked_{false};
  bool has_depth_intrinsics_{false};
  double depth_fx_{0.0};
  double depth_fy_{0.0};
  double depth_cx_{0.0};
  double depth_cy_{0.0};
  int depth_anchor_u_{-1};
  int depth_anchor_v_{-1};
  Eigen::Vector3d depth_aim_point_world_{Eigen::Vector3d::Zero()};

  std::shared_ptr<octomap::OcTree> octree_;
};
