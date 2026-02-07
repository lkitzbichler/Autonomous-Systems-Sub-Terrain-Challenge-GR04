#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <mutex>
#include <string>
#include <vector>

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode();

private:
  enum class Command : uint8_t {
    IDLE = 0,
    START = 1,
    STOP = 2,
    HOLD = 3,
    TAKEOFF = 4,
    LAND = 5,
    RETURN_HOME = 6,
    SCAN = 7
  };

  void onCommand(const std_msgs::msg::UInt8::SharedPtr msg);
  void onFrontierTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);

  void planTimer();
  void publishTimer();
  void readyTimer();

  bool planPath();
  bool isCollisionFree(const octomap::OcTree &tree,
                       const std::vector<octomap::point3d> &offsets,
                       double x, double y, double z) const;
  void updateInflationOffsets(double resolution);
  void publishTrajectoryPoint(const geometry_msgs::msg::Point &target,
                              const geometry_msgs::msg::Vector3 &velocity,
                              double yaw);
  void publishHoldPosition();
  void publishPlannedPath();

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr frontier_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;

  rclcpp::TimerBase::SharedPtr planner_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr ready_timer_;

  // State
  std::mutex map_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  bool have_map_{false};
  std::vector<octomap::point3d> inflation_offsets_;

  std::mutex odom_mutex_;
  geometry_msgs::msg::Point current_position_{};
  geometry_msgs::msg::Vector3 current_velocity_{};
  bool have_odom_{false};

  std::mutex goal_mutex_;
  geometry_msgs::msg::Point goal_position_{};
  bool have_goal_{false};

  std::mutex path_mutex_;
  std::vector<geometry_msgs::msg::Point> path_points_;
  size_t path_index_{0};
  bool path_active_{false};

  bool active_{false};
  bool planning_{false};
  bool need_replan_{false};
  bool return_home_mode_{false};
  double last_yaw_{0.0};

  // Parameters
  std::string command_topic_;
  std::string frontier_topic_;
  std::string odom_topic_;
  std::string octomap_topic_;
  std::string trajectory_topic_;
  std::string ready_topic_;
  std::string goal_reached_topic_;
  std::string map_frame_;

  bool auto_start_{true};
  bool require_map_{true};
  bool hold_when_idle_{true};
  bool simplify_path_{true};
  bool treat_unknown_as_occupied_{false};

  double publish_rate_{20.0};
  double planner_rate_{1.0};
  double max_planning_time_{1.0};
  double planner_range_{1.0};
  double goal_tolerance_{0.5};
  double waypoint_reached_dist_{0.4};
  double lookahead_dist_{0.8};
  double cruise_speed_{1.0};
  double collision_radius_{0.5};
  double occupancy_threshold_{0.5};
  double map_bound_padding_{1.0};
  double path_interpolation_resolution_{0.3};
  double yaw_offset_{0.0};
  double tf_timeout_sec_{0.2};

  std::vector<double> default_bounds_{-50.0, 50.0, -50.0, 50.0, 0.0, 20.0};
  std::vector<double> home_position_{-38.02, 10.0, 6.57};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};
