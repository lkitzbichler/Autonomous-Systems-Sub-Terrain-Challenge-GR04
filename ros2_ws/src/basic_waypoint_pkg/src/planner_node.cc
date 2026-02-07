/*
 * Simple example that shows a trajectory planner using
 * mav_trajectory_generation in ROS2.
 *
 * Build + run via your ROS2 launch setup, e.g.
 *   ros2 run basic_waypoint_pkg basic_waypoint_node
 */

 #include <algorithm>
 #include <memory>
 #include <iostream>
 #include <cmath>
 
 #include "rclcpp/rclcpp.hpp"
 #include "Eigen/Dense"
 #include "std_msgs/msg/bool.hpp"
 
 #include "basic_waypoint_pkg/planner.hpp"
 
 int main(int argc, char ** argv)
 {
   rclcpp::init(argc, argv);
 
   // Create node
   auto node = rclcpp::Node::make_shared("simple_planner");

   const auto done_topic =
     node->declare_parameter<std::string>("done_topic", "basic_waypoint/done");
   const auto done_delay_sec =
     node->declare_parameter<double>("done_delay_sec", 1.0);
   const auto use_trajectory_time =
     node->declare_parameter<bool>("use_trajectory_time", true);

   auto done_pub = node->create_publisher<std_msgs::msg::Bool>(done_topic, 1);
 
   // Instantiate basic planner
   BasicPlanner planner(node);
 
   // Let things settle a bit (similar to ros::Duration(1.0).sleep())
   rclcpp::Rate init_rate(10.0);
   init_rate.sleep();
 
   // Process some callbacks so that the odom callback can run
   rclcpp::Rate spin_rate(50.0);
   for (int i = 0; i < 10; ++i) {
     rclcpp::spin_some(node);
     spin_rate.sleep();
   }
 
  // Plan and publish trajectory
  mav_trajectory_generation::Trajectory trajectory;
  const bool planned = planner.planTrajectory(&trajectory);
  if (!planned) {
    RCLCPP_ERROR(node->get_logger(), "Failed to plan trajectory.");
    rclcpp::shutdown();
    return 1;
  }
  planner.publishTrajectory(trajectory);

  double wait_sec = std::max(0.0, done_delay_sec);
  if (use_trajectory_time) {
    double traj_sec = trajectory.getMaxTime();
    if (!std::isfinite(traj_sec) || traj_sec < 0.0) {
      traj_sec = 0.0;
    }
    wait_sec += traj_sec;
  }

  auto timer = node->create_wall_timer(
    std::chrono::duration<double>(wait_sec),
    [node, done_pub, done_topic]() {
      std_msgs::msg::Bool msg;
      msg.data = true;
      done_pub->publish(msg);
      RCLCPP_WARN(node->get_logger(),
                  "Published %s = true", done_topic.c_str());
      rclcpp::shutdown();
    });
  (void)timer;
 
   RCLCPP_WARN(node->get_logger(), "Waiting %.2fs before signaling done.", wait_sec);
 
   rclcpp::spin(node);
   return 0;
 }
 
