/*
 * Simple example that shows a trajectory planner using
 * mav_trajectory_generation in ROS2.
 *
 * Build + run via your ROS2 launch setup, e.g.
 *   ros2 run basic_waypoint_pkg basic_waypoint_node
 */

 #include <memory>
 #include <iostream>
 
 #include "rclcpp/rclcpp.hpp"
 #include "Eigen/Dense"
 
 #include "basic_waypoint_pkg/planner.hpp"
 
 int main(int argc, char ** argv)
 {
   rclcpp::init(argc, argv);
 
   // Create node
   auto node = rclcpp::Node::make_shared("simple_planner");
 
   // Instantiate basic planner
   BasicPlanner planner(node);
 
   // Let things settle a bit (similar to ros::Duration(1.0).sleep())
   rclcpp::Rate init_rate(10.0);
   init_rate.sleep();
 
   // Define goal point
   Eigen::Vector3d goal_position, goal_velocity;
   goal_position << -100.0, 40.0, 15.0;
   //-38.02096939086914, 9.998017311096191, 6.574100971221924;
   goal_velocity << 0.0, 0.0, 0.0;
 
   // Process some callbacks so that the odom callback can run
   rclcpp::Rate spin_rate(50.0);
   for (int i = 0; i < 10; ++i) {
     rclcpp::spin_some(node);
     spin_rate.sleep();
   }
 
   // Plan and publish trajectory
   mav_trajectory_generation::Trajectory trajectory;
   planner.planTrajectory(goal_position, goal_velocity, &trajectory);
   planner.publishTrajectory(trajectory);
 
   RCLCPP_WARN(node->get_logger(), "DONE. GOODBYE.");
 
   rclcpp::shutdown();
   return 0;
 }
 