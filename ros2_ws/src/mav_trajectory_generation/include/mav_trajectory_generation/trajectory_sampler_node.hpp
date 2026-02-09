/*
 * ROS2 port of trajectory_sampler_node.{h,cpp}
 */

 #ifndef MAV_TRAJECTORY_GENERATION_ROS__TRAJECTORY_SAMPLER_NODE_HPP_
 #define MAV_TRAJECTORY_GENERATION_ROS__TRAJECTORY_SAMPLER_NODE_HPP_
 
 #include <memory>
 #include <string>
 #include <vector>
 
 #include <rclcpp/rclcpp.hpp>
 
 #include <std_srvs/srv/empty.hpp>
 #include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
 
 #include <mav_msgs/conversions.hpp>
 #include <mav_msgs/default_topics.hpp>
 #include <mav_msgs/eigen_mav_msgs.hpp>
 
 #include <mav_planning_msgs/msg/polynomial_segment.hpp>
 #include <mav_planning_msgs/msg/polynomial_trajectory.hpp>
 #include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
 #include <statemachine_pkg/msg/answer.hpp>
 
 #include <mav_trajectory_generation/polynomial.h>
 #include <mav_trajectory_generation/trajectory_sampling.h>
 #include <mav_trajectory_generation/ros_conversions.h>
 
 class TrajectorySamplerNode : public rclcpp::Node
 {
 public:
   explicit TrajectorySamplerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
   ~TrajectorySamplerNode() override = default;
 
 private:
   // Callbacks
   void pathSegmentsCallback(
     const mav_planning_msgs::msg::PolynomialTrajectory::SharedPtr segments_message);
 
   void pathSegments4DCallback(
     const mav_planning_msgs::msg::PolynomialTrajectory4D::SharedPtr segments_message);
 
   void stopSamplingCallback(
     const std::shared_ptr<rmw_request_id_t> request_header,
     const std::shared_ptr<std_srvs::srv::Empty::Request> request,
     std::shared_ptr<std_srvs::srv::Empty::Response> response);
 
  void commandTimerCallback();
  void heartbeatTimerCallback();
 
   // Helper
   void processTrajectory();
 
   // ROS 2 entities
   rclcpp::Subscription<mav_planning_msgs::msg::PolynomialTrajectory>::SharedPtr trajectory_sub_;
   rclcpp::Subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr trajectory4D_sub_;
   rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>::SharedPtr command_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_srv_;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr position_hold_client_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::Publisher<statemachine_pkg::msg::Answer>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
 
   rclcpp::Time start_time_;
 
   // Parameters / state
  bool publish_whole_trajectory_;
  double dt_;
  double current_sample_time_;
  std::string heartbeat_topic_;
  double heartbeat_period_sec_{1.0};
 
   // The trajectory to sample.
   mav_trajectory_generation::Trajectory trajectory_;
 };
 
 #endif  // MAV_TRAJECTORY_GENERATION_ROS__TRAJECTORY_SAMPLER_NODE_HPP_
 
