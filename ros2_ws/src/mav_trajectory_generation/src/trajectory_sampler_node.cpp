/*
 * ROS2 port of trajectory_sampler_node.{h,cpp}
 */

 #include <chrono>

 #include "protocol.hpp"
 #include "trajectory_sampler_node.hpp"
 
 using namespace std::chrono_literals;
 
 TrajectorySamplerNode::TrajectorySamplerNode(const rclcpp::NodeOptions & options)
 : rclcpp::Node("trajectory_sampler_node", options),
   publish_whole_trajectory_(false),
   dt_(0.01),
   current_sample_time_(0.0)
 {
   // Parameters (with defaults from the original code)
  publish_whole_trajectory_ =
    this->declare_parameter<bool>("publish_whole_trajectory", publish_whole_trajectory_);
  dt_ = this->declare_parameter<double>("dt", dt_);
  heartbeat_topic_ = this->declare_parameter<std::string>("heartbeat_topic", "heartbeat");
  heartbeat_period_sec_ = this->declare_parameter<double>("heartbeat_period_sec", 1.0);
 
   // Publisher
  command_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectory>(
    mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
  heartbeat_pub_ = this->create_publisher<statemachine_pkg::msg::Answer>(heartbeat_topic_, 10);
 
   // Subscriptions
   trajectory_sub_ = this->create_subscription<mav_planning_msgs::msg::PolynomialTrajectory>(
     "path_segments",
     10,
     std::bind(&TrajectorySamplerNode::pathSegmentsCallback, this, std::placeholders::_1));
 
   trajectory4D_sub_ = this->create_subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>(
     "path_segments_4D",
     10,
     std::bind(&TrajectorySamplerNode::pathSegments4DCallback, this, std::placeholders::_1));
 
   // Service (stop sampling)
   stop_srv_ = this->create_service<std_srvs::srv::Empty>(
     "stop_sampling",
     std::bind(
       &TrajectorySamplerNode::stopSamplingCallback, this,
       std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
 
   // Client to take over publishing commands
   position_hold_client_ =
     this->create_client<std_srvs::srv::Empty>("back_to_position_hold");
 
   // Timer: create but don't start immediately (cancel, then reset when needed)
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(dt_),
    std::bind(&TrajectorySamplerNode::commandTimerCallback, this));
  publish_timer_->cancel();
  heartbeat_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(heartbeat_period_sec_),
    std::bind(&TrajectorySamplerNode::heartbeatTimerCallback, this));
 
  RCLCPP_INFO(get_logger(), "Initialized trajectory sampler (ROS2).");
}

void TrajectorySamplerNode::heartbeatTimerCallback()
{
  statemachine_pkg::msg::Answer hb;
  hb.node_name = this->get_name();
  hb.state = static_cast<uint8_t>(statemachine_pkg::protocol::AnswerStates::RUNNING);
  hb.info = "RUNNING";
  const auto now = this->now();
  hb.timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
  hb.timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
  heartbeat_pub_->publish(hb);
}
 
 void TrajectorySamplerNode::pathSegmentsCallback(
   const mav_planning_msgs::msg::PolynomialTrajectory::SharedPtr segments_message)
 {
   if (segments_message->segments.empty()) {
     RCLCPP_WARN(get_logger(), "Trajectory sampler: received empty waypoint message");
     return;
   } else {
     RCLCPP_INFO(
       get_logger(), "Trajectory sampler: received %zu waypoints",
       segments_message->segments.size());
   }
 
   bool success =
     mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(*segments_message, &trajectory_);
   if (!success) {
     RCLCPP_WARN(get_logger(), "Failed to convert PolynomialTrajectory message to trajectory.");
     return;
   }
 
   processTrajectory();
 }
 
 void TrajectorySamplerNode::pathSegments4DCallback(
   const mav_planning_msgs::msg::PolynomialTrajectory4D::SharedPtr segments_message)
 {
   if (segments_message->segments.empty()) {
     RCLCPP_WARN(get_logger(), "Trajectory sampler: received empty waypoint message (4D)");
     return;
   } else {
     RCLCPP_INFO(
       get_logger(), "Trajectory sampler (4D): received %zu waypoints",
       segments_message->segments.size());
   }
 
   bool success =
     mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(*segments_message, &trajectory_);
   if (!success) {
     RCLCPP_WARN(get_logger(), "Failed to convert PolynomialTrajectory4D message to trajectory.");
     return;
   }
 
   processTrajectory();
 }
 
 void TrajectorySamplerNode::processTrajectory()
 {
   // Call the service to tell the MAV interface to listen to our commands.
   if (position_hold_client_) {
     if (position_hold_client_->wait_for_service(0s)) {
       auto request = std::make_shared<std_srvs::srv::Empty::Request>();
       // Fire-and-forget; we don't wait on the future.
       (void)position_hold_client_->async_send_request(request);
     } else {
       RCLCPP_WARN(
         get_logger(),
         "Service 'back_to_position_hold' not available when starting trajectory sampling.");
     }
   }
 
   if (publish_whole_trajectory_) {
     // Publish the entire trajectory at once.
     mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
     mav_trajectory_generation::sampleWholeTrajectory(
       trajectory_, dt_, &trajectory_points);
 
     trajectory_msgs::msg::MultiDOFJointTrajectory msg_pub;
     msgMultiDofJointTrajectoryFromEigen(trajectory_points, &msg_pub);
     command_pub_->publish(msg_pub);
   } else {
     // Start timer-based streaming.
     current_sample_time_ = 0.0;
     start_time_ = this->get_clock()->now();
     publish_timer_->reset();
   }
 }
 
 void TrajectorySamplerNode::stopSamplingCallback(
   const std::shared_ptr<rmw_request_id_t> /*request_header*/,
   const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
   std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
 {
   if (publish_timer_) {
     publish_timer_->cancel();
   }
   RCLCPP_INFO(get_logger(), "Stopped trajectory sampling.");
 }
 
 void TrajectorySamplerNode::commandTimerCallback()
 {
   if (current_sample_time_ <= trajectory_.getMaxTime()) {
     trajectory_msgs::msg::MultiDOFJointTrajectory msg;
     mav_msgs::EigenTrajectoryPoint trajectory_point;
 
     bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
       trajectory_, current_sample_time_, &trajectory_point);
     if (!success) {
       RCLCPP_WARN(
         get_logger(),
         "Failed to sample trajectory at time %f, stopping timer.", current_sample_time_);
       publish_timer_->cancel();
       return;
     }
 
     mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
 
     if (!msg.points.empty()) {
       // Fill time_from_start (builtin_interfaces/msg/Duration)
       auto & tfs = msg.points[0].time_from_start;
       const double t = current_sample_time_;
       tfs.sec = static_cast<int32_t>(t);
       tfs.nanosec = static_cast<uint32_t>((t - tfs.sec) * 1e9);
     }
 
     command_pub_->publish(msg);
     current_sample_time_ += dt_;
   } else {
     publish_timer_->cancel();
     RCLCPP_INFO(get_logger(), "Finished streaming trajectory.");
   }
 }
 
 int main(int argc, char ** argv)
 {
   rclcpp::init(argc, argv);
   auto node = std::make_shared<TrajectorySamplerNode>();
   rclcpp::spin(node);
   rclcpp::shutdown();
   return 0;
 }
 
