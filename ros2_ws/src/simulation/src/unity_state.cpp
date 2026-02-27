#include <memory>
#include <string>
#include <iostream>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "TCPStreamReader.h"

// Set to 1 to publish TF world->av as in ROS1 code
#ifndef TFOUTPUT
#define TFOUTPUT 1
#endif

class StateServer : public rclcpp::Node
{
public:
  StateServer(const std::string & ip_address, const std::string & port)
  : rclcpp::Node("unity_state"),
    stream_reader_(ip_address, port)
  {
    // Topic name: avoid leading "/" in ROS2; remap if you need global
    pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("current_state", 10);

#if TFOUTPUT
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
#endif
  }

  void connect()
  {
    RCLCPP_INFO(get_logger(), "[unity_state] Waiting for connection...");
    stream_reader_.WaitConnect();
    RCLCPP_INFO(get_logger(), "[unity_state] Got a connection...");
  }

  // Blocks once, reads one packet, publishes, throws on hard failure
  void publish_state_once()
  {
    if (!stream_reader_.Good()) {
      RCLCPP_ERROR(get_logger(), "[unity_state] state server connection broken");
      throw std::runtime_error("connection broken");
    }

    try {
      // 13 floats expected
      auto data = stream_reader_.ReadBytes(13 * sizeof(float));
      auto * begin = reinterpret_cast<float *>(data.get());

      // Unity is left-handed; same flips as original code:
      nav_msgs::msg::Odometry state_msg;
      state_msg.header.stamp = this->get_clock()->now();
      state_msg.header.frame_id = "world";
      state_msg.child_frame_id = "av";

      // position (x, z, y)
      state_msg.pose.pose.position.x = begin[0];
      state_msg.pose.pose.position.y = begin[2];
      state_msg.pose.pose.position.z = begin[1];

      // quaternion (flip y & z + negate vector part)
      state_msg.pose.pose.orientation.x = -begin[3];
      state_msg.pose.pose.orientation.y = -begin[5];
      state_msg.pose.pose.orientation.z = -begin[4];
      state_msg.pose.pose.orientation.w =  begin[6];

      // linear velocity (x, z, y)
      state_msg.twist.twist.linear.x = begin[7];
      state_msg.twist.twist.linear.y = begin[9];
      state_msg.twist.twist.linear.z = begin[8];

      // angular velocity (negated x, z, y)
      state_msg.twist.twist.angular.x = -begin[10];
      state_msg.twist.twist.angular.y = -begin[12];
      state_msg.twist.twist.angular.z = -begin[11];

      pose_pub_->publish(state_msg);

#if TFOUTPUT
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header.stamp = state_msg.header.stamp;
      tf_msg.header.frame_id = state_msg.header.frame_id; // "world"
      tf_msg.child_frame_id  = state_msg.child_frame_id;  // "av"
      tf_msg.transform.translation.x = state_msg.pose.pose.position.x;
      tf_msg.transform.translation.y = state_msg.pose.pose.position.y;
      tf_msg.transform.translation.z = state_msg.pose.pose.position.z;
      tf_msg.transform.rotation      = state_msg.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
#endif

    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Shutting down state server: %s", e.what());
      stream_reader_.Shutdown();
      throw; // propagate so main can exit like ROS1 did
    }
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  TCPStreamReader stream_reader_;
#if TFOUTPUT
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
#endif
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Defaults kept from ROS1
  std::string port = "12347";
  std::string ip_address = "127.0.0.1";

  auto node = std::make_shared<StateServer>(ip_address, port);
  node->connect();

  // Keep the same blocking loop semantics as ROS1
  while (rclcpp::ok()) {
    node->publish_state_once();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
