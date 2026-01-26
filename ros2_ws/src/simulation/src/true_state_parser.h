#pragma once

#include <unordered_map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "unity_stream_parser.h"

class TrueStateParser : public UnityStreamParser {
public:
  TrueStateParser() {
    node_ = rclcpp::Node::make_shared("true_state_parser");
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
  }

  virtual bool ParseMessage(const UnityHeader& header, 
                            TCPStreamReader& stream_reader,
                            double time_offset) override {
    float px, py, pz;
    float qw, qx, qy, qz;
    float vx, vy, vz;
    float rx, ry, rz;

    px = stream_reader.ReadFloat();
    py = stream_reader.ReadFloat();
    pz = stream_reader.ReadFloat();
    
    qx = stream_reader.ReadFloat();
    qy = stream_reader.ReadFloat();
    qz = stream_reader.ReadFloat();
    qw = stream_reader.ReadFloat();

    vx = stream_reader.ReadFloat();
    vy = stream_reader.ReadFloat();
    vz = stream_reader.ReadFloat();

    rx = stream_reader.ReadFloat();
    ry = stream_reader.ReadFloat();
    rz = stream_reader.ReadFloat();

    if(pose_publishers_.find(header.name) == pose_publishers_.end()) {
      pose_publishers_.insert(std::make_pair(header.name, node_->create_publisher<geometry_msgs::msg::PoseStamped>(header.name + "/pose", 10)));
      twist_publishers_.insert(std::make_pair(header.name, node_->create_publisher<geometry_msgs::msg::TwistStamped>(header.name + "/twist", 10)));
    }    

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.frame_id = "body"; //"odom_nav";
    pose_msg.header.stamp = rclcpp::Time(static_cast<int64_t>((header.timestamp + time_offset) * 1e9));
    
    pose_msg.pose.position.x = px;
    pose_msg.pose.position.y = pz;
    pose_msg.pose.position.z = py;

    pose_msg.pose.orientation.x = -qx;
    pose_msg.pose.orientation.y = -qz;
    pose_msg.pose.orientation.z = -qy;
    pose_msg.pose.orientation.w = qw;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.frame_id = "body"; //"odom_nav";
    twist_msg.header.stamp = pose_msg.header.stamp;
    
    twist_msg.twist.linear.x = vx;    
    twist_msg.twist.linear.y = vz;
    twist_msg.twist.linear.z = vy;

    twist_msg.twist.angular.x = -rx;
    twist_msg.twist.angular.y = -rz;
    twist_msg.twist.angular.z = -ry;

    pose_publishers_[header.name]->publish(pose_msg);
    twist_publishers_[header.name]->publish(twist_msg);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = pose_msg.header.stamp;
    tf.header.frame_id = "world";
    tf.child_frame_id = "true_body";

    tf.transform.translation.x = pose_msg.pose.position.x;
    tf.transform.translation.y = pose_msg.pose.position.y;
    tf.transform.translation.z = pose_msg.pose.position.z;

    tf.transform.rotation.x = pose_msg.pose.orientation.x;
    tf.transform.rotation.y = pose_msg.pose.orientation.y;
    tf.transform.rotation.z = pose_msg.pose.orientation.z;
    tf.transform.rotation.w = pose_msg.pose.orientation.w;

    tf_broadcaster_->sendTransform(tf);
    return true;
  }

private:
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_publishers_;
  std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> twist_publishers_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Node::SharedPtr node_;
};
