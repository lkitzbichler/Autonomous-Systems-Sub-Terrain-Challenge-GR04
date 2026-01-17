#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <libsocket/inetclientstream.hpp>

class UnityCommandStream {
public:
  UnityCommandStream(std::string host, std::string port) {
    node_ = rclcpp::Node::make_shared("unity_command_stream");
    command_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "command_topic", 100, 
      std::bind(&UnityCommandStream::CommandCallback, this, std::placeholders::_1));
    socket_ptr_ = std::unique_ptr<libsocket::inet_stream>(new libsocket::inet_stream(host,
                                                             port,
                                                             LIBSOCKET_IPv4, 0));
  }
  
  void SendCommand(std::string command) {
    (*socket_ptr_) << command;
  }

private:
  void CommandCallback(const std_msgs::msg::String::SharedPtr command) {
    SendCommand(command->data);
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  std::unique_ptr<libsocket::inet_stream> socket_ptr_;
  rclcpp::Node::SharedPtr node_;
};