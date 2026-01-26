#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode();

private:
  void onCommand(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  std::string last_command_;
};
