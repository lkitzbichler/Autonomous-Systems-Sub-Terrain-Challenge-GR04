#pragma once

#include <rclcpp/rclcpp.hpp>
#include <statemachine_pkg/msg/answer.hpp>
#include <statemachine_pkg/msg/command.hpp>

#include <string>

class PathPlannerNode : public rclcpp::Node {
public:
  PathPlannerNode();

private:
  void onCommand(const statemachine_pkg::msg::Command::SharedPtr msg);
  void publishHeartbeat();

  rclcpp::Subscription<statemachine_pkg::msg::Command>::SharedPtr command_sub_;
  rclcpp::Publisher<statemachine_pkg::msg::Answer>::SharedPtr heartbeat_pub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  std::string command_topic_;
  std::string heartbeat_topic_;
  double heartbeat_period_sec_{1.0};
  bool planner_active_{false};
};
