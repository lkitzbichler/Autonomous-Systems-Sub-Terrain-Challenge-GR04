#include "pathplanner.h"

#include <functional>

PathPlannerNode::PathPlannerNode()
  : rclcpp::Node("path_planner") {
  const auto command_topic =
      this->declare_parameter<std::string>("command_topic", "statemachine/command");

  command_sub_ = this->create_subscription<std_msgs::msg::String>(
      command_topic, 10,
      std::bind(&PathPlannerNode::onCommand, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "path_planner ready, listening on '%s'",
              command_topic.c_str());
}

void PathPlannerNode::onCommand(const std_msgs::msg::String::SharedPtr msg) {
  last_command_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "received command: %s", last_command_.c_str());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
