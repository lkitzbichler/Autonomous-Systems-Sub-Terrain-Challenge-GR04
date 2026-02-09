#include "pathplanner.h"

#include <chrono>
#include <functional>

#include "protocol.hpp"

PathPlannerNode::PathPlannerNode()
  : rclcpp::Node("path_planner") {
  // Step 1: Load command/heartbeat parameters
  command_topic_ = this->declare_parameter<std::string>("command_topic", "statemachine/cmd");
  heartbeat_topic_ = this->declare_parameter<std::string>("heartbeat_topic", "heartbeat");
  heartbeat_period_sec_ = this->declare_parameter<double>("heartbeat_period_sec", 1.0);

  // Step 2: Create command subscription and heartbeat publisher/timer
  command_sub_ = this->create_subscription<statemachine_pkg::msg::Command>(
      command_topic_, 10,
      std::bind(&PathPlannerNode::onCommand, this, std::placeholders::_1));
  heartbeat_pub_ = this->create_publisher<statemachine_pkg::msg::Answer>(heartbeat_topic_, 10);
  heartbeat_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(heartbeat_period_sec_),
      std::bind(&PathPlannerNode::publishHeartbeat, this));

  RCLCPP_INFO(this->get_logger(), "path_planner ready, listening on '%s'",
              command_topic_.c_str());
}

void PathPlannerNode::onCommand(const statemachine_pkg::msg::Command::SharedPtr msg) {
  // Step 1: Validate and filter by target node
  if (!msg) {
    return;
  }
  if (msg->target != this->get_name()) {
    return;
  }

  // Step 2: Handle relevant state-machine commands
  switch (msg->command) {
    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::START):
      planner_active_ = true;
      RCLCPP_INFO(this->get_logger(), "[cmd] START");
      break;
    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::RETURN_HOME):
      planner_active_ = true;
      RCLCPP_INFO(this->get_logger(), "[cmd] RETURN_HOME");
      break;
    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::HOLD):
    case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::ABORT):
      planner_active_ = false;
      RCLCPP_INFO(this->get_logger(), "[cmd] HOLD/ABORT");
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "[cmd] ignored id=%u", msg->command);
      break;
  }
}

void PathPlannerNode::publishHeartbeat() {
  // Keep status compact: RUNNING always, detail in info field.
  statemachine_pkg::msg::Answer hb;
  hb.node_name = this->get_name();
  hb.state = static_cast<uint8_t>(statemachine_pkg::protocol::AnswerStates::RUNNING);
  hb.info = planner_active_ ? "RUNNING_ACTIVE" : "RUNNING_IDLE";
  const auto now = this->now();
  hb.timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
  hb.timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
  heartbeat_pub_->publish(hb);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
