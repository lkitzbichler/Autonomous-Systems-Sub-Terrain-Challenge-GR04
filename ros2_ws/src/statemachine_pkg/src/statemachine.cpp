#include "statemachine_pkg/statemachine.h"

StateMachine::StateMachine() : Node("state_machine_node")
{
    RCLCPP_INFO(this->get_logger(), "State Machine Node has been started.");
    
    // Initialize subscribers, publishers, timers here
    // e.g.:
    // cmd_sub_ = this->create_subscription<std_msgs::msg::String>("command_topic", 10, std::bind(&StateMachine::commandCallback, this, std::placeholders::_1));
    // state_pub_ = this->create_publisher<std_msgs::msg::String>("state_topic", 10);
    // timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&StateMachine::onTimer, this));
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachine>());
    rclcpp::shutdown();
    return 0;
}
