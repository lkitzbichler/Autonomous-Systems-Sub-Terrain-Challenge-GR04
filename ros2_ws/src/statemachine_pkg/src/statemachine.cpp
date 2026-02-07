#include "statemachine_pkg/statemachine.h"
#include "statemachine_pkg/msg/answer.hpp"

StateMachine::StateMachine() : Node("state_machine_node")
{
    RCLCPP_INFO(this->get_logger(), "[][StateMachine]: State Machine Node has been started.");
    
    // Create and Load Parameters #####################################################################
    // Lists
    this->checkpoint_list_ = declare_parameter<std::vector<double>>("checkpoints", std::vector<double>{});
    this->node_list_ = declare_parameter<std::vector<std::string>>("nodes", std::vector<std::string>{});
    // Timer periods
    this->pub_state_loop_sec_ = declare_parameter<double>("time/pub_state_loop_sec", 1.0);
    this->alive_tol_sec_ = declare_parameter<double>("time/alive_tol_sec", 10.0);
    // Lantern detections
    this->min_lantern_dist_ = declare_parameter<double>("lanterns/min_merge_dist_m", 0.2);
    // Logging paths
    this->lantern_log_path_ = declare_parameter<std::string>("logging/lantern_log_path", "lanterns_log.csv");
    this->event_log_path_ = declare_parameter<std::string>("logging/event_log_path", "statemachine_events.log");

    // TOPIC PARAMETERS ###############################################################################
    this->topic_state_ = declare_parameter<std::string>("topics/state", "statemachine/state");
    this->topic_cmd_ = declare_parameter<std::string>("topics/cmd", "statemachine/cmd");
    this->topic_heartbeat_ = declare_parameter<std::string>("topics/heartbeat", "statemachine/heartbeat");
    this->topic_lantern_detections_ = declare_parameter<std::string>("topics/lantern_detections", "lantern_detector/detections");
    this->topic_viz_checkpoint = declare_parameter<std::string>("topics/viz_checkpoint", "statemachine/viz_checkpoint");
    this->topic_viz_path_flight = declare_parameter<std::string>("topics/viz_path_flight", "statemachine/viz_path_flight");

    // Create Publishers ##############################################################################
    this->pub_state_ = this->create_publisher<std_msgs::msg::String>(topic_state_, 10);
    this->pub_cmd_ = this->create_publisher<std_msgs::msg::String>(topic_cmd_, 10);
    this->pub_viz_checkpoint_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_viz_checkpoint, 10);
    this->pub_viz_path_flight_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_viz_path_flight, 10);

    // Create Subscribers #############################################################################
    this->sub_heartbeat_ = this->create_subscription<statemachine_pkg::msg::Answer>(topic_heartbeat_, 10, std::bind(&StateMachine::handleAnswer, this, std::placeholders::_1));
    this->sub_lantern_detections_ = this->create_subscription<geometry_msgs::msg::PoseArray>(topic_lantern_detections_, 10, std::bind(&StateMachine::onLanternDetections, this, std::placeholders::_1));

    // Create Timers ##################################################################################
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&StateMachine::onTimer, this));
}

StateMachine::~StateMachine()
{
    RCLCPP_INFO(this->get_logger(), "State Machine Node is shutting down.");
}

void StateMachine::publishState()
{
    // TODO: publish current state once message type is finalized
}

void StateMachine::changeState(MissionState new_state, const std::string &reason)
{
    (void)reason;
    if (state_ != new_state) {
        state_ = new_state;
        state_enter_time_ = this->now(); 
        RCLCPP_INFO(this->get_logger(), "State changed to %s. Reason: %s", toString(state_).c_str(), reason.c_str());
        publishState();
    }
}

void StateMachine::sendCommand(std::string recv_node, Command cmd)
{
    (void)recv_node;
    (void)cmd;
    // TODO: update node heartbeat/answers once message format is finalized
}


void StateMachine::handleAnswer(const statemachine_pkg::msg::Answer::SharedPtr msg)
{
    (void)msg;
    // TODO: update node heartbeat/answers once message format is finalized
}

void StateMachine::onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    (void)msg;
    // TODO: handle lantern detections
}

void StateMachine::associateLantern(const geometry_msgs::msg::Point &pos, bool &is_new,
                                    geometry_msgs::msg::Point &mean_out, size_t &count_out)
{
    (void)pos;
    is_new = false;
    mean_out = geometry_msgs::msg::Point();
    count_out = 0;
    // TODO: implement association logic
}

void StateMachine::onTimer()
{
    publishState();
}













int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachine>());
    rclcpp::shutdown();
    return 0;
}
