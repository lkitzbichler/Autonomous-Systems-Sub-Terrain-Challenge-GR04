/*
 * Basic waypoint node with command handling and heartbeat publishing.
 * Build + run via your ROS2 launch setup, e.g.
 *   ros2 run basic_waypoint_pkg basic_waypoint_node
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "basic_waypoint_pkg/planner.hpp"
#include "protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "statemachine_pkg/msg/answer.hpp"
#include "statemachine_pkg/msg/command.hpp"

/**
 * @brief Application wrapper that handles incoming statemachine commands and
 *        publishes heartbeat messages.
 */
class BasicWaypointApp {
   public:
    /**
     * @brief Construct the application helper for a given ROS2 node.
     *
     * @param node Shared pointer to the underlying ROS2 node.
     */
    explicit BasicWaypointApp(const rclcpp::Node::SharedPtr& node) : node_(node), planner_(node) {
        // declare parameters
        command_topic_ = node_->declare_parameter<std::string>("command_topic", "statemachine/cmd");
        heartbeat_topic_ = node_->declare_parameter<std::string>("heartbeat_topic", "heartbeat");
        heartbeat_period_sec_ = node_->declare_parameter<double>("heartbeat_period_sec", 1.0);

        // create heartbeat publisher
        pub_heartbeat_ = node_->create_publisher<statemachine_pkg::msg::Answer>(heartbeat_topic_, 10);

        // create command subscriber
        sub_cmd_ = node_->create_subscription<statemachine_pkg::msg::Command>(
            command_topic_, 10, std::bind(&BasicWaypointApp::onCommand, this, std::placeholders::_1));

        // start heartbeat timer
        heartbeat_timer_ = node_->create_wall_timer(std::chrono::duration<double>(heartbeat_period_sec_),
                                                    std::bind(&BasicWaypointApp::publishHeartbeat, this));
    }

   private:
    /**
     * @brief Command handler for statemachine commands.
     * @param msg Incoming command message.
     */
    /**
     * @brief Command handler for statemachine commands.
     *
     * Filters out messages that are null or not addressed to this node, then
     * dispatches to the appropriate handler method based on the command type.
     *
     * @param msg Shared pointer to the incoming command message.
     */
    void onCommand(const statemachine_pkg::msg::Command::SharedPtr msg) {
        // validate message and target
        if (!msg) {
            return;
        }
        if (msg->target != node_->get_name()) {
            return;
        }

        // dispatch based on command
        switch (msg->command) {
            case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::TAKEOFF):
                handleTakeoff(*msg);
                break;
            case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::START):
                handleStart();
                break;
            case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::LAND):
                handleLand(*msg);
                break;
            case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::HOLD):
            case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::RETURN_HOME):
            case static_cast<uint8_t>(statemachine_pkg::protocol::Commands::ABORT):
            default:
                RCLCPP_INFO(node_->get_logger(), "Command ignored (id=%u)", msg->command);
                break;
        }
    }

    /**
     * @brief Execute a normal mission trajectory using YAML waypoints.
     */
    /**
     * @brief Execute a normal mission trajectory using YAML waypoints.
     */
    void handleStart() {
        // plan trajectory from parameters
        mav_trajectory_generation::Trajectory trajectory;
        if (!planner_.planTrajectory(&trajectory)) {
            RCLCPP_WARN(node_->get_logger(), "Failed to plan START trajectory.");
            return;
        }

        // publish trajectory
        planner_.publishTrajectory(trajectory);
        RCLCPP_INFO(node_->get_logger(), "START trajectory published.");
    }

    /**
     * @brief Execute a takeoff trajectory to a target point.
     * @param cmd Incoming command containing the target.
     */
    /**
     * @brief Execute a takeoff trajectory to a target point.
     *
     * @param cmd Incoming command containing the target coordinates.
     */
    void handleTakeoff(const statemachine_pkg::msg::Command& cmd) { handleTargetTrajectory(cmd, "TAKEOFF"); }

    /**
     * @brief Execute a landing trajectory to a target point.
     * @param cmd Incoming command containing the landing target.
     */
    /**
     * @brief Execute a landing trajectory to a target point.
     *
     * @param cmd Incoming command containing the landing target coordinates.
     */
    void handleLand(const statemachine_pkg::msg::Command& cmd) { handleTargetTrajectory(cmd, "LAND"); }

    /**
     * @brief Plan and publish a single-target trajectory (used by TAKEOFF/LAND).
     * @param cmd Incoming command with target payload.
     * @param label Human-readable command label for logs.
     */
    /**
     * @brief Plan and publish a single-target trajectory (used by TAKEOFF/LAND).
     *
     * @param cmd Incoming command with target payload.
     * @param label Human-readable command label for logs.
     */
    void handleTargetTrajectory(const statemachine_pkg::msg::Command& cmd, const char* label) {
        // validate target payload
        if (!cmd.has_target) {
            RCLCPP_WARN(node_->get_logger(), "%s command missing target.", label);
            return;
        }

        // build single-goal waypoint list
        std::vector<double> waypoint_flat_list;
        waypoint_flat_list.reserve(3);
        waypoint_flat_list.push_back(cmd.target_pos.x);
        waypoint_flat_list.push_back(cmd.target_pos.y);
        waypoint_flat_list.push_back(cmd.target_pos.z);

        // plan trajectory to target
        mav_trajectory_generation::Trajectory trajectory;
        if (!planner_.planTrajectoryWithWaypoints(waypoint_flat_list, -1, &trajectory)) {
            RCLCPP_WARN(node_->get_logger(), "Failed to plan %s trajectory.", label);
            return;
        }

        // publish trajectory
        planner_.publishTrajectory(trajectory);
        RCLCPP_INFO(node_->get_logger(), "%s trajectory published.", label);
    }

    /**
     * @brief Publish periodic heartbeat for the statemachine.
     */
    /**
     * @brief Publish periodic heartbeat for the statemachine.
     */
    void publishHeartbeat() {
        // build heartbeat message
        statemachine_pkg::msg::Answer msg;
        msg.node_name = node_->get_name();
        msg.state = static_cast<uint8_t>(statemachine_pkg::protocol::AnswerStates::RUNNING);
        msg.info = "RUNNING";

        // stamp with current time
        const auto now = node_->now();
        msg.timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
        msg.timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);

        // publish
        pub_heartbeat_->publish(msg);
    }

   private:
    rclcpp::Node::SharedPtr node_;
    BasicPlanner planner_;

    rclcpp::Publisher<statemachine_pkg::msg::Answer>::SharedPtr pub_heartbeat_;
    rclcpp::Subscription<statemachine_pkg::msg::Command>::SharedPtr sub_cmd_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;

    std::string command_topic_;
    std::string heartbeat_topic_;
    double heartbeat_period_sec_{1.0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // create node with the expected name for statemachine matching
    auto node = rclcpp::Node::make_shared("planner");

    // attach app logic to the node
    auto app = std::make_shared<BasicWaypointApp>(node);
    (void)app;

    // spin
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
