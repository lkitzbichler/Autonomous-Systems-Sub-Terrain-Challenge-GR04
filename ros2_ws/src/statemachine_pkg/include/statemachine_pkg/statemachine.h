#pragma once

#include "statemachine_pkg/msg/answer.hpp"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

class StateMachine : public rclcpp::Node
{
private: // STATES & COMMANDS
    enum class MissionState : uint8_t {
        WAITING = 0,        // Waiting for ready signals to start mission
        TAKEOFF = 1,        // Takeoff from start position
        TRAVELLING = 2,     // Use basic_waypoint to reach cave entrance
        EXPLORING = 3,      // Autonomous exploration (path planning)
        RETURN_HOME = 4,    // Return to exit/home position
        LAND = 5,           // Land and stop
        DONE = 6,           // Mission finished
        ERROR = 98,         // Error state
        ABORTED = 99        // Manual abort
    };

    enum class Command : uint8_t {
        TAKEOFF = 0,        // Takeoff command
        START = 1,          // Start TRAVVELING
        SWITCH_TO_EXPLORE = 2, // Switch to EXPLORING
        HOLD = 3,           // Hold command -> Stop immediately -> Hover
        RETURN_HOME = 4,    // Return home command
        LAND = 5,           // Land command
        ABORT = 99,         // Stop node 
        NONE = 100          // No command
    };

private: // VARIABLES and STRUCTS
    struct NodeInfo{
        std::string name;
        rclcpp::Time last_heartbeat{0, 0, RCL_ROS_TIME};
        bool is_alive{false};
    };

    struct Lantern{
        int id{0};
        geometry_msgs::msg::Point mean;
        std::vector<geometry_msgs::msg::Point> samples;
        size_t count{0};
    };

    MissionState state_ {MissionState::WAITING};
    rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
    Command last_cmd_ {Command::NONE};

    std::vector<NodeInfo> nodes_;
    std::vector<Lantern> lantern_tracks_;
    std::vector<Eigen::Vector3d> checkpoint_positions_;
    
    bool error_ {false};
    bool reached_waypoint_ {false};
    bool abort_requested_ {false};
    short current_checkpoint_index_ {-1};

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::duration<double> period_alive_{1.0};

private: // SUBSCRIBERS, PUBLISHERS, TIMERS
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_checkpoint_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_viz_path_flight_;

    // Subscribers
    rclcpp::Subscription<statemachine_pkg::msg::Answer>::SharedPtr sub_heartbeat_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_lantern_detections_;

private: // PARAMETERS
    // LISTS
    // Nodes
    std::vector<std::string> node_list_;
    // Checkpoints
    std::vector<double> checkpoint_list_;

    // Timer periods
    double pub_state_loop_sec_;
    double alive_tol_sec_;

    // LANTERNS
    double min_lantern_dist_;    

    // TOPICS
    // State
    std::string topic_state_;               // publish SM state
    std::string topic_cmd_;                 // publish commands for other nodes
    std::string topic_heartbeat_;           // subscribe to check if a node is dead

    // Lanterns
    std::string topic_lantern_detections_;  // subscribe to lantern detections

    // Vizualization
    std::string topic_viz_checkpoint;       // publish checkpoint positions for visualization
    std::string topic_viz_path_flight;      // publish flight path for visualization

    // LOGGING
    std::string lantern_log_path_;
    std::string event_log_path_;

public: //CONSTRUCTOR & METHODS
    StateMachine();
    ~StateMachine();

    void publishState();
    void changeState(MissionState new_state, const std::string &reason);

    void sendCommand(std::string recv_node, Command cmd);
    void handleAnswer(const statemachine_pkg::msg::Answer::SharedPtr msg);

    void onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void associateLantern(const geometry_msgs::msg::Point &pos, bool &is_new, geometry_msgs::msg::Point &mean_out, size_t &count_out);

    void onTimer();

private: // HELPER METHODS

private: // LOGGING
    void logEvent(const std::string &message);
    void logCommand(const std::string &recv_node, Command cmd);
    void logLanternPose(const geometry_msgs::msg::PoseStamped &pose, int id, size_t count);

};
