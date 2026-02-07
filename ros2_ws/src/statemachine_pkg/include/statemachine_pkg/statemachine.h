#pragma once

#include <cstddef>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
    struct Lantern{
        int id{0};
        geometry_msgs::msg::Point mean;
        std::vector<geometry_msgs::msg::Point> samples;
        size_t count{0};
    };

    MissionState state_ {MissionState::WAITING};
    Command last_cmd_ {Command::NONE};
    std::vector<Lantern> lantern_tracks_;
    std::vector<Eigen::Vector3d> path_plan_;
    
    bool error_ {false};
    bool reached_waypoint_ {false};
    bool abort_requested_ {false};

private: // SUBSCRIBERS, PUBLISHERS, TIMERS

private: // PARAMETERS


public: //CONSTRUCTOR & METHODS
    StateMachine();

private: // HELPER METHODS

};
