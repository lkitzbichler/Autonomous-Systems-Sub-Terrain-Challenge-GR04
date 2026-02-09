#pragma once

#include "statemachine_pkg/protocol.hpp"
#include "statemachine_pkg/msg/answer.hpp"
#include "statemachine_pkg/msg/command.hpp"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace octomap
{
class OcTree;
}

class StateMachine : public rclcpp::Node
{
public: // STATES & COMMANDS
    using MissionStates = statemachine_pkg::protocol::MissionStates;
    using Commands = statemachine_pkg::protocol::Commands;
    using AnswerStates = statemachine_pkg::protocol::AnswerStates;

private: // VARIABLES and STRUCTS
    /// Helper struct to track node heartbeats
    struct NodeInfo{
        std::string name;
        rclcpp::Time last_heartbeat{0, 0, RCL_ROS_TIME};
        bool is_alive{false};
        bool was_alive{false};
        AnswerStates last_state{AnswerStates::UNKNOWN};
    };

    /// Helper struct to track lantern detections
    struct Lantern{
        int id{0};
        geometry_msgs::msg::Point mean;
        std::vector<geometry_msgs::msg::Point> samples;
        size_t count{0};
    };

    MissionStates state_ {MissionStates::WAITING}; // Current mission state
    rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME}; // Time when the current state was entered
    Commands last_cmd_ {Commands::NONE}; // Last command sent (for logging)

    std::vector<NodeInfo> nodes_; // List of nodes to monitor for heartbeats
    std::vector<Lantern> lantern_tracks_; // List of tracked lanterns with their positions and counts
    std::vector<Eigen::Vector3d> checkpoint_positions_; // List of checkpoint positions for visualization
    std::vector<Eigen::Vector3d> checkpoint_positions_base_; // Static checkpoints from params (without takeoff start)
    
    bool checkpoint_reached_ {false}; // Flag to indicate a checkpoint was reached
    short current_checkpoint_index_ {-1}; // Index of the current checkpoint being targeted, -1 if none
    bool start_checkpoint_inserted_{false}; // True once the takeoff checkpoint is inserted at list front
    short landing_checkpoint_index_{-1}; // Index of the active landing checkpoint
    
    bool boot_timeout_reported_ {false}; // Avoid spamming boot-timeout logs
    bool has_current_pose_{false}; // True once current pose has been received
    bool has_octomap_{false}; // True once an octomap has been received
    geometry_msgs::msg::Point current_position_; // Latest position from state estimate
    std::string current_pose_frame_id_{"world"}; // Frame for current pose/path
    std::vector<geometry_msgs::msg::Point> path_points_; // Flight path points for visualization
    bool planner_done_{false}; // Planner DONE flag (used in exploring)
    std::shared_ptr<octomap::OcTree> octree_; // Latest octomap for landing ground estimation
    rclcpp::TimerBase::SharedPtr timer_; // Timer for periodic tasks like state publishing and heartbeat checks

private: // SUBSCRIBERS, PUBLISHERS, TIMERS
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Publisher<statemachine_pkg::msg::Command>::SharedPtr pub_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_checkpoint_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_viz_path_flight_;

    // Subscribers
    rclcpp::Subscription<statemachine_pkg::msg::Answer>::SharedPtr sub_heartbeat_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_lantern_detections_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_current_state_est_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap_;

private: // PARAMETERS
    // LISTS
    // Nodes
    std::vector<std::string> node_list_;    // List of nodes to monitor
    // Checkpoints
    std::vector<double> checkpoint_list_;   // List of checkpoint positions as flat list [x1, y1, z1, x2, y2, z2, ...]

    // Timer periods
    double pub_state_loop_sec_;             // Period for publishing state
    double alive_tol_sec_;                  // Time after which a node is considered dead if no heartbeat is received
    double boot_timeout_sec_;               // Max time allowed to wait for nodes in WAITING/BOOT
    double checkpoint_reach_dist_m_;        // Distance threshold for reaching a checkpoint
    double path_sample_dist_m_;             // Min distance between path samples
    double landing_xy_radius_m_;            // XY search radius for local ground estimation
    double landing_clearance_m_;            // Clearance above estimated ground for landing target
    double landing_probe_depth_m_;          // Max downward ray distance for ground estimation

    // LANTERNS
    double min_lantern_dist_;               // Minimum distance to consider a new lantern detection as the same as an existing track and merge it

    // TOPICS
    // State
    std::string topic_state_;               // publish SM state
    std::string topic_cmd_;                 // publish commands for other nodes
    std::string topic_heartbeat_;           // subscribe to check if a node is dead
    std::string topic_current_state_est_;   // subscribe to current state estimate
    std::string topic_octomap_;             // subscribe to octomap map data

    // Lanterns
    std::string topic_lantern_detections_;  // subscribe to lantern detections

    // Vizualization
    std::string topic_viz_checkpoint;       // publish checkpoint positions for visualization
    std::string topic_viz_path_flight;      // publish flight path for visualization

    // LOGGING
    std::string lantern_log_path_;          // File path for logging lantern detections (CSV format)
    std::string event_log_path_;            // File path for logging state machine events (plain text format)

    // NODE ROLE INDICES (refer to entries in node_list_)
    int node_controller_index_{-1};         // Controller node index
    int node_waypoint_index_{-1};           // Basic waypoint node index
    int node_planner_index_{-1};            // Path planner node index

public: //CONSTRUCTOR & METHODS
    StateMachine();                         // Constructor
    ~StateMachine();                        // Destructor

    void publishState();                    // Publish the current state to the topic
    void changeState(MissionStates new_state, const std::string &reason);   // Change the current state and publish it, with a reason for logging

    void sendCommand(std::string recv_node, Commands cmd);  // Send a command to a specific node, with logging
    void sendCommandWithTarget(const std::string &recv_node, Commands cmd, const geometry_msgs::msg::Point &target);  // Send a command with an optional target position
    void handleAnswer(const statemachine_pkg::msg::Answer::SharedPtr msg);  // Handle incoming answers/heartbeats from nodes to update their alive status

    void onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg);   // Handle lantern detections
    void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg); // Update local octomap cache
    void associateLantern(const geometry_msgs::msg::Point &pos, bool &is_new, geometry_msgs::msg::Point &mean_out, size_t &count_out);  // Associate a new lantern detection with existing tracks or create a new track, returning whether it's new, the mean position, and count of detections
    void onCurrentStateEst(const nav_msgs::msg::Odometry::SharedPtr msg); // Update current position from state estimate

    void onTimer(); // Timer callback for periodic tasks like publishing state and checking node heartbeats

private: // HELPER METHODS
    void checkHeartbeats();                 // Check node heartbeats and mark dead nodes
    void checkCheckpoints();                // Evaluate checkpoint progress
    void tryInsertStartCheckpoint();        // Insert takeoff checkpoint once a valid start pose is available
    bool estimateGroundHeight(double &ground_z_out) const; // Estimate ground height below current UAV position from octomap
    bool prepareLandingCheckpoint(geometry_msgs::msg::Point &target_out); // Build landing target and append/update dynamic landing checkpoint
    void handleFlagEvents();                // React to flag changes (edge-triggered)
    void publishPathViz();                  // Publish flight path visualization
    void publishCheckpointViz();            // Publish checkpoint markers
    std::string nodeNameByIndex(int index) const; // Resolve role index to node name

private: // LOGGING
    void logEvent(const std::string &message);  // Log a state machine event with a message
    void logCommand(const std::string &recv_node, Commands cmd);    // Log a command sent to a node with the command details
    void logLanternPose(const geometry_msgs::msg::PoseStamped &pose, int id, size_t count); // Log a lantern detection with its pose, associated track ID, and count of detections for that track

private: // STRING HELPERS
    static std::string toString(MissionStates state); // Convert mission state enum to string
    static std::string toString(Commands cmd);        // Convert command enum to string

};
