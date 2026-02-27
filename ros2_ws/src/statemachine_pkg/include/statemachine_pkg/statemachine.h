#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "statemachine_pkg/msg/answer.hpp"
#include "statemachine_pkg/msg/command.hpp"
#include "statemachine_pkg/protocol.hpp"

namespace octomap {
class OcTree;
}

/**
 * @class StateMachine
 * @brief Coordinates the high-level mission state machine for an autonomous UAV.
 *
 * Manages states like TAKEOFF, TRAVELLING, EXPLORING, RETURN_HOME, and LAND.
 * Monitored nodes are checked via heartbeats, and commands are dispatched based on state transitions.
 */
class StateMachine : public rclcpp::Node {
   public:  // STATES & COMMANDS
    using MissionStates = statemachine_pkg::protocol::MissionStates;
    using Commands = statemachine_pkg::protocol::Commands;
    using AnswerStates = statemachine_pkg::protocol::AnswerStates;

   private:  // VARIABLES and STRUCTS
    /**
     * @brief Helper struct to track heartbeat status and state of monitored nodes.
     */
    struct NodeInfo {
        std::string node_name;                            ///< Name of the node in the system.
        rclcpp::Time last_heartbeat{0, 0, RCL_ROS_TIME};  ///< Timestamp of the last received heartbeat.
        bool is_alive{false};   ///< Flag indicating if the node is currently considered alive.
        bool was_alive{false};  ///< Flag indicating if the node has been alive at least once.
        AnswerStates last_state{AnswerStates::UNKNOWN};  ///< Last reported state of the node.
    };

    MissionStates current_mission_state_{MissionStates::WAITING};  ///< Current high-level mission state.
    rclcpp::Time state_entry_timestamp_{
        0, 0, RCL_ROS_TIME};  ///< Timestamp when the current mission state was entered.
    rclcpp::Time last_takeoff_command_timestamp_{
        0, 0, RCL_ROS_TIME};  ///< Timestamp of the last periodic TAKEOFF command dispatch.
    Commands last_sent_command_{Commands::NONE};  ///< Last dispatched command (stored for logging and logic).

    std::vector<NodeInfo>
        monitored_nodes_;  ///< Collection of external nodes to monitor via heartbeat messages.
    std::size_t latest_lantern_count_{
        0};  ///< Most recently reported total count of unique lanterns discovered.
    std::vector<std::size_t>
        lantern_sighting_samples_;  ///< Cumulative sighting counts for each discovered lantern.
    std::vector<Eigen::Vector3d> active_checkpoint_positions_m_;  ///< Current list of waypoints in meters for
                                                                  ///< visualization and navigation.
    std::vector<Eigen::Vector3d> base_checkpoint_positions_m_;    ///< Static waypoints from configuration in
                                                                ///< meters (excludes dynamic takeoff start).

    bool is_checkpoint_reached_{
        false};  ///< Flag triggered when the UAV reaches the current mission waypoint.
    short active_checkpoint_index_{
        -1};  ///< Index of the currently targeted waypoint in active_checkpoint_positions_m_ (-1 if none).
    bool has_takeoff_checkpoint_been_inserted_{
        false};  ///< True if a dynamic 5m-above-start waypoint has been prepended to the mission.
    short active_landing_checkpoint_index_{
        -1};  ///< Index of the dynamically generated landing waypoint (-1 if not yet generated).

    bool has_boot_timeout_been_reported_{
        false};  ///< Prevents repeated log entries for boot timeout in WAITING state.
    bool has_received_current_pose_{
        false};  ///< True if at least one state estimation message has been received.
    bool has_received_octomap_{
        false};  ///< True if at least one octomap message has been received for ground estimation.
    geometry_msgs::msg::Point current_position_m_;  ///< Latest UAV position in meters from state estimate.
    std::string current_pose_frame_id_{
        "world"};  ///< Reference frame ID for the current pose and visualization paths.
    std::vector<geometry_msgs::msg::Point>
        flight_path_points_;  ///< History of UAV positions for line-strip path visualization.
    bool is_planner_exploration_completed_{
        false};  ///< Flag indicating if the path planner node reported exploration is complete.
    bool is_planner_return_home_completed_{
        false};  ///< Flag indicating if the path planner node reported return-home is complete.
    std::shared_ptr<octomap::OcTree>
        latest_octomap_octree_;  ///< Cached octree used for map-based ground estimation during landing.
    rclcpp::TimerBase::SharedPtr periodic_task_timer_;  ///< Timer governing periodic tasks like state
                                                        ///< publishing and heartbeat monitoring.

   private:  // SUBSCRIBERS, PUBLISHERS, TIMERS
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Publisher<statemachine_pkg::msg::Command>::SharedPtr pub_cmd_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_viz_checkpoint_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_viz_path_flight_;

    // Subscribers
    rclcpp::Subscription<statemachine_pkg::msg::Answer>::SharedPtr sub_heartbeat_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_lantern_detections_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_lantern_counts_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_current_state_est_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap_;

   private:  // PARAMETERS
    // LISTS
    // Nodes
    std::vector<std::string> node_list_;  // List of nodes to monitor
    // Checkpoints
    std::vector<double> checkpoint_list_;  // List of checkpoint positions as flat
                                           // list [x1, y1, z1, x2, y2, z2, ...]

    // Timer periods
    double pub_state_loop_sec_;             // Period for publishing state
    double alive_tol_sec_;                  // Time after which a node is considered dead if no
                                            // heartbeat is received
    double boot_timeout_sec_;               // Max time allowed to wait for nodes in WAITING/BOOT
    double takeoff_cmd_retry_sec_;          // Retry period for TAKEOFF command while state
                                            // is TAKEOFF
    double checkpoint_reach_dist_m_;        // Distance threshold for reaching a checkpoint
    double path_sample_dist_m_;             // Min distance between path samples
    double landing_xy_radius_m_;            // XY search radius for local ground estimation
    double landing_xy_radius_max_m_;        // Max XY search radius for adaptive ground
                                            // estimation
    double landing_xy_radius_increment_m_;  // Radius increment for adaptive ground
                                            // estimation
    double landing_clearance_m_;            // Clearance above estimated ground for landing
                                            // target
    double landing_probe_depth_m_;          // Max downward ray distance for ground estimation
    int landing_min_hit_count_;             // Minimum number of downward ray hits required
    double landing_min_hit_fraction_;       // Minimum ratio of ray hits in the sampled
                                            // disk
    bool landing_use_start_fallback_;       // Use start-checkpoint-based fallback if
                                            // map ground estimate fails

    // TOPICS
    // State
    std::string topic_state_;              // publish SM state
    std::string topic_cmd_;                // publish commands for other nodes
    std::string topic_heartbeat_;          // subscribe to check if a node is dead
    std::string topic_current_state_est_;  // subscribe to current state estimate
    std::string topic_octomap_;            // subscribe to octomap map data

    // Lanterns
    std::string topic_lantern_detections_;  // subscribe to lantern detections
    std::string topic_lantern_counts_;      // subscribe to detector counts aligned
                                            // with detections

    // Vizualization
    std::string topic_viz_checkpoint;   // publish checkpoint positions for visualization
    std::string topic_viz_path_flight;  // publish flight path for visualization

    // LOGGING
    std::string lantern_log_path_;  // File path for logging lantern detections
                                    // (CSV format)
    std::string event_log_path_;    // File path for logging state machine events
                                    // (plain text format)
    std::string octomap_log_path_;  // File path for saving the final octomap

    // NODE ROLE INDICES (refer to entries in node_list_)
    int node_controller_index_{-1};  // Controller node index
    int node_waypoint_index_{-1};    // Basic waypoint node index
    int node_planner_index_{-1};     // Path planner node index

   public:  // CONSTRUCTOR & METHODS
    /**
     * @brief Node constructor for StateMachine.
     */
    StateMachine();

    /**
     * @brief Node destructor for StateMachine.
     */
    ~StateMachine();

    /**
     * @brief Formats the current state as a string and publishes it to the /state topic.
     */
    void publishState();

    /**
     * @brief Transitions the mission state, stores state entry time, and notifies via logging.
     * @param target_mission_state The new mission state to enter.
     * @param state_change_reason Human-readable explanation for the transition.
     */
    void changeState(MissionStates target_mission_state, const std::string& state_change_reason);

    /**
     * @brief Dispatches a high-level command to a specific node in the ROS 2 graph.
     * @param receiver_node_name Exact string name of the target node.
     * @param command_enum The instruction to send.
     */
    void sendCommand(std::string receiver_node_name, Commands command_enum);

    /**
     * @brief Dispatches a command along with a specific waypoint target to a node.
     * @param receiver_node_name Exact string name of the target node.
     * @param command_enum The instruction to send.
     * @param target_position_m A 3D coordinate in the world frame for the target.
     */
    void sendCommandWithTarget(const std::string& receiver_node_name, Commands command_enum,
                               const geometry_msgs::msg::Point& target_position_m);

    /**
     * @brief Processes heartbeat responses from external nodes to maintain health status.
     * @param shared_answer_msg Pointer to the incoming protocol message.
     */
    void handleAnswer(const statemachine_pkg::msg::Answer::SharedPtr shared_answer_msg);

    /**
     * @brief Callback function periodically invoked to update lantern detection status.
     * @param pose_array_msg Array containing the current set of lantern poses in the field of view.
     */
    void onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr pose_array_msg);

    /**
     * @brief Updates the confidence counts for each detected lantern.
     * @param int_array_msg Number of sightings reported for each track by the detector.
     */
    void onLanternCounts(const std_msgs::msg::Int32MultiArray::SharedPtr int_array_msg);

    /**
     * @brief Caches the most recent Octomap for landing ground estimation.
     * @param octomap_msg Binary encoded Octomap data.
     */
    void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr octomap_msg);

    /**
     * @brief Receives the current UAV state estimate and updates local position.
     * @param odometry_msg Standard odometry message containing current pose and frame.
     */
    void onCurrentStateEst(const nav_msgs::msg::Odometry::SharedPtr odometry_msg);

    /**
     * @brief Periodic loop callback for state machine logic (frequency set by pub_state_loop_sec_).
     */
    void onTimer();

   private:  // HELPER METHODS
    /**
     * @brief Verifies that all expected nodes in the configuration are reporting heartbeats.
     * Starts the mission transition when all required nodes are healthy.
     */
    void checkHeartbeats();

    /**
     * @brief Computes the distance to the current active waypoint and flags reaching events.
     */
    void checkCheckpoints();

    /**
     * @brief At initialization, attempts to insert a 5m takeoff checkpoint relative to the UAV's current
     * start.
     */
    void tryInsertStartCheckpoint();

    /**
     * @brief Raycasts downward into the current Octomap to find a landing surface.
     * @param calculated_ground_z_m Output variable for the estimated Z-elevation in meters.
     * @return bool True if a valid ground surface was found below the UAV.
     */
    bool estimateGroundHeight(double& calculated_ground_z_m) const;

    /**
     * @brief Generates a safe landing waypoint based on local terrain or start-fallback.
     * @param landing_target_out Output coordinate for the landing maneuver.
     * @return bool True if a valid landing target could be calculated.
     */
    bool prepareLandingCheckpoint(geometry_msgs::msg::Point& landing_target_out);

    /**
     * @brief Handles logical side effects of state transitions and goal reach flags.
     */
    void handleFlagEvents();

    /**
     * @brief Publishes a LINE_STRIP marker illustrating the UAV's flight trajectory.
     */
    void publishPathViz();

    /**
     * @brief Publishes SPHERE markers illustrating mission waypoints.
     */
    void publishCheckpointViz();

    /**
     * @brief Maps node role indices to their configured string names.
     * @param list_index Position in node_list_ parameter.
     * @return std::string Resolved name or empty string if index is invalid.
     */
    std::string nodeNameByIndex(int list_index) const;

   private:  // LOGGING
    /**
     * @brief Outputs an event string to the node's configured text log file.
     * @param event_message Textual description of the system event.
     */
    void logEvent(const std::string& event_message);

    /**
     * @brief Records a sent command for persistence in the event log.
     * @param receiver_node_name Target of the command.
     * @param command_enum The command being dispatched.
     */
    void logCommand(const std::string& receiver_node_name, Commands command_enum);

    /**
     * @brief Generates a persistent CSV entry for a lantern discovery event.
     * @param lantern_pose_stamped Pose data of the lantern.
     * @param instance_id Track ID assigned by the detection package.
     * @param total_detections_count Number of times the tracker has confirmed this lantern.
     */
    void logLanternPose(const geometry_msgs::msg::PoseStamped& lantern_pose_stamped, int instance_id,
                        size_t total_detections_count);

   private:  // STRING HELPERS
    /**
     * @brief Utility to convert MissionStates enum to readable text.
     * @param current_state enum value to convert.
     * @return std::string State name.
     */
    static std::string toString(MissionStates current_state);

    /**
     * @brief Utility to convert Commands enum to readable text.
     * @param active_command enum value to convert.
     * @return std::string Command name.
     */
    static std::string toString(Commands active_command);
};
