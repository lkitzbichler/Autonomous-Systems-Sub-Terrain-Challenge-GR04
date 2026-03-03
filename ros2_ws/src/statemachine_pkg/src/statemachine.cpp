#include "statemachine_pkg/statemachine.h"

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include "statemachine_pkg/msg/answer.hpp"
#include "statemachine_pkg/msg/command.hpp"

namespace {

/**
 * @brief Computes the 3D Euclidean distance between two points.
 * @param point_a First coordinate.
 * @param point_b Second coordinate.
 * @return double L2 norm of the difference vector.
 */
double calculateEuclideanDistance3D(const geometry_msgs::msg::Point& point_a,
                                    const geometry_msgs::msg::Point& point_b) {
    const double delta_x_m = point_a.x - point_b.x;
    const double delta_y_m = point_a.y - point_b.y;
    const double delta_z_m = point_a.z - point_b.z;
    return std::sqrt(delta_x_m * delta_x_m + delta_y_m * delta_y_m + delta_z_m * delta_z_m);
}

constexpr double kTakeoffHeightM = 5.0;       ///< Fixed takeoff height above start position in meters.
constexpr double kMinStartPosNormM = 1e-3;    ///< Reject near-zero start pose (0,0,0) as invalid (meters).
constexpr double kMinLandingDescentM = 0.05;  ///< Guard against non-descending landing targets (meters).
constexpr std::size_t kRequiredLanternCount =
    5;  ///< Mission target: unique lantern tracks required for completion.

}  // namespace

// #################################################################################################
// Constructor / Destructor
// #################################################################################################

StateMachine::StateMachine() : Node("state_machine_node") {
    RCLCPP_INFO(this->get_logger(), "[][StateMachine]: State Machine Node has been started.");

    // Create and Load Parameters
    // ##################################################################### Lists
    this->checkpoint_list_ =
        declare_parameter<std::vector<double>>("checkpoints.positions", std::vector<double>{});
    this->node_list_ = declare_parameter<std::vector<std::string>>("nodes", std::vector<std::string>{});
    // Timer periods
    this->pub_state_loop_sec_ = declare_parameter<double>("time.pub_state_loop_sec", 1.0);
    this->alive_tol_sec_ = declare_parameter<double>("time.alive_tol_sec", 10.0);
    this->boot_timeout_sec_ = declare_parameter<double>("time.boot_timeout_sec", 60.0);
    this->takeoff_cmd_retry_sec_ = declare_parameter<double>("time.takeoff_cmd_retry_sec", 1.5);
    this->path_sample_dist_m_ = declare_parameter<double>("viz.path_sample_dist_m", 0.2);
    this->checkpoint_reach_dist_m_ = declare_parameter<double>("checkpoints.reach_dist_m", 0.5);
    this->landing_xy_radius_m_ = declare_parameter<double>("landing.xy_radius_m", 1.5);
    this->landing_xy_radius_max_m_ = declare_parameter<double>("landing.xy_radius_max_m", 3.0);
    this->landing_xy_radius_increment_m_ = declare_parameter<double>("landing.xy_radius_increment_m", 0.5);
    this->landing_clearance_m_ = declare_parameter<double>("landing.clearance_m", 0.2);
    this->landing_probe_depth_m_ = declare_parameter<double>("landing.probe_depth_m", 80.0);
    this->landing_min_hit_count_ = declare_parameter<int>("landing.min_hit_count", 3);
    this->landing_min_hit_fraction_ = declare_parameter<double>("landing.min_hit_fraction", 0.05);
    this->landing_use_start_fallback_ = declare_parameter<bool>("landing.use_start_fallback", true);
    // Node role indices (refer to entries in node_list_)
    this->node_controller_index_ = declare_parameter<int>("node_roles.controller_index", -1);
    this->node_waypoint_index_ = declare_parameter<int>("node_roles.waypoint_index", -1);
    this->node_planner_index_ = declare_parameter<int>("node_roles.planner_index", -1);
    // Logging paths
    this->lantern_log_path_ = declare_parameter<std::string>("logging.lantern_log_path", "lanterns_log.csv");
    this->event_log_path_ =
        declare_parameter<std::string>("logging.event_log_path", "statemachine_events.log");
    this->octomap_log_path_ = declare_parameter<std::string>("logging.octomap_log_path", "final_map.bt");

    // TOPIC PARAMETERS
    // ###############################################################################
    this->topic_state_ = declare_parameter<std::string>("topics.topic_state", "statemachine/state");
    this->topic_cmd_ = declare_parameter<std::string>("topics.topic_cmd_", "statemachine/cmd");
    this->topic_heartbeat_ =
        declare_parameter<std::string>("topics.topic_heartbeat", "statemachine/heartbeat");
    this->topic_lantern_detections_ =
        declare_parameter<std::string>("topics.topic_lantern_detections", "detected_lanterns");
    this->topic_lantern_counts_ =
        declare_parameter<std::string>("topics.topic_lantern_counts", "detected_lanterns/counts");
    this->topic_current_state_est_ =
        declare_parameter<std::string>("topics.current_state_est", "current_state_est");
    this->topic_octomap_ = declare_parameter<std::string>("topics.topic_octomap", "octomap_binary");
    this->topic_viz_checkpoint =
        declare_parameter<std::string>("topics.topic_viz_checkpoint", "statemachine/viz/checkpoint");
    this->topic_viz_path_flight =
        declare_parameter<std::string>("topics.topic_viz_path_flight", "statemachine/viz/path_flight");

    // Create Publishers
    // ##############################################################################
    this->pub_state_ = this->create_publisher<std_msgs::msg::String>(topic_state_, 10);
    this->pub_cmd_ = this->create_publisher<statemachine_pkg::msg::Command>(topic_cmd_, 10);
    this->pub_viz_checkpoint_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_viz_checkpoint, 10);
    this->pub_viz_path_flight_ =
        this->create_publisher<visualization_msgs::msg::Marker>(topic_viz_path_flight, 10);

    // Create Subscribers
    // #############################################################################
    this->sub_heartbeat_ = this->create_subscription<statemachine_pkg::msg::Answer>(
        topic_heartbeat_, 10, std::bind(&StateMachine::handleAnswer, this, std::placeholders::_1));
    this->sub_lantern_detections_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        topic_lantern_detections_, 10,
        std::bind(&StateMachine::onLanternDetections, this, std::placeholders::_1));
    this->sub_lantern_counts_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        topic_lantern_counts_, 10, std::bind(&StateMachine::onLanternCounts, this, std::placeholders::_1));
    this->sub_current_state_est_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_current_state_est_, 10,
        std::bind(&StateMachine::onCurrentStateEst, this, std::placeholders::_1));
    this->sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        topic_octomap_, 1, std::bind(&StateMachine::onOctomap, this, std::placeholders::_1));

    // Create Timers
    // ##################################################################################
    const double loop_duration_sec = std::max(0.01, pub_state_loop_sec_);
    periodic_task_timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                                       std::chrono::duration<double>(loop_duration_sec)),
                                                   std::bind(&StateMachine::onTimer, this));

    // Initialize node heartbeat list from config
    // ####################################################
    monitored_nodes_.clear();
    monitored_nodes_.reserve(node_list_.size());
    for (const auto& node_name_from_list : node_list_) {
        if (!node_name_from_list.empty()) {
            monitored_nodes_.push_back(NodeInfo{node_name_from_list});
        }
    }

    // Initialize checkpoint list from config
    // ########################################################
    active_checkpoint_positions_m_.clear();
    for (size_t i = 0; i + 2 < checkpoint_list_.size(); i += 3) {
        active_checkpoint_positions_m_.emplace_back(checkpoint_list_[i], checkpoint_list_[i + 1],
                                                    checkpoint_list_[i + 2]);
    }
    base_checkpoint_positions_m_ =
        active_checkpoint_positions_m_;    // Store static list before runtime insertion
    state_entry_timestamp_ = this->now();  // Start WAITING timeout from node startup time
}

StateMachine::~StateMachine() {
    RCLCPP_INFO(this->get_logger(), "State Machine Node is shutting down.");
    this->logEvent("[SHUTDOWN]: State Machine Node is shutting down.");
}

// #################################################################################################
// State & Command Publishing
// #################################################################################################

/**
 * @brief Publish the current state as a string message on the state topic.
 */
void StateMachine::publishState() {
    std_msgs::msg::String state_msg;                    // Message container for the current state
    state_msg.data = toString(current_mission_state_);  // Convert enum to string for logging/monitoring
    pub_state_->publish(state_msg);                     // Publish the message
}

/**
 * @brief Switch to a new mission state, store entry time, and publish update.
 * @param target_mission_state Target mission state.
 * @param state_change_reason Human-readable reason for logging.
 */
void StateMachine::changeState(MissionStates target_mission_state, const std::string& state_change_reason) {
    // Step 1: Ignore if state doesn't change
    if (current_mission_state_ == target_mission_state) {
        return;
    }

    // Step 2: Update state and entry time
    current_mission_state_ = target_mission_state;
    state_entry_timestamp_ = this->now();

    // Step 3: Reset boot-timeout log flag when leaving/entering WAITING
    if (current_mission_state_ != MissionStates::WAITING) {
        has_boot_timeout_been_reported_ = false;
    }
    if (current_mission_state_ != MissionStates::TAKEOFF) {
        last_takeoff_command_timestamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    // Step 4: Log and publish new state
    logEvent("[State] " + toString(current_mission_state_) + " (" + state_change_reason + ")");
    publishState();

    // Step 5: Dispatch commands for the new state
    const std::string node_controller_name = nodeNameByIndex(node_controller_index_);
    const std::string node_waypoint_name = nodeNameByIndex(node_waypoint_index_);
    const std::string node_planner_name = nodeNameByIndex(node_planner_index_);

    switch (current_mission_state_) {
        case MissionStates::WAITING:
            if (!node_controller_name.empty()) sendCommand(node_controller_name, Commands::HOLD);
            break;
        case MissionStates::TAKEOFF:
            if (!node_controller_name.empty()) sendCommand(node_controller_name, Commands::START);
            if (!node_waypoint_name.empty() && !active_checkpoint_positions_m_.empty()) {
                geometry_msgs::msg::Point target_pos;
                target_pos.x = active_checkpoint_positions_m_.front().x();
                target_pos.y = active_checkpoint_positions_m_.front().y();
                target_pos.z = active_checkpoint_positions_m_.front().z();
                sendCommandWithTarget(node_waypoint_name, Commands::TAKEOFF, target_pos);
                last_takeoff_command_timestamp_ = this->now();
            } else if (!node_waypoint_name.empty()) {
                logEvent("[TAKEOFF] no checkpoint available for takeoff target");
            }
            break;
        case MissionStates::TRAVELLING:
            if (!node_controller_name.empty()) sendCommand(node_controller_name, Commands::START);
            if (!node_waypoint_name.empty()) sendCommand(node_waypoint_name, Commands::START);
            break;
        case MissionStates::EXPLORING:
            is_planner_exploration_completed_ = false;
            is_planner_return_home_completed_ = false;
            if (!node_controller_name.empty()) sendCommand(node_controller_name, Commands::START);
            if (!node_planner_name.empty()) sendCommand(node_planner_name, Commands::START);
            break;
        case MissionStates::RETURN_HOME:
            is_planner_return_home_completed_ = false;
            if (!node_controller_name.empty()) sendCommand(node_controller_name, Commands::START);
            if (!node_planner_name.empty()) {
                if (has_takeoff_checkpoint_been_inserted_ && !active_checkpoint_positions_m_.empty()) {
                    geometry_msgs::msg::Point target_pos;
                    target_pos.x = active_checkpoint_positions_m_.front().x();
                    target_pos.y = active_checkpoint_positions_m_.front().y();
                    target_pos.z = active_checkpoint_positions_m_.front().z();
                    sendCommandWithTarget(node_planner_name, Commands::RETURN_HOME, target_pos);
                } else {
                    sendCommand(node_planner_name, Commands::RETURN_HOME);
                }
            }
            break;
        case MissionStates::LAND:
            active_landing_checkpoint_index_ = -1;  // Force fresh landing target creation on LAND entry
            if (!node_controller_name.empty()) sendCommand(node_controller_name, Commands::START);
            if (!node_planner_name.empty()) sendCommand(node_planner_name, Commands::HOLD);
            if (!node_waypoint_name.empty()) {
                geometry_msgs::msg::Point target_pos;
                if (prepareLandingCheckpoint(target_pos)) {
                    sendCommandWithTarget(node_waypoint_name, Commands::LAND, target_pos);
                } else {
                    logEvent("[LAND] ground estimation failed, landing command not sent");
                    RCLCPP_WARN(this->get_logger(),
                                "[LAND] ground estimation failed, landing command not sent");
                }
            }
            break;
        case MissionStates::DONE:
            if (latest_octomap_octree_) {
                latest_octomap_octree_->writeBinary(octomap_log_path_);
                logEvent("[Map] Saved octomap to " + octomap_log_path_);
                RCLCPP_INFO(this->get_logger(), "Saved octomap to %s", octomap_log_path_.c_str());
            }
            [[fallthrough]];
        case MissionStates::ERROR:
        case MissionStates::ABORTED:
            if (!node_controller_name.empty()) sendCommand(node_controller_name, Commands::HOLD);
            if (!node_waypoint_name.empty()) sendCommand(node_waypoint_name, Commands::HOLD);
            if (!node_planner_name.empty()) sendCommand(node_planner_name, Commands::HOLD);
            break;
    }
}

/**
 * @brief Send a command to a specific node via the shared command topic.
 * @param receiver_node_name Target node name.
 * @param command_enum Command enum to send.
 */
void StateMachine::sendCommand(std::string receiver_node_name, Commands command_enum) {
    statemachine_pkg::msg::Command cmd_msg;                // Command container for one target node
    cmd_msg.target = receiver_node_name;                   // Target node name
    cmd_msg.command = static_cast<uint8_t>(command_enum);  // Encode enum to message field
    cmd_msg.has_target = false;                            // No target by default
    const auto current_node_time = this->now();            // Timestamp for tracing/debugging
    cmd_msg.stamp.sec = static_cast<int32_t>(current_node_time.nanoseconds() / 1000000000LL);
    cmd_msg.stamp.nanosec = static_cast<uint32_t>(current_node_time.nanoseconds() % 1000000000LL);

    pub_cmd_->publish(cmd_msg);                    // Publish on shared command topic
    this->last_sent_command_ = command_enum;       // Store last command for potential state-based logic
    logCommand(receiver_node_name, command_enum);  // Persist command in event log
}

/**
 * @brief Send a command to a specific node with a target position.
 * @param receiver_node_name Target node name.
 * @param command_enum Command enum to send.
 * @param target_position_m Target position for the command in meters.
 */
void StateMachine::sendCommandWithTarget(const std::string& receiver_node_name, Commands command_enum,
                                         const geometry_msgs::msg::Point& target_position_m) {
    statemachine_pkg::msg::Command cmd_msg;                // Command container for one target node
    cmd_msg.target = receiver_node_name;                   // Target node name
    cmd_msg.command = static_cast<uint8_t>(command_enum);  // Encode enum to message field
    cmd_msg.has_target = true;                             // Explicit target payload
    cmd_msg.target_pos = target_position_m;                // Target position
    const auto current_node_time = this->now();            // Timestamp for tracing/debugging
    cmd_msg.stamp.sec = static_cast<int32_t>(current_node_time.nanoseconds() / 1000000000LL);
    cmd_msg.stamp.nanosec = static_cast<uint32_t>(current_node_time.nanoseconds() % 1000000000LL);

    pub_cmd_->publish(cmd_msg);                    // Publish on shared command topic
    this->last_sent_command_ = command_enum;       // Store last command for potential state-based logic
    logCommand(receiver_node_name, command_enum);  // Persist command in event log
}

// #################################################################################################
// Heartbeat / Answers
// #################################################################################################

/**
 * @brief Processes heartbeat responses from external nodes to maintain health status.
 * @param shared_answer_msg Pointer to the incoming protocol message.
 */
void StateMachine::handleAnswer(const statemachine_pkg::msg::Answer::SharedPtr shared_answer_msg) {
    // Step 1: Validate message
    if (!shared_answer_msg) {
        return;
    }
    if (shared_answer_msg->node_name.empty()) {
        logEvent("[Answer] missing node_name");
        return;
    }

    const std::string planner_node_name = nodeNameByIndex(node_planner_index_);
    const bool is_planner_node_msg =
        !planner_node_name.empty() && shared_answer_msg->node_name == planner_node_name;

    // Step 2: Convert status code to enum (RUNNING-only; DONE accepted from planner only)
    AnswerStates reported_status = AnswerStates::UNKNOWN;
    if (shared_answer_msg->state == static_cast<uint8_t>(AnswerStates::RUNNING)) {
        reported_status = AnswerStates::RUNNING;
    } else if (shared_answer_msg->state == static_cast<uint8_t>(AnswerStates::DONE)) {
        if (is_planner_node_msg) {
            reported_status = AnswerStates::DONE;
        }
    }

    auto convertStatusToString = [](AnswerStates status_enum) -> std::string {
        switch (status_enum) {
            case AnswerStates::RUNNING:
                return "RUNNING";
            case AnswerStates::DONE:
                return "DONE";
            case AnswerStates::UNKNOWN:
                return "UNKNOWN";
        }
        return "UNKNOWN";
    };

    // Step 3: Update heartbeat table (create entry if needed)
    auto node_it = std::find_if(
        monitored_nodes_.begin(), monitored_nodes_.end(),
        [&](const NodeInfo& node_info) { return node_info.node_name == shared_answer_msg->node_name; });
    if (node_it == monitored_nodes_.end()) {
        monitored_nodes_.push_back(NodeInfo{shared_answer_msg->node_name});
        node_it = std::prev(monitored_nodes_.end());
    }

    if (shared_answer_msg->timestamp.sec == 0 && shared_answer_msg->timestamp.nanosec == 0) {
        node_it->last_heartbeat = this->now();
    } else {
        node_it->last_heartbeat = rclcpp::Time(shared_answer_msg->timestamp);
    }
    node_it->is_alive = true;

    // Step 4: Log only on status change (avoid log spam)
    if (node_it->last_state != reported_status) {
        logEvent("[Answer] " + shared_answer_msg->node_name + " -> " +
                 convertStatusToString(reported_status));
        node_it->last_state = reported_status;
    }

    // Step 5: Log first time a node comes online
    if (!node_it->was_alive) {
        node_it->was_alive = true;
        logEvent("[Node] " + shared_answer_msg->node_name + " ist online");
        RCLCPP_INFO(this->get_logger(), "[Node] %s ist online", shared_answer_msg->node_name.c_str());
    }

    // Step 6: Decode planner DONE info codes explicitly (no generic DONE flag).
    if (reported_status == AnswerStates::DONE && is_planner_node_msg) {
        if (shared_answer_msg->info == "DONE_EXPLORATION_COMPLETE") {
            is_planner_exploration_completed_ = true;
            logEvent("[Planner] DONE_EXPLORATION_COMPLETE");
        } else if (shared_answer_msg->info == "DONE_RETURN_HOME_REACHED") {
            is_planner_return_home_completed_ = true;
            logEvent("[Planner] DONE_RETURN_HOME_REACHED");
        } else {
            logEvent("[Planner] DONE with unknown info: " + shared_answer_msg->info);
        }
    }
}

/**
 * @brief Caches the most recent Octomap for landing ground estimation.
 * @param octomap_msg Binary encoded Octomap data.
 */
void StateMachine::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr octomap_msg) {
    // Step 1: Validate input
    if (!octomap_msg) {
        return;
    }
    if (!octomap_msg->binary) {
        RCLCPP_WARN(this->get_logger(), "Received non-binary octomap on '%s', expected binary map.",
                    topic_octomap_.c_str());
        return;
    }

    // Step 2: Convert message to OcTree
    octomap::AbstractOcTree* raw_octomap_tree = octomap_msgs::binaryMsgToMap(*octomap_msg);
    if (!raw_octomap_tree) {
        RCLCPP_WARN(this->get_logger(), "Failed to convert octomap message.");
        return;
    }

    auto* concrete_octomap_tree = dynamic_cast<octomap::OcTree*>(raw_octomap_tree);
    if (!concrete_octomap_tree) {
        delete raw_octomap_tree;
        RCLCPP_WARN(this->get_logger(), "Octomap conversion yielded unsupported tree type.");
        return;
    }

    // Step 3: Store map
    latest_octomap_octree_.reset(concrete_octomap_tree);
    has_received_octomap_ = true;
}

// #################################################################################################
// Lanterns
// #################################################################################################

/**
 * @brief Callback function periodically invoked to update lantern detection status.
 * @param pose_array_msg Array containing the current set of lantern poses in the field of view.
 */
void StateMachine::onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr pose_array_msg) {
    if (!pose_array_msg) {
        return;
    }

    if (current_mission_state_ != MissionStates::EXPLORING) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Ignoring lantern detections outside EXPLORING state");
        return;
    }

    latest_lantern_count_ = pose_array_msg->poses.size();

    // Why: Keep the detector topic as the only source of truth for lantern identity/count.
    if (latest_lantern_count_ == 0) {
        logEvent("[Lantern]: empty pose array received");
    }

    for (size_t i = 0; i < pose_array_msg->poses.size(); ++i) {
        geometry_msgs::msg::PoseStamped lantern_pose_stamped;
        lantern_pose_stamped.header = pose_array_msg->header;
        lantern_pose_stamped.pose.position = pose_array_msg->poses[i].position;
        lantern_pose_stamped.pose.orientation.w = 1.0;

        const int lantern_instance_id = static_cast<int>(i) + 1;
        const size_t sighting_count =
            (i < lantern_sighting_samples_.size()) ? lantern_sighting_samples_[i] : 1U;
        logLanternPose(lantern_pose_stamped, lantern_instance_id, sighting_count);
        logEvent("[Lantern] detector reports lantern " + std::to_string(lantern_instance_id) + " " +
                 std::to_string(sighting_count) + " times");
    }
}

/**
 * @brief Updates the confidence counts for each detected lantern.
 * @param int_array_msg Number of sightings reported for each track by the detector.
 */
void StateMachine::onLanternCounts(const std_msgs::msg::Int32MultiArray::SharedPtr int_array_msg) {
    if (!int_array_msg) {
        return;
    }

    if (current_mission_state_ != MissionStates::EXPLORING) {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "Ignoring lantern counts outside EXPLORING state");
        return;
    }

    lantern_sighting_samples_.clear();
    lantern_sighting_samples_.reserve(int_array_msg->data.size());
    for (const auto sighting_value : int_array_msg->data) {
        // Why: Clamp invalid negatives to keep logging/state checks robust against malformed input.
        lantern_sighting_samples_.push_back(sighting_value > 0 ? static_cast<std::size_t>(sighting_value)
                                                               : 0U);
    }
}

// #################################################################################################
// Periodic Checks / Events
// #################################################################################################

/**
 * @brief Periodic loop callback for state machine logic (frequency set by pub_state_loop_sec_).
 */
void StateMachine::onTimer() {
    publishState();
    checkHeartbeats();
    checkCheckpoints();
    handleFlagEvents();

    // Step 5: Retry TAKEOFF command while waiting for first successful takeoff trajectory.
    if (current_mission_state_ == MissionStates::TAKEOFF) {
        const std::string node_waypoint_name = nodeNameByIndex(node_waypoint_index_);
        if (!node_waypoint_name.empty() && !active_checkpoint_positions_m_.empty()) {
            const auto current_node_time = this->now();
            const double command_retry_interval_sec = std::max(0.2, takeoff_cmd_retry_sec_);
            const bool is_first_retry_attempt = last_takeoff_command_timestamp_.nanoseconds() == 0;
            const bool is_retry_timer_due =
                !is_first_retry_attempt &&
                (current_node_time - last_takeoff_command_timestamp_).seconds() >= command_retry_interval_sec;
            if (is_first_retry_attempt || is_retry_timer_due) {
                geometry_msgs::msg::Point target_pos;
                target_pos.x = active_checkpoint_positions_m_.front().x();
                target_pos.y = active_checkpoint_positions_m_.front().y();
                target_pos.z = active_checkpoint_positions_m_.front().z();
                sendCommandWithTarget(node_waypoint_name, Commands::TAKEOFF, target_pos);
                last_takeoff_command_timestamp_ = current_node_time;
                logEvent("[TAKEOFF] retry command sent");
            }
        }
    }

    // Step 6: Retry LAND target creation while waiting for octomap/current pose
    if (current_mission_state_ == MissionStates::LAND && active_landing_checkpoint_index_ < 0) {
        const std::string node_waypoint_name = nodeNameByIndex(node_waypoint_index_);
        if (!node_waypoint_name.empty()) {
            geometry_msgs::msg::Point landing_target_pos;
            if (prepareLandingCheckpoint(landing_target_pos)) {
                sendCommandWithTarget(node_waypoint_name, Commands::LAND, landing_target_pos);
            }
        }
    }

    publishPathViz();
    publishCheckpointViz();
}

/**
 * @brief Verifies that all expected nodes in the configuration are reporting heartbeats.
 */
void StateMachine::checkHeartbeats() {
    const auto current_node_time = this->now();
    const auto active_graph_node_names = this->get_node_names();

    // Step 1: Update alive flags based on last heartbeat
    for (auto& node_info : monitored_nodes_) {
        // octomap_server is an external node without our custom heartbeat publisher.
        // For this node we treat ROS graph presence as "alive".
        if (node_info.node_name == "octomap_server") {
            const auto node_name_it = std::find(active_graph_node_names.begin(),
                                                active_graph_node_names.end(), node_info.node_name);
            node_info.is_alive = (node_name_it != active_graph_node_names.end());
            if (node_info.is_alive) {
                node_info.last_heartbeat = current_node_time;
            }
            continue;
        }

        if (node_info.last_heartbeat.nanoseconds() == 0) {
            node_info.is_alive = false;
        } else {
            const double heartbeat_elapsed_time_sec =
                (current_node_time - node_info.last_heartbeat).seconds();
            node_info.is_alive = heartbeat_elapsed_time_sec <= alive_tol_sec_;
        }
    }

    // Step 2: If all nodes are alive, start mission from WAITING
    if (current_mission_state_ == MissionStates::WAITING) {
        const bool are_all_monitored_nodes_alive =
            std::all_of(monitored_nodes_.begin(), monitored_nodes_.end(), [](const NodeInfo& n) {
                // octomap_server is external infrastructure and does not publish our
                // custom heartbeat. Do not block mission startup on its ROS-graph
                // presence.
                if (n.node_name == "octomap_server") {
                    return true;
                }
                return n.is_alive;
            });
        if (are_all_monitored_nodes_alive && has_takeoff_checkpoint_been_inserted_) {
            changeState(MissionStates::TAKEOFF, "all nodes online");
        }
    }

    // Step 3: If we are waiting too long, report who is missing (once)
    const double state_elapsed_time_sec = (current_node_time - state_entry_timestamp_).seconds();
    if (current_mission_state_ == MissionStates::WAITING && state_elapsed_time_sec >= boot_timeout_sec_ &&
        !has_boot_timeout_been_reported_) {
        std::string missing_nodes_list;
        for (const auto& node_info : monitored_nodes_) {
            if (!node_info.is_alive) {
                if (!missing_nodes_list.empty()) {
                    missing_nodes_list += ", ";
                }
                missing_nodes_list += node_info.node_name;
            }
        }
        if (missing_nodes_list.empty()) {
            logEvent("[Heartbeat] boot timeout reached but no missing nodes detected");
            RCLCPP_INFO(this->get_logger(), "[Heartbeat] boot timeout reached but no missing nodes detected");
        } else {
            logEvent("[Heartbeat] boot timeout, missing: " + missing_nodes_list);
            RCLCPP_INFO(this->get_logger(), "[Heartbeat] boot timeout, missing: %s",
                        missing_nodes_list.c_str());
        }
        has_boot_timeout_been_reported_ = true;
    }
}

/**
 * @brief Computes the distance to the current active waypoint and flags reaching events.
 */
void StateMachine::checkCheckpoints() {
    // Step 1: Ensure checkpoints exist
    if (active_checkpoint_positions_m_.empty()) {
        return;
    }

    // Step 2: Ensure we have an active checkpoint index
    if (active_checkpoint_index_ < 0) {
        active_checkpoint_index_ = 0;
        return;
    }

    // Step 3: Ensure current pose is known
    if (!has_received_current_pose_) {
        return;
    }

    // Step 4: Guard against out-of-range index
    if (active_checkpoint_index_ >= static_cast<short>(active_checkpoint_positions_m_.size())) {
        return;
    }

    // Step 5: Compute distance to current checkpoint
    const auto& current_target_waypoint =
        active_checkpoint_positions_m_[static_cast<size_t>(active_checkpoint_index_)];
    const double delta_x_m = current_position_m_.x - current_target_waypoint.x();
    const double delta_y_m = current_position_m_.y - current_target_waypoint.y();
    const double delta_z_m = current_position_m_.z - current_target_waypoint.z();
    const double distance_to_waypoint_m =
        std::sqrt(delta_x_m * delta_x_m + delta_y_m * delta_y_m + delta_z_m * delta_z_m);

    // Step 6: If reached, advance index and raise event flag
    if (distance_to_waypoint_m <= checkpoint_reach_dist_m_) {
        is_checkpoint_reached_ = true;
        active_checkpoint_index_++;
    }
}

/**
 * @brief Maps node role indices to their configured string names.
 * @param list_index Position in node_list_ parameter.
 * @return std::string Resolved name or empty string if index is invalid.
 */
std::string StateMachine::nodeNameByIndex(int list_index) const {
    if (list_index < 0 || static_cast<size_t>(list_index) >= node_list_.size()) {
        return std::string();
    }
    return node_list_[static_cast<size_t>(list_index)];
}

/**
 * @brief Handles logical side effects of state transitions and goal reach flags.
 */
void StateMachine::handleFlagEvents() {
    // Step 1: React to checkpoint reached event
    if (is_checkpoint_reached_) {
        const int reached_checkpoint_id = static_cast<int>(active_checkpoint_index_) - 1;
        logEvent("[Event] checkpoint reached index=" + std::to_string(reached_checkpoint_id));

        // Step 2: State transitions based on checkpoint index
        if (current_mission_state_ == MissionStates::TAKEOFF && reached_checkpoint_id == 0) {
            changeState(MissionStates::TRAVELLING, "checkpoint 0 reached");
        } else if (current_mission_state_ == MissionStates::TRAVELLING && reached_checkpoint_id == 1) {
            changeState(MissionStates::EXPLORING, "checkpoint 1 reached");
        } else if (current_mission_state_ == MissionStates::LAND &&
                   reached_checkpoint_id == active_landing_checkpoint_index_) {
            changeState(MissionStates::DONE, "landing checkpoint reached");
        }

        // Step 2: Reset flag to allow next checkpoint event
        is_checkpoint_reached_ = false;
    }

    // Step 3: Mission target reached -> return home immediately.
    if (current_mission_state_ == MissionStates::EXPLORING) {
        if (latest_lantern_count_ >= kRequiredLanternCount) {
            changeState(MissionStates::RETURN_HOME, "lantern goal reached");
        }
    }

    // Step 4: Start landing after planner confirms return-home, or when home
    // target is physically reached.
    if (current_mission_state_ == MissionStates::RETURN_HOME && is_planner_return_home_completed_) {
        changeState(MissionStates::LAND, "planner reported return-home complete");
    } else if (current_mission_state_ == MissionStates::RETURN_HOME && has_received_current_pose_ &&
               has_takeoff_checkpoint_been_inserted_ && !active_checkpoint_positions_m_.empty()) {
        const auto& home_waypoint_pos = active_checkpoint_positions_m_.front();
        geometry_msgs::msg::Point home_target_pos;
        home_target_pos.x = home_waypoint_pos.x();
        home_target_pos.y = home_waypoint_pos.y();
        home_target_pos.z = home_waypoint_pos.z();
        if (calculateEuclideanDistance3D(current_position_m_, home_target_pos) <= checkpoint_reach_dist_m_) {
            changeState(MissionStates::LAND, "return-home target reached");
        }
    }
}

// #################################################################################################
// State Estimate / Visualization
// #################################################################################################

/**
 * @brief Receives the current UAV state estimate and updates local position.
 * @param odometry_msg Standard odometry message containing current pose and frame.
 */
void StateMachine::onCurrentStateEst(const nav_msgs::msg::Odometry::SharedPtr odometry_msg) {
    if (!odometry_msg) {
        return;
    }

    current_position_m_ = odometry_msg->pose.pose.position;
    has_received_current_pose_ = true;
    if (!odometry_msg->header.frame_id.empty()) {
        current_pose_frame_id_ = odometry_msg->header.frame_id;
    }

    // Step 1: Try to insert takeoff checkpoint once a valid pose is available
    tryInsertStartCheckpoint();

    // Step 2: Append path point if far enough from last point
    if (flight_path_points_.empty()) {
        flight_path_points_.push_back(current_position_m_);
        return;
    }
    const auto& last_path_point = flight_path_points_.back();
    const double delta_x_m = current_position_m_.x - last_path_point.x;
    const double delta_y_m = current_position_m_.y - last_path_point.y;
    const double delta_z_m = current_position_m_.z - last_path_point.z;
    const double accumulated_distance_m =
        std::sqrt(delta_x_m * delta_x_m + delta_y_m * delta_y_m + delta_z_m * delta_z_m);
    if (accumulated_distance_m >= path_sample_dist_m_) {
        flight_path_points_.push_back(current_position_m_);
    }
}

/**
 * @brief Publishes a LINE_STRIP marker illustrating the UAV's flight trajectory.
 */
void StateMachine::publishPathViz() {
    if (flight_path_points_.size() < 2) {
        return;
    }

    visualization_msgs::msg::Marker trajectory_line_marker;
    trajectory_line_marker.header.frame_id = current_pose_frame_id_;
    trajectory_line_marker.header.stamp = this->now();
    trajectory_line_marker.ns = "flight_path";
    trajectory_line_marker.id = 0;
    trajectory_line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_line_marker.action = visualization_msgs::msg::Marker::ADD;
    trajectory_line_marker.scale.x = 0.08;  // Line width
    trajectory_line_marker.color.r = 0.1f;
    trajectory_line_marker.color.g = 0.9f;
    trajectory_line_marker.color.b = 0.2f;
    trajectory_line_marker.color.a = 1.0f;
    trajectory_line_marker.points = flight_path_points_;

    pub_viz_path_flight_->publish(trajectory_line_marker);
}

/**
 * @brief Publishes SPHERE markers illustrating mission waypoints.
 */
void StateMachine::publishCheckpointViz() {
    if (active_checkpoint_positions_m_.empty()) {
        return;
    }

    visualization_msgs::msg::MarkerArray checkpoint_markers;
    checkpoint_markers.markers.reserve(active_checkpoint_positions_m_.size());

    for (size_t waypoint_idx = 0; waypoint_idx < active_checkpoint_positions_m_.size(); ++waypoint_idx) {
        visualization_msgs::msg::Marker waypoint_marker;
        waypoint_marker.header.frame_id = current_pose_frame_id_;
        waypoint_marker.header.stamp = this->now();
        waypoint_marker.ns = "checkpoints";
        waypoint_marker.id = static_cast<int>(waypoint_idx);
        waypoint_marker.type = visualization_msgs::msg::Marker::SPHERE;
        waypoint_marker.action = visualization_msgs::msg::Marker::ADD;
        waypoint_marker.scale.x = 0.5;
        waypoint_marker.scale.y = 0.5;
        waypoint_marker.scale.z = 0.5;
        waypoint_marker.color.r = 0.9f;
        waypoint_marker.color.g = 0.2f;
        waypoint_marker.color.b = 0.2f;
        waypoint_marker.color.a = 1.0f;
        waypoint_marker.pose.orientation.w = 1.0;
        waypoint_marker.pose.position.x = active_checkpoint_positions_m_[waypoint_idx].x();
        waypoint_marker.pose.position.y = active_checkpoint_positions_m_[waypoint_idx].y();
        waypoint_marker.pose.position.z = active_checkpoint_positions_m_[waypoint_idx].z();
        checkpoint_markers.markers.push_back(waypoint_marker);
    }

    pub_viz_checkpoint_->publish(checkpoint_markers);
}

// #################################################################################################
// Checkpoint Helpers
// #################################################################################################

/**
 * @brief At initialization, attempts to insert a 5m takeoff checkpoint relative to the UAV's current start.
 */
void StateMachine::tryInsertStartCheckpoint() {
    // Step 1: Insert only once
    if (has_takeoff_checkpoint_been_inserted_) {
        return;
    }

    // Step 2: Require a valid current pose
    if (!has_received_current_pose_) {
        return;
    }

    // Step 3: Reject zero pose (simulation not running or uninitialized)
    const double trajectory_origin_norm_m = std::sqrt(current_position_m_.x * current_position_m_.x +
                                                      current_position_m_.y * current_position_m_.y +
                                                      current_position_m_.z * current_position_m_.z);
    if (trajectory_origin_norm_m < kMinStartPosNormM) {
        return;
    }

    // Step 4: Build start checkpoint with fixed takeoff height
    const Eigen::Vector3d takeoff_waypoint_pos_m(current_position_m_.x, current_position_m_.y,
                                                 current_position_m_.z + kTakeoffHeightM);

    // Step 5: Rebuild list with start checkpoint at front
    active_checkpoint_positions_m_.clear();
    active_checkpoint_positions_m_.reserve(base_checkpoint_positions_m_.size() + 1);
    active_checkpoint_positions_m_.push_back(takeoff_waypoint_pos_m);
    active_checkpoint_positions_m_.insert(active_checkpoint_positions_m_.end(),
                                          base_checkpoint_positions_m_.begin(),
                                          base_checkpoint_positions_m_.end());

    // Step 6: Reset checkpoint index to start from the new first checkpoint
    active_checkpoint_index_ = -1;
    has_takeoff_checkpoint_been_inserted_ = true;
    logEvent("[Checkpoint] inserted takeoff checkpoint at start position");
}

/**
 * @brief Raycasts downward into the current Octomap to find a landing surface.
 * @param calculated_ground_z_m Output variable for the estimated Z-elevation in meters.
 * @return bool True if a valid ground surface was found below the UAV.
 */
bool StateMachine::estimateGroundHeight(double& calculated_ground_z_m) const {
    // Step 1: Require pose and map
    if (!has_received_current_pose_ || !has_received_octomap_ || !latest_octomap_octree_) {
        return false;
    }

    // Step 2: Prepare local XY grid sampling and downward ray direction
    const double base_search_radius_m = std::max(0.0, landing_xy_radius_m_);
    const double max_search_radius_m = std::max(base_search_radius_m, landing_xy_radius_max_m_);
    const double search_radius_increment_m = std::max(0.05, landing_xy_radius_increment_m_);
    const double sampling_step_m = std::max(0.05, latest_octomap_octree_->getResolution());
    const octomap::point3d ray_direction_down(0.0F, 0.0F, -1.0F);
    const double ray_start_z_m = current_position_m_.z;

    // Step 3: Probe from small to larger radii and accept when hit quality is sufficient.
    for (double current_search_radius_m = base_search_radius_m;
         current_search_radius_m <= max_search_radius_m + 1e-6;
         current_search_radius_m += search_radius_increment_m) {
        bool is_hit_found = false;
        double best_elevation_z_m = -std::numeric_limits<double>::infinity();
        int total_sample_points_count = 0;
        int successful_ray_hits_count = 0;

        for (double offset_x_m = -current_search_radius_m; offset_x_m <= current_search_radius_m + 1e-6;
             offset_x_m += sampling_step_m) {
            for (double offset_y_m = -current_search_radius_m; offset_y_m <= current_search_radius_m + 1e-6;
                 offset_y_m += sampling_step_m) {
                if ((offset_x_m * offset_x_m + offset_y_m * offset_y_m) >
                    (current_search_radius_m * current_search_radius_m)) {
                    continue;
                }

                ++total_sample_points_count;
                const octomap::point3d ray_origin(static_cast<float>(current_position_m_.x + offset_x_m),
                                                  static_cast<float>(current_position_m_.y + offset_y_m),
                                                  static_cast<float>(ray_start_z_m));
                octomap::point3d ray_hit_point;
                const bool is_ray_hit_successful = latest_octomap_octree_->castRay(
                    ray_origin, ray_direction_down, ray_hit_point, true, landing_probe_depth_m_);
                if (!is_ray_hit_successful) {
                    continue;
                }

                const double hit_elevation_z_m = static_cast<double>(ray_hit_point.z());
                if (hit_elevation_z_m >= ray_start_z_m) {
                    continue;
                }

                ++successful_ray_hits_count;
                if (!is_hit_found || hit_elevation_z_m > best_elevation_z_m) {
                    best_elevation_z_m = hit_elevation_z_m;
                    is_hit_found = true;
                }
            }
        }

        if (!is_hit_found) {
            continue;
        }

        const int min_hits_required_absolute = std::max(1, landing_min_hit_count_);
        const double minimum_hit_fraction_ratio = std::clamp(landing_min_hit_fraction_, 0.0, 1.0);
        const int min_hits_required_by_fraction = static_cast<int>(std::ceil(
            minimum_hit_fraction_ratio * static_cast<double>(std::max(1, total_sample_points_count))));
        const int total_min_hits_required =
            std::max(min_hits_required_absolute, min_hits_required_by_fraction);

        if (successful_ray_hits_count >= total_min_hits_required) {
            calculated_ground_z_m = best_elevation_z_m;
            return true;
        }
    }

    // Step 4: No radius produced enough valid downward hits.
    return false;
}

/**
 * @brief Generates a safe landing waypoint based on local terrain or start-fallback.
 * @param landing_target_out Output coordinate for the landing maneuver.
 * @return bool True if a valid landing target could be calculated.
 */
bool StateMachine::prepareLandingCheckpoint(geometry_msgs::msg::Point& landing_target_out) {
    // Step 1: Require current pose and a valid ground estimate
    double estimated_ground_z_m = 0.0;
    if (!has_received_current_pose_) {
        return false;
    }

    const bool is_map_ground_estimation_successful = estimateGroundHeight(estimated_ground_z_m);
    if (!is_map_ground_estimation_successful && landing_use_start_fallback_ &&
        has_takeoff_checkpoint_been_inserted_ && !active_checkpoint_positions_m_.empty()) {
        // Fallback: derive original start ground from inserted takeoff checkpoint
        // (start + kTakeoffHeightM).
        const auto& start_takeoff_waypoint = active_checkpoint_positions_m_.front();
        estimated_ground_z_m = start_takeoff_waypoint.z() - kTakeoffHeightM;
        landing_target_out.x = start_takeoff_waypoint.x();
        landing_target_out.y = start_takeoff_waypoint.y();
        landing_target_out.z = estimated_ground_z_m + landing_clearance_m_;
        logEvent("[LAND] fallback to start checkpoint target");
    } else if (!is_map_ground_estimation_successful) {
        return false;
    } else {
        // Step 2: Build target above local ground
        landing_target_out.x = current_position_m_.x;
        landing_target_out.y = current_position_m_.y;
        landing_target_out.z = estimated_ground_z_m + landing_clearance_m_;
    }

    // Step 3: Ensure this is an actual descent target
    if (landing_target_out.z >= (current_position_m_.z - kMinLandingDescentM)) {
        logEvent("[LAND] target too close or above current altitude");
        return false;
    }

    // Step 4: Append or update dedicated landing checkpoint and activate it
    const Eigen::Vector3d landing_waypoint_pos_m(landing_target_out.x, landing_target_out.y,
                                                 landing_target_out.z);
    if (active_landing_checkpoint_index_ >= 0 &&
        active_landing_checkpoint_index_ < static_cast<short>(active_checkpoint_positions_m_.size())) {
        active_checkpoint_positions_m_[static_cast<size_t>(active_landing_checkpoint_index_)] =
            landing_waypoint_pos_m;
    } else {
        active_checkpoint_positions_m_.push_back(landing_waypoint_pos_m);
        active_landing_checkpoint_index_ = static_cast<short>(active_checkpoint_positions_m_.size() - 1);
    }
    active_checkpoint_index_ = active_landing_checkpoint_index_;
    is_checkpoint_reached_ = false;

    logEvent("[LAND] checkpoint set z=" + std::to_string(landing_target_out.z));
    return true;
}

// #################################################################################################
// Logging / String Helpers
// #################################################################################################

/**
 * @brief Generates a persistent CSV entry for a lantern discovery event.
 * @param lantern_pose_stamped Pose data of the lantern.
 * @param instance_id Track ID assigned by the detection package.
 * @param total_detections_count Number of times the tracker has confirmed this lantern.
 */
void StateMachine::logLanternPose(const geometry_msgs::msg::PoseStamped& lantern_pose_stamped,
                                  int instance_id, size_t total_detections_count) {
    // Step 1: Check if logging is enabled
    if (lantern_log_path_.empty()) {
        return;  // Logging disabled
    }

    // Step 2: Read existing CSV (if any)
    std::vector<std::string> log_file_lines;
    {
        std::ifstream input_file_stream(lantern_log_path_);
        std::string single_line;
        while (input_file_stream.good() && std::getline(input_file_stream, single_line)) {
            if (!single_line.empty()) {
                log_file_lines.push_back(single_line);
            }
        }
    }

    // Step 3: Resolve timestamp from wall/system clock (independent of ROS time)
    const auto system_now_tp = std::chrono::system_clock::now();
    const auto system_now_tt = std::chrono::system_clock::to_time_t(system_now_tp);
    std::tm system_now_tm{};
#ifdef _WIN32
    gmtime_s(&system_now_tm, &system_now_tt);
#else
    gmtime_r(&system_now_tt, &system_now_tm);
#endif

    std::ostringstream date_hour_stream;
    date_hour_stream << std::put_time(&system_now_tm, "%Y/%m/%d");

    // Step 4: Ensure CSV header exists
    const std::string csv_header_row = "id,zeit,username,anzahl,x,y,z";
    if (log_file_lines.empty() || log_file_lines.front() != csv_header_row) {
        log_file_lines.insert(log_file_lines.begin(), csv_header_row);
    }

    // Step 5: Compose/replace the row for (id, date, user)
    const char* environment_user_ptr = std::getenv("USER");
    const std::string username_string = (environment_user_ptr && *environment_user_ptr)
                                            ? std::string(environment_user_ptr)
                                            : std::string("unknown");
    const std::string instance_id_str = std::to_string(instance_id);
    const std::string date_hour_str = date_hour_stream.str();
    const std::string new_csv_row = instance_id_str + "," + date_hour_str + "," + username_string + "," +
                                    std::to_string(total_detections_count) + "," +
                                    std::to_string(lantern_pose_stamped.pose.position.x) + "," +
                                    std::to_string(lantern_pose_stamped.pose.position.y) + "," +
                                    std::to_string(lantern_pose_stamped.pose.position.z);

    bool has_been_updated = false;
    for (size_t line_idx = 1; line_idx < log_file_lines.size(); ++line_idx) {
        const auto& current_row_text = log_file_lines[line_idx];
        const auto first_comma_pos = current_row_text.find(',');
        const auto second_comma_pos = current_row_text.find(',', first_comma_pos + 1);
        const auto third_comma_pos = current_row_text.find(',', second_comma_pos + 1);
        if (first_comma_pos == std::string::npos || second_comma_pos == std::string::npos ||
            third_comma_pos == std::string::npos) {
            continue;
        }
        const std::string existing_row_id = current_row_text.substr(0, first_comma_pos);
        const std::string existing_row_date =
            current_row_text.substr(first_comma_pos + 1, second_comma_pos - first_comma_pos - 1);
        const std::string existing_row_user =
            current_row_text.substr(second_comma_pos + 1, third_comma_pos - second_comma_pos - 1);
        if (existing_row_id == instance_id_str && existing_row_date == date_hour_str &&
            existing_row_user == username_string) {
            log_file_lines[line_idx] = new_csv_row;
            has_been_updated = true;
            break;
        }
    }

    if (!has_been_updated) {
        log_file_lines.push_back(new_csv_row);
    }

    // Step 6: Write CSV back to disk
    std::ofstream output_file_stream(lantern_log_path_, std::ios::trunc);
    if (!output_file_stream) {
        RCLCPP_WARN(this->get_logger(), "Failed to open lantern log file: %s", lantern_log_path_.c_str());
        return;
    }

    for (const auto& log_row : log_file_lines) {
        output_file_stream << log_row << "\n";
    }
}

/**
 * @brief Outputs an event string to the node's configured text log file.
 * @param event_message Textual description of the system event.
 */
void StateMachine::logEvent(const std::string& event_message) {
    if (event_log_path_.empty()) {
        return;  // Logging disabled
    }

    std::ofstream event_log_stream(event_log_path_, std::ios::app);  // Append to log file
    if (!event_log_stream) {
        RCLCPP_WARN(this->get_logger(), "Failed to open event log file: %s", event_log_path_.c_str());
        return;
    }

    const auto system_now_tp = std::chrono::system_clock::now();
    const double timestamp_sec = std::chrono::duration<double>(system_now_tp.time_since_epoch()).count();

    std::ostringstream log_line_formatter;  // Build log line with required prefix
    log_line_formatter << "[" << std::fixed << std::setprecision(3) << timestamp_sec << "]"
                       << "[StateMachine] " << event_message;

    event_log_stream << log_line_formatter.str() << "\n";  // Timestamped log line
}

/**
 * @brief Records a sent command for persistence in the event log.
 * @param receiver_node_name Target of the command.
 * @param command_enum The command being dispatched.
 */
void StateMachine::logCommand(const std::string& receiver_node_name, Commands command_enum) {
    logEvent("[Command]: -> " + receiver_node_name + " : " +
             toString(command_enum));  // Category + command log
}

/**
 * @brief Utility to convert MissionStates enum to readable text.
 * @param current_state enum value to convert.
 * @return std::string State name.
 */
std::string StateMachine::toString(MissionStates current_state) {
    // Compact enum-to-string mapping for readable logs and topic output
    switch (current_state) {
        case MissionStates::WAITING:
            return "WAITING";
        case MissionStates::TAKEOFF:
            return "TAKEOFF";
        case MissionStates::TRAVELLING:
            return "TRAVELLING";
        case MissionStates::EXPLORING:
            return "EXPLORING";
        case MissionStates::RETURN_HOME:
            return "RETURN_HOME";
        case MissionStates::LAND:
            return "LAND";
        case MissionStates::DONE:
            return "DONE";
        case MissionStates::ERROR:
            return "ERROR";
        case MissionStates::ABORTED:
            return "ABORTED";
    }
    return "UNKNOWN";
}

/**
 * @brief Utility to convert Commands enum to readable text.
 * @param active_command enum value to convert.
 * @return std::string Command name.
 */
std::string StateMachine::toString(Commands active_command) {
    // Compact enum-to-string mapping for readable logs and topic output
    switch (active_command) {
        case Commands::TAKEOFF:
            return "TAKEOFF";
        case Commands::START:
            return "START";
        case Commands::SWITCH_TO_EXPLORE:
            return "SWITCH_TO_EXPLORE";
        case Commands::HOLD:
            return "HOLD";
        case Commands::RETURN_HOME:
            return "RETURN_HOME";
        case Commands::LAND:
            return "LAND";
        case Commands::ABORT:
            return "ABORT";
        case Commands::NONE:
            return "NONE";
    }
    return "UNKNOWN";
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachine>());
    rclcpp::shutdown();
    return 0;
}
