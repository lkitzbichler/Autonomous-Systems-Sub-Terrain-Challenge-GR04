#include "statemachine_pkg/statemachine.h"
#include "statemachine_pkg/msg/answer.hpp"
#include "statemachine_pkg/msg/command.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <ctime>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

namespace {

double distance3(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

constexpr double kTakeoffHeightM = 5.0; // Fixed takeoff height above start position
constexpr double kMinStartPosNormM = 1e-3; // Reject near-zero start pose (0,0,0) as invalid
constexpr double kMinLandingDescentM = 0.05; // Guard against non-descending landing targets

} // namespace

// #################################################################################################
// Constructor / Destructor
// #################################################################################################

StateMachine::StateMachine() : Node("state_machine_node")
{
    RCLCPP_INFO(this->get_logger(), "[][StateMachine]: State Machine Node has been started.");
    
    // Create and Load Parameters #####################################################################
    // Lists
    this->checkpoint_list_ = declare_parameter<std::vector<double>>("checkpoints.positions", std::vector<double>{});
    this->node_list_ = declare_parameter<std::vector<std::string>>("nodes", std::vector<std::string>{});
    // Timer periods
    this->pub_state_loop_sec_ = declare_parameter<double>("time.pub_state_loop_sec", 1.0);
    this->alive_tol_sec_ = declare_parameter<double>("time.alive_tol_sec", 10.0);
    this->boot_timeout_sec_ = declare_parameter<double>("time.boot_timeout_sec", 60.0);
    this->path_sample_dist_m_ = declare_parameter<double>("viz.path_sample_dist_m", 0.2);
    this->checkpoint_reach_dist_m_ = declare_parameter<double>("checkpoints.reach_dist_m", 0.5);
    this->landing_xy_radius_m_ = declare_parameter<double>("landing.xy_radius_m", 0.5);
    this->landing_clearance_m_ = declare_parameter<double>("landing.clearance_m", 0.2);
    this->landing_probe_depth_m_ = declare_parameter<double>("landing.probe_depth_m", 30.0);
    // Node role indices (refer to entries in node_list_)
    this->node_controller_index_ = declare_parameter<int>("node_roles.controller_index", -1);
    this->node_waypoint_index_ = declare_parameter<int>("node_roles.waypoint_index", -1);
    this->node_planner_index_ = declare_parameter<int>("node_roles.planner_index", -1);
    // Lantern detections
    this->min_lantern_dist_ = declare_parameter<double>("lanterns.lantern_merge_dist_m", 0.2);
    // Logging paths
    this->lantern_log_path_ = declare_parameter<std::string>("logging.lantern_log_path", "lanterns_log.csv");
    this->event_log_path_ = declare_parameter<std::string>("logging.event_log_path", "statemachine_events.log");

    // TOPIC PARAMETERS ###############################################################################
    this->topic_state_ = declare_parameter<std::string>("topics.topic_state", "statemachine/state");
    this->topic_cmd_ = declare_parameter<std::string>("topics.topic_cmd_", "statemachine/cmd");
    this->topic_heartbeat_ = declare_parameter<std::string>("topics.topic_heartbeat", "statemachine/heartbeat");
    this->topic_lantern_detections_ = declare_parameter<std::string>("topics.topic_lantern_detections", "detected_lanterns");
    this->topic_current_state_est_ = declare_parameter<std::string>("topics.current_state_est", "current_state_est");
    this->topic_octomap_ = declare_parameter<std::string>("topics.topic_octomap", "octomap_binary");
    this->topic_viz_checkpoint = declare_parameter<std::string>("topics.topic_viz_checkpoint", "statemachine/viz/checkpoint");
    this->topic_viz_path_flight = declare_parameter<std::string>("topics.topic_viz_path_flight", "statemachine/viz/path_flight");

    // Create Publishers ##############################################################################
    this->pub_state_ = this->create_publisher<std_msgs::msg::String>(topic_state_, 10);
    this->pub_cmd_ = this->create_publisher<statemachine_pkg::msg::Command>(topic_cmd_, 10);
    this->pub_viz_checkpoint_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic_viz_checkpoint, 10);
    this->pub_viz_path_flight_ = this->create_publisher<visualization_msgs::msg::Marker>(topic_viz_path_flight, 10);

    // Create Subscribers #############################################################################
    this->sub_heartbeat_ = this->create_subscription<statemachine_pkg::msg::Answer>(topic_heartbeat_, 10, std::bind(&StateMachine::handleAnswer, this, std::placeholders::_1));
    this->sub_lantern_detections_ = this->create_subscription<geometry_msgs::msg::PoseArray>(topic_lantern_detections_, 10, std::bind(&StateMachine::onLanternDetections, this, std::placeholders::_1));
    this->sub_current_state_est_ = this->create_subscription<nav_msgs::msg::Odometry>(topic_current_state_est_, 10, std::bind(&StateMachine::onCurrentStateEst, this, std::placeholders::_1));
    this->sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(topic_octomap_, 1, std::bind(&StateMachine::onOctomap, this, std::placeholders::_1));

    // Create Timers ##################################################################################
    const double loop_sec = std::max(0.01, pub_state_loop_sec_);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(loop_sec)),
        std::bind(&StateMachine::onTimer, this));

    // Initialize node heartbeat list from config ####################################################
    nodes_.clear();
    nodes_.reserve(node_list_.size());
    for (const auto &name : node_list_) {
        if (!name.empty()) {
            nodes_.push_back(NodeInfo{name});
        }
    }

    // Initialize checkpoint list from config ########################################################
    checkpoint_positions_.clear();
    for (size_t i = 0; i + 2 < checkpoint_list_.size(); i += 3) {
        checkpoint_positions_.emplace_back(
            checkpoint_list_[i],
            checkpoint_list_[i + 1],
            checkpoint_list_[i + 2]);
    }
    checkpoint_positions_base_ = checkpoint_positions_; // Store static list before runtime insertion
    state_enter_time_ = this->now(); // Start WAITING timeout from node startup time
}

StateMachine::~StateMachine()
{
    RCLCPP_INFO(this->get_logger(), "State Machine Node is shutting down.");
    this->logEvent("[SHUTDOWN]: State Machine Node is shutting down.");
}

// #################################################################################################
// State & Command Publishing
// #################################################################################################

/**
 * @brief Publish the current state as a string message on the state topic.
 */
void StateMachine::publishState()
{
    std_msgs::msg::String msg; // Message container for the current state
    msg.data = toString(state_); // Convert enum to string for logging/monitoring
    pub_state_->publish(msg); // Publish the message
}



/**
 * @brief Switch to a new mission state, store entry time, and publish update.
 * @param new_state Target mission state.
 * @param reason Human-readable reason for logging.
 */
void StateMachine::changeState(MissionStates new_state, const std::string &reason)
{
    // Step 1: Ignore if state doesn't change
    if (state_ == new_state) {
        return;
    }

    // Step 2: Update state and entry time
    state_ = new_state;
    state_enter_time_ = this->now();

    // Step 3: Reset boot-timeout log flag when leaving/entering WAITING
    if (state_ != MissionStates::WAITING) {
        boot_timeout_reported_ = false;
    }

    // Step 4: Log and publish new state
    logEvent("[State] " + toString(state_) + " (" + reason + ")");
    publishState();

    // Step 5: Dispatch commands for the new state
    const std::string node_controller = nodeNameByIndex(node_controller_index_);
    const std::string node_waypoint = nodeNameByIndex(node_waypoint_index_);
    const std::string node_planner = nodeNameByIndex(node_planner_index_);

    switch (state_) {
        case MissionStates::WAITING:
            if (!node_controller.empty()) sendCommand(node_controller, Commands::HOLD);
            break;
        case MissionStates::TAKEOFF:
            if (!node_controller.empty()) sendCommand(node_controller, Commands::START);
            if (!node_waypoint.empty() && !checkpoint_positions_.empty()) {
                geometry_msgs::msg::Point target;
                target.x = checkpoint_positions_.front().x();
                target.y = checkpoint_positions_.front().y();
                target.z = checkpoint_positions_.front().z();
                sendCommandWithTarget(node_waypoint, Commands::TAKEOFF, target);
            } else if (!node_waypoint.empty()) {
                logEvent("[TAKEOFF] no checkpoint available for takeoff target");
            }
            break;
        case MissionStates::TRAVELLING:
            if (!node_controller.empty()) sendCommand(node_controller, Commands::START);
            if (!node_waypoint.empty()) sendCommand(node_waypoint, Commands::START);
            break;
        case MissionStates::EXPLORING:
            planner_done_ = false; // Reset planner completion flag
            if (!node_controller.empty()) sendCommand(node_controller, Commands::START);
            if (!node_planner.empty()) sendCommand(node_planner, Commands::START);
            break;
        case MissionStates::RETURN_HOME:
            if (!node_controller.empty()) sendCommand(node_controller, Commands::START);
            if (!node_planner.empty()) sendCommand(node_planner, Commands::RETURN_HOME);
            break;
        case MissionStates::LAND:
            landing_checkpoint_index_ = -1; // Force fresh landing target creation on LAND entry
            if (!node_controller.empty()) sendCommand(node_controller, Commands::START);
            if (!node_waypoint.empty()) {
                geometry_msgs::msg::Point target;
                if (prepareLandingCheckpoint(target)) {
                    sendCommandWithTarget(node_waypoint, Commands::LAND, target);
                } else {
                    logEvent("[LAND] ground estimation failed, landing command not sent");
                    RCLCPP_WARN(this->get_logger(), "[LAND] ground estimation failed, landing command not sent");
                }
            }
            break;
        case MissionStates::DONE:
        case MissionStates::ERROR:
        case MissionStates::ABORTED:
            if (!node_controller.empty()) sendCommand(node_controller, Commands::HOLD);
            if (!node_waypoint.empty()) sendCommand(node_waypoint, Commands::HOLD);
            if (!node_planner.empty()) sendCommand(node_planner, Commands::HOLD);
            break;
    }
}

/**
 * @brief Send a command to a specific node via the shared command topic.
 * @param recv_node Target node name.
 * @param cmd Command enum to send.
 */
void StateMachine::sendCommand(std::string recv_node, Commands cmd)
{
    statemachine_pkg::msg::Command msg; // Command container for one target node
    msg.target = recv_node;             // Target node name
    msg.command = static_cast<uint8_t>(cmd); // Encode enum to message field
    msg.has_target = false;             // No target by default
    const auto now = this->now();       // Timestamp for tracing/debugging
    msg.stamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    msg.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);

    pub_cmd_->publish(msg); // Publish on shared command topic
    this->last_cmd_= cmd; // Store last command for potential state-based logic 
    logCommand(recv_node, cmd); // Persist command in event log
}

/**
 * @brief Send a command to a specific node with a target position.
 * @param recv_node Target node name.
 * @param cmd Command enum to send.
 * @param target Target position for the command.
 */
void StateMachine::sendCommandWithTarget(const std::string &recv_node, Commands cmd, const geometry_msgs::msg::Point &target)
{
    statemachine_pkg::msg::Command msg; // Command container for one target node
    msg.target = recv_node;             // Target node name
    msg.command = static_cast<uint8_t>(cmd); // Encode enum to message field
    msg.has_target = true;              // Explicit target payload
    msg.target_pos = target;            // Target position
    const auto now = this->now();       // Timestamp for tracing/debugging
    msg.stamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    msg.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);

    pub_cmd_->publish(msg); // Publish on shared command topic
    this->last_cmd_= cmd; // Store last command for potential state-based logic 
    logCommand(recv_node, cmd); // Persist command in event log
}

// #################################################################################################
// Heartbeat / Answers
// #################################################################################################

/**
 * @brief Handle heartbeat/answer messages from other nodes.
 * @param msg Incoming Answer message (node name + status + timestamp).
 */
void StateMachine::handleAnswer(const statemachine_pkg::msg::Answer::SharedPtr msg)
{
    // Step 1: Validate message
    if (!msg) {
        return;
    }
    if (msg->node_name.empty()) {
        logEvent("[Answer] missing node_name");
        return;
    }

    // Step 2: Convert status code to enum (RUNNING-only; everything else is UNKNOWN)
    AnswerStates status = AnswerStates::UNKNOWN;
    if (msg->state == static_cast<uint8_t>(AnswerStates::RUNNING)) {
        status = AnswerStates::RUNNING;
    } else if (msg->state == static_cast<uint8_t>(AnswerStates::DONE)) {
        const std::string planner_name = nodeNameByIndex(node_planner_index_);
        if (!planner_name.empty() && msg->node_name == planner_name) {
            status = AnswerStates::DONE;
        }
    }
    auto statusToString = [](AnswerStates s) -> std::string {
        switch (s) {
            case AnswerStates::RUNNING: return "RUNNING";
            case AnswerStates::DONE: return "DONE";
            case AnswerStates::UNKNOWN: return "UNKNOWN";
        }
        return "UNKNOWN";
    };

    // Step 3: Update heartbeat table (create entry if needed)
    auto it = std::find_if(nodes_.begin(), nodes_.end(),
        [&](const NodeInfo &n) { return n.name == msg->node_name; });
    if (it == nodes_.end()) {
        nodes_.push_back(NodeInfo{msg->node_name});
        it = std::prev(nodes_.end());
    }
    if (msg->timestamp.sec == 0 && msg->timestamp.nanosec == 0) {
        it->last_heartbeat = this->now();
    } else {
        it->last_heartbeat = rclcpp::Time(msg->timestamp);
    }
    it->is_alive = true;

    // Step 4: Log only on status change (avoid log spam)
    if (it->last_state != status) {
        logEvent("[Answer] " + msg->node_name + " -> " + statusToString(status));
        it->last_state = status;
    }

    // Step 5: Log first time a node comes online
    if (!it->was_alive) {
        it->was_alive = true;
        logEvent("[Node] " + msg->node_name + " ist online");
        RCLCPP_INFO(this->get_logger(), "[Node] %s ist online", msg->node_name.c_str());
    }

    // Step 5: Only store status/heartbeat here; transitions happen elsewhere
    if (status == AnswerStates::DONE) {
        planner_done_ = true;
    }
}

/**
 * @brief Cache the latest binary octomap message for ground estimation.
 * @param msg Incoming octomap message.
 */
void StateMachine::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    // Step 1: Validate input
    if (!msg) {
        return;
    }
    if (!msg->binary) {
        RCLCPP_WARN(this->get_logger(), "Received non-binary octomap on '%s', expected binary map.", topic_octomap_.c_str());
        return;
    }

    // Step 2: Convert message to OcTree
    octomap::AbstractOcTree *tree_raw = octomap_msgs::binaryMsgToMap(*msg);
    if (!tree_raw) {
        RCLCPP_WARN(this->get_logger(), "Failed to convert octomap message.");
        return;
    }

    auto *tree = dynamic_cast<octomap::OcTree *>(tree_raw);
    if (!tree) {
        delete tree_raw;
        RCLCPP_WARN(this->get_logger(), "Octomap conversion yielded unsupported tree type.");
        return;
    }

    // Step 3: Store map
    octree_.reset(tree);
    has_octomap_ = true;
}

// #################################################################################################
// Lanterns
// #################################################################################################

/**
 * @brief Process lantern detections, update tracking, and log results.
 * @param msg Pose array containing detected lantern positions.
 */
void StateMachine::onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // Step 1: Validate input message
    if (!msg) {
        return; // Defensive: ignore null messages
    }

    if (msg->poses.empty()) {
        logEvent("[Lantern]: empty pose array received"); // Keep a trace of empty detections
        return;
    }

    // Step 2: Process each detection and update/insert a track
    for (const auto &pose : msg->poses) {
        bool is_new = false;
        geometry_msgs::msg::Point mean;
        size_t count = 0;

        // Associate detection with existing tracks or create a new one
        associateLantern(pose.position, is_new, mean, count);

        // Step 3: Resolve the track id for logging
        int id = -1;
        double best_dist = std::numeric_limits<double>::max();
        for (const auto &track : lantern_tracks_) {
            if (track.count != count) {
                continue; // Skip mismatched sample counts
            }
            const double dist = distance3(track.mean, mean);
            if (dist < best_dist) {
                best_dist = dist;
                id = track.id;
            }
        }
        if (id < 0) {
            for (const auto &track : lantern_tracks_) {
                const double dist = distance3(track.mean, mean);
                if (dist < best_dist) {
                    best_dist = dist;
                    id = track.id;
                }
            }
        }

        // Step 4: Build a stamped pose for CSV logging
        geometry_msgs::msg::PoseStamped stamped;
        stamped.header = msg->header;
        stamped.pose.position = mean;
        stamped.pose.orientation.w = 1.0;

        // Persist track info to CSV
        logLanternPose(stamped, id, count);

        // Step 5: Optional info log for new tracks
        if (is_new) {
            logEvent("[Lantern]: new track created");
        }
    }
}

/**
 * @brief Associate a detection with the nearest track or create a new track.
 * @param pos Detected lantern position.
 * @param is_new True if a new track was created.
 * @param mean_out Updated mean position of the associated track.
 * @param count_out Updated sample count for the associated track.
 */
void StateMachine::associateLantern(const geometry_msgs::msg::Point &pos, bool &is_new, geometry_msgs::msg::Point &mean_out, size_t &count_out)
{
    // Step 1: Find nearest existing track within merge distance
    int best_index = -1;
    double best_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < lantern_tracks_.size(); ++i) {
        const double dist = distance3(lantern_tracks_[i].mean, pos);
        if (dist <= min_lantern_dist_ && dist < best_dist) {
            best_dist = dist;
            best_index = static_cast<int>(i);
        }
    }

    // Step 2: Create a new track if none is close enough
    if (best_index < 0) {
        Lantern track;
        track.id = static_cast<int>(lantern_tracks_.size()) + 1; // Compact new id
        track.mean = pos;
        track.samples.push_back(pos);
        track.count = 1;
        lantern_tracks_.push_back(track);

        is_new = true;
        mean_out = pos;
        count_out = track.count;
        return;
    }

    // Step 3: Update running mean and sample count for matched track
    auto &track = lantern_tracks_[static_cast<size_t>(best_index)];
    track.mean.x = (track.mean.x * track.count + pos.x) / (track.count + 1);
    track.mean.y = (track.mean.y * track.count + pos.y) / (track.count + 1);
    track.mean.z = (track.mean.z * track.count + pos.z) / (track.count + 1);
    track.count++;
    track.samples.push_back(pos);

    is_new = false;
    mean_out = track.mean;
    count_out = track.count;
}

// #################################################################################################
// Periodic Checks / Events
// #################################################################################################

/**
 * @brief Periodic timer callback.
 */
void StateMachine::onTimer()
{
    publishState();
    checkHeartbeats();
    checkCheckpoints();
    handleFlagEvents();

    // Step 5: Retry LAND target creation while waiting for octomap/current pose
    if (state_ == MissionStates::LAND && landing_checkpoint_index_ < 0) {
        const std::string node_waypoint = nodeNameByIndex(node_waypoint_index_);
        if (!node_waypoint.empty()) {
            geometry_msgs::msg::Point target;
            if (prepareLandingCheckpoint(target)) {
                sendCommandWithTarget(node_waypoint, Commands::LAND, target);
            }
        }
    }

    publishPathViz();
    publishCheckpointViz();
}

/**
 * @brief Check all node heartbeats against the timeout and log missing nodes.
 */
void StateMachine::checkHeartbeats()
{
    const auto now = this->now();
    const auto graph_nodes = this->get_node_names();

    // Step 1: Update alive flags based on last heartbeat
    for (auto &node : nodes_) {
        // octomap_server is an external node without our custom heartbeat publisher.
        // For this node we treat ROS graph presence as "alive".
        if (node.name == "octomap_server") {
            const auto it = std::find(graph_nodes.begin(), graph_nodes.end(), node.name);
            node.is_alive = (it != graph_nodes.end());
            if (node.is_alive) {
                node.last_heartbeat = now;
            }
            continue;
        }

        if (node.last_heartbeat.nanoseconds() == 0) {
            node.is_alive = false;
        } else {
            const double dt = (now - node.last_heartbeat).seconds();
            node.is_alive = dt <= alive_tol_sec_;
        }
    }

    // Step 2: If all nodes are alive, start mission from WAITING
    if (state_ == MissionStates::WAITING) {
        const bool all_alive = std::all_of(nodes_.begin(), nodes_.end(),
            [](const NodeInfo &n) { return n.is_alive; });
        if (all_alive && start_checkpoint_inserted_) {
            changeState(MissionStates::TAKEOFF, "all nodes online");
        }
    }

    // Step 3: If we are waiting too long, report who is missing (once)
    const double state_dt = (now - state_enter_time_).seconds();
    if (state_ == MissionStates::WAITING && state_dt >= boot_timeout_sec_ && !boot_timeout_reported_) {
        std::string missing;
        for (const auto &node : nodes_) {
            if (!node.is_alive) {
                if (!missing.empty()) {
                    missing += ", ";
                }
                missing += node.name;
            }
        }
        if (missing.empty()) {
            logEvent("[Heartbeat] boot timeout reached but no missing nodes detected");
            RCLCPP_INFO(this->get_logger(), "[Heartbeat] boot timeout reached but no missing nodes detected");
        } else {
            logEvent("[Heartbeat] boot timeout, missing: " + missing);
            RCLCPP_INFO(this->get_logger(), "[Heartbeat] boot timeout, missing: %s", missing.c_str());
        }
        boot_timeout_reported_ = true;
    }
}

/**
 * @brief Evaluate checkpoint progress and update flags.
 */
void StateMachine::checkCheckpoints()
{
    // Step 1: Ensure checkpoints exist
    if (checkpoint_positions_.empty()) {
        return;
    }

    // Step 2: Ensure we have an active checkpoint index
    if (current_checkpoint_index_ < 0) {
        current_checkpoint_index_ = 0;
        return;
    }

    // Step 3: Ensure current pose is known
    if (!has_current_pose_) {
        return;
    }

    // Step 4: Guard against out-of-range index
    if (current_checkpoint_index_ >= static_cast<short>(checkpoint_positions_.size())) {
        return;
    }

    // Step 5: Compute distance to current checkpoint
    const auto &cp = checkpoint_positions_[static_cast<size_t>(current_checkpoint_index_)];
    const double dx = current_position_.x - cp.x();
    const double dy = current_position_.y - cp.y();
    const double dz = current_position_.z - cp.z();
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Step 6: If reached, advance index and raise event flag
    if (dist <= checkpoint_reach_dist_m_) {
        checkpoint_reached_ = true;
        current_checkpoint_index_++;
    }
}

/**
 * @brief Resolve a node name by index from node_list_.
 * @param index Index in the node list.
 * @return Node name or empty string if index is invalid.
 */
std::string StateMachine::nodeNameByIndex(int index) const
{
    if (index < 0 || static_cast<size_t>(index) >= node_list_.size()) {
        return std::string();
    }
    return node_list_[static_cast<size_t>(index)];
}

/**
 * @brief React to events raised by flags.
 */
void StateMachine::handleFlagEvents()
{
    // Step 1: React to checkpoint reached event
    if (checkpoint_reached_) {
        const int reached_index = static_cast<int>(current_checkpoint_index_) - 1;
        logEvent("[Event] checkpoint reached index=" + std::to_string(reached_index));

        // Step 2: State transitions based on checkpoint index
        if (state_ == MissionStates::TAKEOFF && reached_index == 0) {
            changeState(MissionStates::TRAVELLING, "checkpoint 0 reached");
        } else if (state_ == MissionStates::TRAVELLING && reached_index == 1) {
            changeState(MissionStates::EXPLORING, "checkpoint 1 reached");
        } else if (state_ == MissionStates::LAND && reached_index == landing_checkpoint_index_) {
            changeState(MissionStates::DONE, "landing checkpoint reached");
        }

        // Step 2: Reset flag to allow next checkpoint event
        checkpoint_reached_ = false;
    }

    // Step 3: Exploration complete condition
    if (state_ == MissionStates::EXPLORING) {
        if (planner_done_ && lantern_tracks_.size() >= 5) {
            changeState(MissionStates::LAND, "exploration complete");
        }
    }
}

// #################################################################################################
// State Estimate / Visualization
// #################################################################################################

/**
 * @brief Update current position from state estimate.
 * @param msg Odometry message containing the current pose.
 */
void StateMachine::onCurrentStateEst(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    current_position_ = msg->pose.pose.position;
    has_current_pose_ = true;
    if (!msg->header.frame_id.empty()) {
        current_pose_frame_id_ = msg->header.frame_id;
    }

    // Step 1: Try to insert takeoff checkpoint once a valid pose is available
    tryInsertStartCheckpoint();

    // Step 2: Append path point if far enough from last point
    if (path_points_.empty()) {
        path_points_.push_back(current_position_);
        return;
    }
    const auto &last = path_points_.back();
    const double dx = current_position_.x - last.x;
    const double dy = current_position_.y - last.y;
    const double dz = current_position_.z - last.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist >= path_sample_dist_m_) {
        path_points_.push_back(current_position_);
    }
}

/**
 * @brief Publish a line-strip visualization of the flight path.
 */
void StateMachine::publishPathViz()
{
    if (path_points_.size() < 2) {
        return;
    }

    visualization_msgs::msg::Marker line;
    line.header.frame_id = current_pose_frame_id_;
    line.header.stamp = this->now();
    line.ns = "flight_path";
    line.id = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.08; // Line width
    line.color.r = 0.1f;
    line.color.g = 0.9f;
    line.color.b = 0.2f;
    line.color.a = 1.0f;
    line.points = path_points_;

    pub_viz_path_flight_->publish(line);
}

/**
 * @brief Publish checkpoint markers for visualization.
 */
void StateMachine::publishCheckpointViz()
{
    if (checkpoint_positions_.empty()) {
        return;
    }

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.reserve(checkpoint_positions_.size());

    for (size_t i = 0; i < checkpoint_positions_.size(); ++i) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = current_pose_frame_id_;
        m.header.stamp = this->now();
        m.ns = "checkpoints";
        m.id = static_cast<int>(i);
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.5;
        m.scale.y = 0.5;
        m.scale.z = 0.5;
        m.color.r = 0.9f;
        m.color.g = 0.2f;
        m.color.b = 0.2f;
        m.color.a = 1.0f;
        m.pose.orientation.w = 1.0;
        m.pose.position.x = checkpoint_positions_[i].x();
        m.pose.position.y = checkpoint_positions_[i].y();
        m.pose.position.z = checkpoint_positions_[i].z();
        markers.markers.push_back(m);
    }

    pub_viz_checkpoint_->publish(markers);
}

// #################################################################################################
// Checkpoint Helpers
// #################################################################################################

/**
 * @brief Insert a takeoff checkpoint (start position + 5m) at the front of the list.
 */
void StateMachine::tryInsertStartCheckpoint()
{
    // Step 1: Insert only once
    if (start_checkpoint_inserted_) {
        return;
    }

    // Step 2: Require a valid current pose
    if (!has_current_pose_) {
        return;
    }

    // Step 3: Reject zero pose (simulation not running or uninitialized)
    const double norm = std::sqrt(
        current_position_.x * current_position_.x +
        current_position_.y * current_position_.y +
        current_position_.z * current_position_.z);
    if (norm < kMinStartPosNormM) {
        return;
    }

    // Step 4: Build start checkpoint with fixed takeoff height
    const Eigen::Vector3d start_cp(
        current_position_.x,
        current_position_.y,
        current_position_.z + kTakeoffHeightM);

    // Step 5: Rebuild list with start checkpoint at front
    checkpoint_positions_.clear();
    checkpoint_positions_.reserve(checkpoint_positions_base_.size() + 1);
    checkpoint_positions_.push_back(start_cp);
    checkpoint_positions_.insert(
        checkpoint_positions_.end(),
        checkpoint_positions_base_.begin(),
        checkpoint_positions_base_.end());

    // Step 6: Reset checkpoint index to start from the new first checkpoint
    current_checkpoint_index_ = -1;
    start_checkpoint_inserted_ = true;
    logEvent("[Checkpoint] inserted takeoff checkpoint at start position");
}

/**
 * @brief Estimate the highest occupied point below the UAV in a local XY radius.
 * @param ground_z_out Output ground height estimate.
 * @return True if at least one downward ray hit is found.
 */
bool StateMachine::estimateGroundHeight(double &ground_z_out) const
{
    // Step 1: Require pose and map
    if (!has_current_pose_ || !has_octomap_ || !octree_) {
        return false;
    }

    // Step 2: Prepare local XY grid sampling and downward ray direction
    const double radius = std::max(0.0, landing_xy_radius_m_);
    const double step = std::max(0.05, octree_->getResolution());
    const octomap::point3d dir_down(0.0F, 0.0F, -1.0F);
    const double z_start = current_position_.z;

    bool hit_found = false;
    double best_ground_z = -std::numeric_limits<double>::infinity();

    // Step 3: Cast rays downward in a disk around current XY and keep highest hit below UAV
    for (double dx = -radius; dx <= radius + 1e-6; dx += step) {
        for (double dy = -radius; dy <= radius + 1e-6; dy += step) {
            if ((dx * dx + dy * dy) > (radius * radius)) {
                continue;
            }

            const octomap::point3d origin(
                static_cast<float>(current_position_.x + dx),
                static_cast<float>(current_position_.y + dy),
                static_cast<float>(z_start));
            octomap::point3d hit;
            const bool ray_hit = octree_->castRay(origin, dir_down, hit, true, landing_probe_depth_m_);
            if (!ray_hit) {
                continue;
            }

            const double hit_z = static_cast<double>(hit.z());
            if (hit_z >= z_start) {
                continue;
            }
            if (!hit_found || hit_z > best_ground_z) {
                best_ground_z = hit_z;
                hit_found = true;
            }
        }
    }

    // Step 4: Return result
    if (!hit_found) {
        return false;
    }
    ground_z_out = best_ground_z;
    return true;
}

/**
 * @brief Build a dynamic landing checkpoint and point command target from map-estimated ground.
 * @param target_out Landing target output.
 * @return True if target generation succeeded.
 */
bool StateMachine::prepareLandingCheckpoint(geometry_msgs::msg::Point &target_out)
{
    // Step 1: Require current pose and a valid ground estimate
    double ground_z = 0.0;
    if (!has_current_pose_ || !estimateGroundHeight(ground_z)) {
        return false;
    }

    // Step 2: Build target above local ground
    target_out.x = current_position_.x;
    target_out.y = current_position_.y;
    target_out.z = ground_z + landing_clearance_m_;

    // Step 3: Ensure this is an actual descent target
    if (target_out.z >= (current_position_.z - kMinLandingDescentM)) {
        logEvent("[LAND] target too close or above current altitude");
        return false;
    }

    // Step 4: Append or update dedicated landing checkpoint and activate it
    const Eigen::Vector3d landing_cp(target_out.x, target_out.y, target_out.z);
    if (landing_checkpoint_index_ >= 0 &&
        landing_checkpoint_index_ < static_cast<short>(checkpoint_positions_.size())) {
        checkpoint_positions_[static_cast<size_t>(landing_checkpoint_index_)] = landing_cp;
    } else {
        checkpoint_positions_.push_back(landing_cp);
        landing_checkpoint_index_ = static_cast<short>(checkpoint_positions_.size() - 1);
    }
    current_checkpoint_index_ = landing_checkpoint_index_;
    checkpoint_reached_ = false;

    logEvent("[LAND] checkpoint set z=" + std::to_string(target_out.z));
    return true;
}

// #################################################################################################
// Logging / String Helpers
// #################################################################################################

/**
 * @brief Log a lantern detection to CSV with id, date, user, count, and position.
 * @param pose Lantern pose to log (frame and timestamp are used).
 * @param id Lantern track id.
 * @param count Number of samples for this track.
 */
void StateMachine::logLanternPose(const geometry_msgs::msg::PoseStamped &pose, int id, size_t count)
{
    // Step 1: Check if logging is enabled
    if (lantern_log_path_.empty()) {
        return; // Logging disabled
    }

    // Step 2: Read existing CSV (if any)
    std::vector<std::string> lines;
    {
        std::ifstream in(lantern_log_path_);
        std::string line;
        while (in.good() && std::getline(in, line)) {
            if (!line.empty()) {
                lines.push_back(line);
            }
        }
    }

    // Step 3: Resolve timestamp and format date
    rclcpp::Time stamp = pose.header.stamp;
    if (stamp.nanoseconds() <= 0) {
        stamp = this->now();
    }

    const auto ns = std::chrono::nanoseconds(stamp.nanoseconds());
    const auto tp = std::chrono::time_point<std::chrono::system_clock>(ns);
    const auto tt = std::chrono::system_clock::to_time_t(tp);
    std::tm tm{};
    gmtime_r(&tt, &tm);

    std::ostringstream date;
    date << std::put_time(&tm, "%Y/%m/%d");

    // Step 4: Ensure CSV header exists
    const std::string header = "id,zeit,username,anzahl,x,y,z";
    if (lines.empty() || lines.front() != header) {
        lines.insert(lines.begin(), header);
    }

    // Step 5: Compose/replace the row for (id, date, user)
    const std::string user = this->get_name();
    const std::string id_str = std::to_string(id);
    const std::string date_str = date.str();
    const std::string new_line = id_str + "," + date_str + "," +
        user + "," + std::to_string(count) + "," +
        std::to_string(pose.pose.position.x) + "," +
        std::to_string(pose.pose.position.y) + "," +
        std::to_string(pose.pose.position.z);

    bool updated = false;
    for (size_t i = 1; i < lines.size(); ++i) {
        const auto &row = lines[i];
        const auto first = row.find(',');
        const auto second = row.find(',', first + 1);
        const auto third = row.find(',', second + 1);
        if (first == std::string::npos || second == std::string::npos || third == std::string::npos) {
            continue;
        }
        const std::string row_id = row.substr(0, first);
        const std::string row_date = row.substr(first + 1, second - first - 1);
        const std::string row_user = row.substr(second + 1, third - second - 1);
        if (row_id == id_str && row_date == date_str && row_user == user) {
            lines[i] = new_line;
            updated = true;
            break;
        }
    }

    if (!updated) {
        lines.push_back(new_line);
    }

    // Step 6: Write CSV back to disk
    std::ofstream out(lantern_log_path_, std::ios::trunc);
    if (!out) {
        RCLCPP_WARN(this->get_logger(), "Failed to open lantern log file: %s", lantern_log_path_.c_str());
        return;
    }

    for (const auto &row : lines) {
        out << row << "\n";
    }
}

/**
 * @brief Append a formatted event line to the event log.
 * @param message Log message content (without prefix).
 */
void StateMachine::logEvent(const std::string &message)
{
    if (event_log_path_.empty()) {
        return; // Logging disabled
    }

    std::ofstream out(event_log_path_, std::ios::app); // Append to log file
    if (!out) {
        RCLCPP_WARN(this->get_logger(), "Failed to open event log file: %s", event_log_path_.c_str());
        return;
    }

    std::ostringstream line; // Build log line with required prefix
    line << "[" << std::fixed << std::setprecision(3) << this->now().seconds() << "]"
         << "[StateMachine] "
         << message;

    out << line.str() << "\n"; // Timestamped log line
}

/**
 * @brief Log a command with target node and command name.
 * @param recv_node Target node name.
 * @param cmd Command enum value.
 */
void StateMachine::logCommand(const std::string &recv_node, Commands cmd)
{
    logEvent("[Command]: -> " + recv_node + " : " + toString(cmd)); // Category + command log
}

/**
 * @brief Convert mission state enum to a readable string.
 * @param state Mission state enum value.
 * @return Readable state name.
 */
std::string StateMachine::toString(MissionStates state)
{
    // Compact enum-to-string mapping for readable logs and topic output
    switch (state) {
        case MissionStates::WAITING:     return "WAITING";
        case MissionStates::TAKEOFF:     return "TAKEOFF";
        case MissionStates::TRAVELLING:  return "TRAVELLING";
        case MissionStates::EXPLORING:   return "EXPLORING";
        case MissionStates::RETURN_HOME: return "RETURN_HOME";
        case MissionStates::LAND:        return "LAND";
        case MissionStates::DONE:        return "DONE";
        case MissionStates::ERROR:       return "ERROR";
        case MissionStates::ABORTED:     return "ABORTED";
    }
    return "UNKNOWN";
}

/**
 * @brief Convert command enum to a readable string.
 * @param cmd Command enum value.
 * @return Readable command name.
 */
std::string StateMachine::toString(Commands cmd)
{
    // Compact enum-to-string mapping for readable logs and topic output
    switch (cmd) {
        case Commands::TAKEOFF:          return "TAKEOFF";
        case Commands::START:            return "START";
        case Commands::SWITCH_TO_EXPLORE:return "SWITCH_TO_EXPLORE";
        case Commands::HOLD:             return "HOLD";
        case Commands::RETURN_HOME:      return "RETURN_HOME";
        case Commands::LAND:             return "LAND";
        case Commands::ABORT:            return "ABORT";
        case Commands::NONE:             return "NONE";
    }
    return "UNKNOWN";
}







int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachine>());
    rclcpp::shutdown();
    return 0;
}
