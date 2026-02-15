#include "path_planning_pkg/path_planner.h"

#include <array>
#include <algorithm>
#include <queue>
#include <map>
#include <chrono>
#include <cmath>
#include <limits>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mav_planning_msgs/msg/polynomial_segment4_d.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace {

double distance3(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

} // namespace

// #################################################################################################
// Constructor / Destructor
// #################################################################################################

PathPlanner::PathPlanner() : rclcpp::Node("path_planner")
{
    RCLCPP_INFO(this->get_logger(), "[][PathPlanner]: Node startup.");

    // Create and load parameters ###################################################################
    // Topics
    topic_cmd_ = declare_parameter<std::string>("topics.command_topic", "statemachine/cmd");
    topic_state_ = declare_parameter<std::string>("topics.state_topic", "statemachine/state");
    topic_heartbeat_ = declare_parameter<std::string>("topics.heartbeat_topic", "heartbeat");
    topic_odom_ = declare_parameter<std::string>("topics.odom_topic", "current_state_est");
    topic_octomap_ = declare_parameter<std::string>("topics.map_topic", "octomap_binary");
    topic_trajectory_ = declare_parameter<std::string>("topics.trajectory_topic", "trajectory");
    topic_graph_markers_ =
        declare_parameter<std::string>("topics.graph_markers_topic", "path_planner/graph_markers");
    topic_candidate_markers_ =
        declare_parameter<std::string>("topics.candidate_markers_topic", "path_planner/candidate_markers");
    topic_current_plan_ =
        declare_parameter<std::string>("topics.current_plan_topic", "path_planner/current_plan");

    // Timing
    loop_period_sec_ = declare_parameter<double>("time.loop_period_sec", 0.2);
    heartbeat_period_sec_ = declare_parameter<double>("time.heartbeat_period_sec", 1.0);
    input_timeout_sec_ = declare_parameter<double>("time.input_timeout_sec", 1.0);
    map_timeout_sec_ = declare_parameter<double>("time.map_timeout_sec", 0.0);
    command_stale_sec_ = declare_parameter<double>("time.command_stale_sec", 5.0);
    replan_period_sec_ = declare_parameter<double>("time.replan_period_sec", 0.5);
    heartbeat_on_mode_change_ = declare_parameter<bool>("time.heartbeat_on_mode_change", true);

    // Command handling
    accept_empty_target_ = declare_parameter<bool>("commands.accept_empty_target", false);
    accept_switch_to_explore_cmd_ =
        declare_parameter<bool>("commands.accept_switch_to_explore_cmd", true);

    // Frames
    planning_frame_ = declare_parameter<std::string>("frames.planning_frame", "world");

    // Safety
    inflation_m_ = declare_parameter<double>("safety.inflation_m", 1.0);
    min_clearance_m_ = declare_parameter<double>("safety.min_clearance_m", 1.5);
    clearance_step_m_ = declare_parameter<double>("safety.clearance_step_m", 0.5);
    clearance_search_max_m_ = declare_parameter<double>("safety.clearance_search_max_m", 8.0);

    // Graph recording
    transit_node_spacing_m_ = declare_parameter<double>("graph.transit_node_spacing_m", 10.0);
    graph_merge_radius_m_ = declare_parameter<double>("graph.merge_radius_m", 3.0);
    graph_ahead_query_dist_m_ = declare_parameter<double>("graph.ahead_query_dist_m", 12.0);
    graph_loop_connect_radius_m_ = declare_parameter<double>("graph.loop_connect_radius_m", 12.0);
    graph_loop_connect_max_links_ = declare_parameter<int>("graph.loop_connect_max_links", 3);
    route_node_reached_dist_m_ = declare_parameter<double>("graph.route_node_reached_dist_m", 1.2);

    // Explore candidate generation
    candidate_distance_m_ = declare_parameter<double>("explore.candidate_distance_m", 8.0);
    candidate_half_fov_deg_ = declare_parameter<double>("explore.candidate_half_fov_deg", 60.0);
    candidate_bin_count_ = declare_parameter<int>("explore.candidate_bin_count", 7);
    candidate_vertical_half_fov_deg_ =
        declare_parameter<double>("explore.candidate_vertical_half_fov_deg", 30.0);
    candidate_vertical_bin_count_ = declare_parameter<int>("explore.candidate_vertical_bin_count", 3);
    branch_probe_distance_m_ = declare_parameter<double>("explore.branch_probe_distance_m", 60.0);
    side_opening_min_abs_yaw_deg_ =
        declare_parameter<double>("explore.side_opening_min_abs_yaw_deg", 80.0);
    side_opening_min_depth_m_ = declare_parameter<double>("explore.side_opening_min_depth_m", 60.0);
    side_opening_bundle_half_yaw_deg_ =
        declare_parameter<double>("explore.side_opening_bundle_half_yaw_deg", 10.0);
    side_opening_bundle_half_pitch_deg_ =
        declare_parameter<double>("explore.side_opening_bundle_half_pitch_deg", 12.0);
    side_opening_bundle_yaw_bins_ = declare_parameter<int>("explore.side_opening_bundle_yaw_bins", 5);
    side_opening_bundle_pitch_bins_ = declare_parameter<int>("explore.side_opening_bundle_pitch_bins", 3);
    side_opening_required_fraction_ =
        declare_parameter<double>("explore.side_opening_required_fraction", 0.7);
    side_opening_confirm_cycles_ = declare_parameter<int>("explore.side_opening_confirm_cycles", 3);
    side_opening_max_candidates_ = declare_parameter<int>("explore.side_opening_max_candidates", 1);
    side_opening_reuse_radius_m_ = declare_parameter<double>("explore.side_opening_reuse_radius_m", 15.0);
    prefer_unknown_over_free_ = declare_parameter<bool>("explore.prefer_unknown_over_free", true);
    frontier_node_spacing_m_ = declare_parameter<double>("explore.frontier_node_spacing_m", 6.0);
    max_frontier_nodes_per_cycle_ = declare_parameter<int>("explore.max_frontier_nodes_per_cycle", 3);
    create_side_frontier_nodes_ = declare_parameter<bool>("explore.create_side_frontier_nodes", true);
    known_path_reject_radius_m_ = declare_parameter<double>("explore.known_path_reject_radius_m", 4.0);
    frontier_cleanup_radius_m_ = declare_parameter<double>("explore.frontier_cleanup_radius_m", 10.0);
    event_cooldown_sec_ = declare_parameter<double>("explore.event_cooldown_sec", 2.0);

    // Trajectory output
    trajectory_publish_enabled_ =
        declare_parameter<bool>("trajectory.publish_enabled", true);
    trajectory_nominal_speed_mps_ =
        declare_parameter<double>("trajectory.nominal_speed_mps", 2.5);
    trajectory_min_segment_time_sec_ =
        declare_parameter<double>("trajectory.min_segment_time_sec", 0.8);
    trajectory_goal_replan_dist_m_ =
        declare_parameter<double>("trajectory.goal_replan_dist_m", 1.0);

    // Publishers ###################################################################################
    pub_heartbeat_ = create_publisher<statemachine_pkg::msg::Answer>(topic_heartbeat_, 10);
    pub_trajectory_ =
        create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(topic_trajectory_, 10);
    pub_graph_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(topic_graph_markers_, 10);
    pub_candidate_markers_ =
        create_publisher<visualization_msgs::msg::MarkerArray>(topic_candidate_markers_, 10);
    pub_current_plan_ = create_publisher<nav_msgs::msg::Path>(topic_current_plan_, 10);

    // Subscribers ##################################################################################
    sub_cmd_ = create_subscription<statemachine_pkg::msg::Command>(
        topic_cmd_, 10, std::bind(&PathPlanner::onCommand, this, std::placeholders::_1));
    sub_sm_state_ = create_subscription<std_msgs::msg::String>(
        topic_state_, 10, std::bind(&PathPlanner::onState, this, std::placeholders::_1));
    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        topic_odom_, 10, std::bind(&PathPlanner::onOdometry, this, std::placeholders::_1));
    sub_octomap_ = create_subscription<octomap_msgs::msg::Octomap>(
        topic_octomap_, 1, std::bind(&PathPlanner::onOctomap, this, std::placeholders::_1));

    // Timer ########################################################################################
    const double loop_sec = std::max(0.01, loop_period_sec_);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(loop_sec)),
        std::bind(&PathPlanner::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "[][PathPlanner]: Ready (cmd='%s', state='%s').",
                topic_cmd_.c_str(), topic_state_.c_str());
}

PathPlanner::~PathPlanner()
{
    RCLCPP_INFO(this->get_logger(), "[][PathPlanner]: Shutdown.");
}

// #################################################################################################
// Callbacks
// #################################################################################################

void PathPlanner::onCommand(const statemachine_pkg::msg::Command::SharedPtr msg)
{
    // Step 1: Validate message and target
    if (!msg) {
        return;
    }
    if (!isTargetMatch(msg->target)) {
        ++ignored_command_count_;
        return;
    }
    if (isStaleCommand(*msg)) {
        ++ignored_command_count_;
        RCLCPP_WARN(this->get_logger(), "[cmd] ignored stale/out-of-order command id=%u", msg->command);
        return;
    }

    // Step 2: Store command metadata
    last_command_ = static_cast<Commands>(msg->command);
    if (msg->stamp.sec == 0 && msg->stamp.nanosec == 0) {
        last_command_stamp_ = this->now();
    } else {
        last_command_stamp_ = rclcpp::Time(msg->stamp);
    }

    // Step 3: React to relevant mission commands
    bool handled = true;
    switch (msg->command) {
    case static_cast<uint8_t>(Commands::START):
        changeMode(PlannerMode::EXPLORE, "command START");
        break;
    case static_cast<uint8_t>(Commands::SWITCH_TO_EXPLORE):
        if (accept_switch_to_explore_cmd_) {
            changeMode(PlannerMode::EXPLORE, "command SWITCH_TO_EXPLORE");
        } else {
            handled = false;
        }
        break;
    case static_cast<uint8_t>(Commands::RETURN_HOME):
        if (msg->has_target) {
            return_home_target_ = msg->target_pos;
            has_return_home_target_ = true;
        }
        changeMode(PlannerMode::RETURN_HOME, "command RETURN_HOME");
        break;
    case static_cast<uint8_t>(Commands::HOLD):
        changeMode(PlannerMode::HOLD, "command HOLD");
        break;
    case static_cast<uint8_t>(Commands::ABORT):
        changeMode(PlannerMode::ABORTED, "command ABORT");
        break;
    default:
        handled = false;
        break;
    }

    if (handled) {
        ++processed_command_count_;
    } else {
        ++ignored_command_count_;
        RCLCPP_INFO(this->get_logger(), "[cmd] ignored unsupported id=%u", msg->command);
    }
}

void PathPlanner::onState(const std_msgs::msg::String::SharedPtr msg)
{
    // Step 1: Validate and cache state text
    if (!msg) {
        return;
    }
    statemachine_state_ = msg->data;

    // Step 2: Update recording gate and mode transitions for transit capture
    updateTransitGateFromState(statemachine_state_);
}

void PathPlanner::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Step 1: Validate and cache latest odometry
    if (!msg) {
        return;
    }
    latest_odom_ = *msg;
    has_odom_ = true;
    odom_frame_id_ = msg->header.frame_id;
    odom_frame_ok_ = isPlanningFrame(odom_frame_id_);

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
        last_odom_time_ = this->now();
    } else {
        last_odom_time_ = rclcpp::Time(msg->header.stamp);
    }

    if (!odom_frame_ok_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[odom] frame mismatch: expected '%s', got '%s'",
                             planning_frame_.c_str(), odom_frame_id_.c_str());
    }

    // Step 2: During transit-record mode, capture sparse breadcrumb nodes
    if (mode_ == PlannerMode::TRANSIT_RECORD && transit_record_enabled_) {
        tryRecordTransitNode();
    }
}

void PathPlanner::onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    // Step 1: Validate input and basic metadata.
    if (!msg) {
        return;
    }
    if (!msg->binary) {
        has_map_ = false;
        octree_.reset();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[map] received non-binary octomap on '%s'", topic_octomap_.c_str());
        return;
    }

    map_frame_id_ = msg->header.frame_id;
    map_frame_ok_ = isPlanningFrame(map_frame_id_);
    if (!map_frame_ok_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[map] frame mismatch: expected '%s', got '%s'",
                             planning_frame_.c_str(), map_frame_id_.c_str());
    }

    // Step 2: Convert octomap message to OcTree.
    octomap::AbstractOcTree *tree_raw = octomap_msgs::binaryMsgToMap(*msg);
    if (!tree_raw) {
        has_map_ = false;
        octree_.reset();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[map] failed to convert octomap message");
        return;
    }

    auto *tree = dynamic_cast<octomap::OcTree *>(tree_raw);
    if (!tree) {
        delete tree_raw;
        has_map_ = false;
        octree_.reset();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[map] unsupported octomap tree type");
        return;
    }

    // Step 3: Cache map and timestamp.
    octree_.reset(tree);
    has_map_ = true;
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
        last_map_time_ = this->now();
    } else {
        last_map_time_ = rclcpp::Time(msg->header.stamp);
    }
}

void PathPlanner::onTimer()
{
    // Step 1: Surface degraded input quality in active planner modes.
    const bool active_mode = (mode_ == PlannerMode::EXPLORE ||
                              mode_ == PlannerMode::BACKTRACK ||
                              mode_ == PlannerMode::RETURN_HOME);
    if (active_mode && !hasValidPlanningInputs()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[inputs] planning disabled: valid_odom=%s valid_map=%s",
                             hasValidOdom() ? "true" : "false",
                             hasValidMap() ? "true" : "false");
    }

    // Step 2: Publish periodic heartbeat in a controlled rate.
    const auto now = this->now();
    const bool first_pub = (last_heartbeat_time_.nanoseconds() == 0);
    const bool due = first_pub || ((now - last_heartbeat_time_).seconds() >= heartbeat_period_sec_);
    if (due) {
        publishHeartbeat();
        last_heartbeat_time_ = now;
    }

    // Step 3: Update and publish branch candidates for RViz inspection.
    updateBranchCandidates();
    updateExploreGraphFromCandidates();
    publishCandidateMarkers();

    // Step 4: Publish graph visualization for runtime inspection.
    publishGraphMarkers();

    // Step 5: Build local path and publish trajectory output if replanning is due.
    updateLocalPlanAndTrajectory();
}

// #################################################################################################
// Helpers
// #################################################################################################

void PathPlanner::changeMode(PlannerMode new_mode, const std::string &reason)
{
    if (mode_ == new_mode) {
        return;
    }

    mode_ = new_mode;
    RCLCPP_INFO(this->get_logger(), "[mode] -> %s (%s)", toString(mode_).c_str(), reason.c_str());

    // Reset short-horizon planning cache on every mode transition.
    has_last_plan_goal_ = false;
    last_plan_publish_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);

    if (mode_ != PlannerMode::EXPLORE) {
        has_frontier_seed_position_ = false;
        last_branch_detected_ = false;
        force_graph_replan_after_loop_ = false;
    }

    if (mode_ == PlannerMode::EXPLORE) {
        active_route_node_ids_.clear();
        active_route_index_ = 0;
        exploration_done_reported_ = false;
    }

    if (mode_ == PlannerMode::BACKTRACK) {
        active_route_node_ids_.clear();
        active_route_index_ = 0;
    }

    if (mode_ == PlannerMode::RETURN_HOME) {
        active_route_node_ids_.clear();
        active_route_index_ = 0;
        return_home_done_reported_ = false;
    }

    if (mode_ == PlannerMode::HOLD || mode_ == PlannerMode::ABORTED || mode_ == PlannerMode::IDLE) {
        active_route_node_ids_.clear();
        active_route_index_ = 0;
    }

    // Step 2: Optionally publish an immediate heartbeat on mode change.
    if (heartbeat_on_mode_change_) {
        publishHeartbeat();
        last_heartbeat_time_ = this->now();
    }
}

void PathPlanner::updateTransitGateFromState(const std::string &sm_state)
{
    // Gate enabled only in TRAVELLING, per interface contract.
    const bool was_enabled = transit_record_enabled_;
    transit_record_enabled_ = (sm_state == "TRAVELLING");

    if (!was_enabled && transit_record_enabled_) {
        if (mode_ == PlannerMode::IDLE) {
            changeMode(PlannerMode::TRANSIT_RECORD, "statemachine state TRAVELLING");
        }
        return;
    }

    if (was_enabled && !transit_record_enabled_ && mode_ == PlannerMode::TRANSIT_RECORD) {
        changeMode(PlannerMode::IDLE, "left TRAVELLING");
    }
}

void PathPlanner::tryRecordTransitNode()
{
    // Step 0: Require valid odometry
    if (!hasValidOdom()) {
        return;
    }

    const auto &pos = latest_odom_.pose.pose.position;
    const double spacing_m = std::max(0.1, transit_node_spacing_m_);

    // Step 1: Enforce spacing against the last accepted transit graph node.
    if (last_transit_graph_node_id_ > 0) {
        const auto *last_node = findNodeById(last_transit_graph_node_id_);
        if (last_node && distance3(last_node->position, pos) < spacing_m) {
            return;
        }
    }

    // Step 2: Insert or merge node into persistent graph (duplicate suppression).
    bool inserted_new = false;
    const int node_id = addOrMergeGraphNode(pos, true, inserted_new);
    if (node_id <= 0) {
        return;
    }

    // Step 3: Maintain edge continuity for transit path recording.
    if (last_transit_graph_node_id_ > 0 && last_transit_graph_node_id_ != node_id) {
        upsertGraphEdge(last_transit_graph_node_id_, node_id);
    }

    // Step 4: Lightweight legacy breadcrumb list for debug continuity.
    if (inserted_new || transit_nodes_.empty()) {
        transit_nodes_.push_back(TransitNode{next_transit_node_id_++, pos, this->now()});
    }

    // Step 5: Advance transit cursor.
    last_transit_graph_node_id_ = node_id;
    if (home_node_id_ <= 0) {
        home_node_id_ = node_id;
    }

    // Step 6: Emit compact graph growth stats for stepwise validation.
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "[graph] nodes=%zu edges=%zu last_node=%d",
                         graph_nodes_.size(), graph_edges_.size(), last_transit_graph_node_id_);
}

void PathPlanner::publishHeartbeat()
{
    statemachine_pkg::msg::Answer hb;
    hb.node_name = this->get_name();
    hb.state = static_cast<uint8_t>(AnswerStates::RUNNING);
    if (!pending_event_info_.empty()) {
        hb.info = pending_event_info_;
        pending_event_info_.clear();
    } else {
        hb.info = heartbeatInfo();
    }

    const auto now = this->now();
    hb.timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    hb.timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    pub_heartbeat_->publish(hb);
}

void PathPlanner::publishDone(const std::string &info_code)
{
    statemachine_pkg::msg::Answer msg;
    msg.node_name = this->get_name();
    msg.state = static_cast<uint8_t>(AnswerStates::DONE);
    msg.info = info_code;
    const auto now = this->now();
    msg.timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    msg.timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    pub_heartbeat_->publish(msg);
    last_heartbeat_time_ = now;
}

void PathPlanner::publishGraphMarkers()
{
    if (!pub_graph_markers_) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    const auto now = this->now();

    // Node markers -------------------------------------------------------------------------------
    for (const auto &node : graph_nodes_) {
        if (!node.valid) {
            continue;
        }

        visualization_msgs::msg::Marker m;
        m.header.frame_id = planning_frame_;
        m.header.stamp = now;
        m.ns = "graph_nodes";
        m.id = node.id;
        m.type = visualization_msgs::msg::Marker::SPHERE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.pose.position = node.position;
        m.scale.x = 0.45;
        m.scale.y = 0.45;
        m.scale.z = 0.45;
        m.color.a = 0.95F;

        if (node.is_transit) {
            m.color.r = 0.10F;
            m.color.g = 0.60F;
            m.color.b = 1.00F;
        } else {
            switch (node.status) {
            case GraphNodeStatus::UNVISITED:
                m.color.r = 1.00F;
                m.color.g = 0.50F;
                m.color.b = 0.10F;
                break;
            case GraphNodeStatus::VISITED:
                m.color.r = 0.10F;
                m.color.g = 0.85F;
                m.color.b = 0.20F;
                break;
            case GraphNodeStatus::FRONTIER:
                m.color.r = 0.95F;
                m.color.g = 0.80F;
                m.color.b = 0.10F;
                break;
            case GraphNodeStatus::DEAD_END:
                m.color.r = 0.95F;
                m.color.g = 0.20F;
                m.color.b = 0.20F;
                break;
            }
        }

        array.markers.push_back(m);
    }

    // Edge marker as line list -------------------------------------------------------------------
    visualization_msgs::msg::Marker lines;
    lines.header.frame_id = planning_frame_;
    lines.header.stamp = now;
    lines.ns = "graph_edges";
    lines.id = 1;
    lines.type = visualization_msgs::msg::Marker::LINE_LIST;
    lines.action = visualization_msgs::msg::Marker::ADD;
    lines.scale.x = 0.10;
    lines.color.r = 0.90F;
    lines.color.g = 0.90F;
    lines.color.b = 0.90F;
    lines.color.a = 0.90F;

    for (const auto &edge : graph_edges_) {
        if (!edge.valid) {
            continue;
        }
        const auto *from = findNodeById(edge.from_id);
        const auto *to = findNodeById(edge.to_id);
        if (!from || !to || !from->valid || !to->valid) {
            continue;
        }
        lines.points.push_back(from->position);
        lines.points.push_back(to->position);
    }

    array.markers.push_back(lines);
    pub_graph_markers_->publish(array);
}

void PathPlanner::updateBranchCandidates()
{
    latest_candidates_.clear();
    candidate_known_path_flags_.clear();
    side_opening_targets_.clear();
    selected_candidate_index_.reset();
    branch_detected_ = false;
    free_candidate_count_ = 0;
    unknown_candidate_count_ = 0;

    // Candidate generation is only meaningful in planner-active modes.
    const bool active_mode = (mode_ == PlannerMode::EXPLORE ||
                              mode_ == PlannerMode::BACKTRACK ||
                              mode_ == PlannerMode::RETURN_HOME ||
                              mode_ == PlannerMode::TRANSIT_RECORD);
    if (!active_mode || !hasValidOdom()) {
        return;
    }
    const bool map_ok = hasValidMap();

    const int requested_bins_yaw = std::max(1, candidate_bin_count_);
    const int bins_yaw = (requested_bins_yaw % 2 == 0) ? requested_bins_yaw + 1 : requested_bins_yaw;
    const int requested_bins_pitch = std::max(1, candidate_vertical_bin_count_);
    const int bins_pitch =
        (requested_bins_pitch % 2 == 0) ? requested_bins_pitch + 1 : requested_bins_pitch;
    const double half_fov_yaw_deg = std::max(0.0, candidate_half_fov_deg_);
    const double half_fov_pitch_deg = std::max(0.0, candidate_vertical_half_fov_deg_);
    const double probe_dist_m = std::max(0.5, candidate_distance_m_);
    const double branch_probe_dist_m = std::max(probe_dist_m, branch_probe_distance_m_);
    constexpr double kDegToRad = 0.017453292519943295;

    const auto &pose = latest_odom_.pose.pose;
    const double yaw = yawFromQuaternion(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    const geometry_msgs::msg::Point origin = pose.position;

    auto classifyProbe = [&](const geometry_msgs::msg::Point &target) {
        if (!map_ok) {
            return CandidateStatus::UNKNOWN;
        }
        geometry_msgs::msg::Point hit;
        const bool blocked_los = raycast(origin, target, hit, false);
        if (blocked_los || isOccupied(target)) {
            return CandidateStatus::BLOCKED;
        }
        if (isUnknown(target)) {
            return CandidateStatus::UNKNOWN;
        }
        if (isFree(target)) {
            return CandidateStatus::FREE;
        }
        return CandidateStatus::BLOCKED;
    };

    for (int ip = 0; ip < bins_pitch; ++ip) {
        const double beta =
            (bins_pitch == 1) ? 0.0 : (-1.0 + 2.0 * static_cast<double>(ip) / (bins_pitch - 1));
        const double rel_pitch_deg = beta * half_fov_pitch_deg;
        const double rel_pitch_rad = rel_pitch_deg * kDegToRad;

        for (int iy = 0; iy < bins_yaw; ++iy) {
            const double alpha =
                (bins_yaw == 1) ? 0.0 : (-1.0 + 2.0 * static_cast<double>(iy) / (bins_yaw - 1));
            const double rel_yaw_deg = alpha * half_fov_yaw_deg;
            const double heading = yaw + rel_yaw_deg * kDegToRad;

            geometry_msgs::msg::Point target;
            target.x = origin.x + std::cos(rel_pitch_rad) * std::cos(heading) * probe_dist_m;
            target.y = origin.y + std::cos(rel_pitch_rad) * std::sin(heading) * probe_dist_m;
            target.z = origin.z + std::sin(rel_pitch_rad) * probe_dist_m;

            CandidatePoint candidate;
            candidate.point = target;
            candidate.rel_yaw_deg = rel_yaw_deg;
            candidate.rel_pitch_deg = rel_pitch_deg;
            candidate.status = classifyProbe(target);
            if (candidate.status == CandidateStatus::UNKNOWN) {
                ++unknown_candidate_count_;
            } else if (candidate.status == CandidateStatus::FREE) {
                ++free_candidate_count_;
            }

            latest_candidates_.push_back(candidate);

            bool known_path = false;
            if (known_path_reject_radius_m_ > 0.0) {
                const auto nearest = findNearestNodeId(candidate.point, known_path_reject_radius_m_);
                if (nearest.has_value()) {
                    const auto *node = findNodeById(*nearest);
                    if (node && node->valid && (node->is_transit || node->status == GraphNodeStatus::VISITED)) {
                        known_path = true;
                    }
                }
            }
            candidate_known_path_flags_.push_back(known_path);
        }
    }

    // Pick a preferred direction for upcoming policy steps:
    // unknown can be prioritized over free (configurable).
    auto candidatePriority = [this](CandidateStatus status) {
        if (status == CandidateStatus::UNKNOWN) {
            return prefer_unknown_over_free_ ? 3 : 2;
        }
        if (status == CandidateStatus::FREE) {
            return prefer_unknown_over_free_ ? 2 : 3;
        }
        return 1;
    };

    int best_priority = std::numeric_limits<int>::min();
    double best_angle_cost = std::numeric_limits<double>::max();
    bool found_not_known = false;
    for (std::size_t i = 0; i < latest_candidates_.size(); ++i) {
        const auto &c = latest_candidates_[i];
        if (!(c.status == CandidateStatus::UNKNOWN || c.status == CandidateStatus::FREE)) {
            continue;
        }

        const bool known_path = (i < candidate_known_path_flags_.size() && candidate_known_path_flags_[i]);
        if (found_not_known && known_path) {
            continue;
        }

        int prio = candidatePriority(c.status);
        if (known_path) {
            prio -= 10;
        }
        const double angle_cost = std::abs(c.rel_yaw_deg) + 0.7 * std::abs(c.rel_pitch_deg);
        if (prio > best_priority ||
            (prio == best_priority && angle_cost < best_angle_cost)) {
            best_priority = prio;
            best_angle_cost = angle_cost;
            selected_candidate_index_ = i;
            if (!known_path) {
                found_not_known = true;
            }
        }
    }

    struct SideBundleResult {
        bool detected{false};
        double mean_depth_m{0.0};
        double best_pitch_deg{0.0};
    };

    auto evaluateSideBundle = [&](double side_sign) {
        SideBundleResult result;
        if (!map_ok) {
            return result;
        }

        int yaw_bins = std::max(1, side_opening_bundle_yaw_bins_);
        if ((yaw_bins % 2) == 0) {
            ++yaw_bins;
        }
        int pitch_bins = std::max(1, side_opening_bundle_pitch_bins_);
        if ((pitch_bins % 2) == 0) {
            ++pitch_bins;
        }

        const double half_yaw_deg = std::max(0.0, side_opening_bundle_half_yaw_deg_);
        const double half_pitch_deg = std::max(0.0, side_opening_bundle_half_pitch_deg_);
        const double min_side_abs_yaw = std::max(0.0, side_opening_min_abs_yaw_deg_);
        const double min_open_depth = std::max(0.1, side_opening_min_depth_m_);
        const double required_fraction = std::clamp(side_opening_required_fraction_, 0.0, 1.0);

        int sample_count = 0;
        int pass_count = 0;
        double depth_sum = 0.0;
        double best_depth = -1.0;
        double best_pitch_deg = 0.0;

        for (int ip = 0; ip < pitch_bins; ++ip) {
            const double beta =
                (pitch_bins == 1) ? 0.0 : (-1.0 + 2.0 * static_cast<double>(ip) / (pitch_bins - 1));
            const double rel_pitch_deg = beta * half_pitch_deg;
            const double rel_pitch_rad = rel_pitch_deg * kDegToRad;

            for (int iy = 0; iy < yaw_bins; ++iy) {
                const double alpha =
                    (yaw_bins == 1) ? 0.0 : (-1.0 + 2.0 * static_cast<double>(iy) / (yaw_bins - 1));
                const double rel_yaw_deg = side_sign * 90.0 + alpha * half_yaw_deg;
                if (std::abs(rel_yaw_deg) < min_side_abs_yaw) {
                    continue;
                }

                const double heading = yaw + rel_yaw_deg * kDegToRad;
                geometry_msgs::msg::Point far_target;
                far_target.x = origin.x + std::cos(rel_pitch_rad) * std::cos(heading) * branch_probe_dist_m;
                far_target.y = origin.y + std::cos(rel_pitch_rad) * std::sin(heading) * branch_probe_dist_m;
                far_target.z = origin.z + std::sin(rel_pitch_rad) * branch_probe_dist_m;

                geometry_msgs::msg::Point hit;
                const bool blocked = raycast(origin, far_target, hit, false);
                const double open_depth = blocked ? distance3(origin, hit) : branch_probe_dist_m;

                ++sample_count;
                depth_sum += open_depth;
                if (open_depth > best_depth) {
                    best_depth = open_depth;
                    best_pitch_deg = rel_pitch_deg;
                }
                if (open_depth >= min_open_depth) {
                    ++pass_count;
                }
            }
        }

        if (sample_count <= 0) {
            return result;
        }

        const double pass_fraction = static_cast<double>(pass_count) / static_cast<double>(sample_count);
        if (pass_fraction >= required_fraction) {
            result.detected = true;
            result.mean_depth_m = depth_sum / static_cast<double>(sample_count);
            result.best_pitch_deg = best_pitch_deg;
        }
        return result;
    };

    const auto left_bundle = evaluateSideBundle(-1.0);
    const auto right_bundle = evaluateSideBundle(1.0);
    side_opening_left_streak_ = left_bundle.detected ? (side_opening_left_streak_ + 1) : 0;
    side_opening_right_streak_ = right_bundle.detected ? (side_opening_right_streak_ + 1) : 0;

    auto makeSideTarget = [&](double side_sign, double rel_pitch_deg) {
        geometry_msgs::msg::Point target;
        const double rel_yaw_deg = side_sign * 90.0;
        const double rel_pitch_rad = rel_pitch_deg * kDegToRad;
        const double heading = yaw + rel_yaw_deg * kDegToRad;
        const double branch_step = std::max(0.5, candidate_distance_m_);
        target.x = origin.x + std::cos(rel_pitch_rad) * std::cos(heading) * branch_step;
        target.y = origin.y + std::cos(rel_pitch_rad) * std::sin(heading) * branch_step;
        target.z = origin.z + std::sin(rel_pitch_rad) * branch_step;
        return target;
    };

    struct StableSideCandidate {
        geometry_msgs::msg::Point target{};
        double mean_depth_m{0.0};
    };
    std::vector<StableSideCandidate> stable_sides;
    const int confirm_cycles = std::max(1, side_opening_confirm_cycles_);
    if (left_bundle.detected && side_opening_left_streak_ >= confirm_cycles) {
        stable_sides.push_back({makeSideTarget(-1.0, left_bundle.best_pitch_deg), left_bundle.mean_depth_m});
    }
    if (right_bundle.detected && side_opening_right_streak_ >= confirm_cycles) {
        stable_sides.push_back({makeSideTarget(1.0, right_bundle.best_pitch_deg), right_bundle.mean_depth_m});
    }

    std::stable_sort(stable_sides.begin(), stable_sides.end(),
                     [](const StableSideCandidate &lhs, const StableSideCandidate &rhs) {
        return lhs.mean_depth_m > rhs.mean_depth_m;
    });

    const int side_limit = std::max(0, side_opening_max_candidates_);
    for (int i = 0; i < side_limit && i < static_cast<int>(stable_sides.size()); ++i) {
        const auto &candidate = stable_sides[static_cast<std::size_t>(i)];
        if (isOccupied(candidate.target)) {
            continue;
        }
        bool known_path = false;
        if (known_path_reject_radius_m_ > 0.0) {
            const auto nearest = findNearestNodeId(candidate.target, known_path_reject_radius_m_);
            if (nearest.has_value()) {
                const auto *node = findNodeById(*nearest);
                if (node && node->valid && (node->is_transit || node->status == GraphNodeStatus::VISITED)) {
                    known_path = true;
                }
            }
        }
        if (!known_path) {
            side_opening_targets_.push_back(candidate.target);
        }
    }

    branch_detected_ = !side_opening_targets_.empty();
    if (branch_detected_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[branch] stable side opening count=%zu left_streak=%d right_streak=%d",
                             side_opening_targets_.size(), side_opening_left_streak_, side_opening_right_streak_);
    }
}

void PathPlanner::updateExploreGraphFromCandidates()
{
    // Graph updates from local fan-candidates are only used in exploration-like modes.
    const bool active_mode = (mode_ == PlannerMode::EXPLORE ||
                              mode_ == PlannerMode::BACKTRACK ||
                              mode_ == PlannerMode::RETURN_HOME);
    if (!active_mode || !hasValidOdom() || !hasValidMap() || latest_candidates_.empty()) {
        last_branch_detected_ = branch_detected_;
        return;
    }

    // Raise branch event once per rising edge (with cooldown for spam protection).
    if (branch_detected_ && !last_branch_detected_) {
        queueEventInfo("EVENT_BRANCH_DETECTED", last_branch_event_time_);
    }
    last_branch_detected_ = branch_detected_;

    const auto now = this->now();
    const auto &pose = latest_odom_.pose.pose;
    const auto &origin = pose.position;
    constexpr double kDegToRad = 0.017453292519943295;
    const double yaw = yawFromQuaternion(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);

    // Keep graph anchored at the current vehicle position.
    bool inserted_anchor = false;
    const int anchor_id = addOrMergeGraphNode(origin, false, inserted_anchor);
    if (anchor_id <= 0) {
        return;
    }
    auto *anchor = findNodeById(anchor_id);
    if (!anchor) {
        return;
    }
    previous_anchor_node_id_ = current_anchor_node_id_;
    current_anchor_node_id_ = anchor_id;
    anchor->status = GraphNodeStatus::VISITED;
    anchor->yaw = yaw;
    anchor->stamp = now;
    anchor->valid = true;

    bool loop_closed = false;

    // Try to explicitly connect to already-visited graph in a wider radius.
    const double connect_radius = std::max(graph_merge_radius_m_, graph_loop_connect_radius_m_);
    const int max_links = std::max(0, graph_loop_connect_max_links_);
    int links_added = 0;
    const auto nearby_ids = findNearbyNodeIds(origin, connect_radius);
    for (const int node_id : nearby_ids) {
        if (node_id <= 0 || node_id == anchor_id || node_id == previous_anchor_node_id_) {
            continue;
        }
        if (max_links > 0 && links_added >= max_links) {
            break;
        }

        const auto *node = findNodeById(node_id);
        if (!node || !node->valid) {
            continue;
        }
        if (!(node->is_transit || node->status == GraphNodeStatus::VISITED ||
              node->status == GraphNodeStatus::DEAD_END)) {
            continue;
        }

        geometry_msgs::msg::Point hit;
        const bool blocked = raycast(origin, node->position, hit, false);
        if (blocked || isOccupied(node->position)) {
            continue;
        }

        const bool had_edge = hasGraphEdge(anchor_id, node_id);
        upsertGraphEdge(anchor_id, node_id);
        if (!had_edge) {
            loop_closed = true;
            ++links_added;
        }
    }

    // During BACKTRACK / RETURN_HOME we only maintain anchor + loop connections.
    if (mode_ != PlannerMode::EXPLORE) {
        if (loop_closed) {
            force_graph_replan_after_loop_ = true;
            queueEventInfo("EVENT_LOOP_CLOSED", last_loop_event_time_);
        }
        return;
    }

    // Apply placement hysteresis to avoid node flood while hovering.
    const double min_seed_spacing = std::max(0.2, frontier_node_spacing_m_);
    if (has_frontier_seed_position_ &&
        distance3(last_frontier_seed_position_, origin) < min_seed_spacing) {
        return;
    }

    // Collect exploratory candidates and rank deterministically.
    std::vector<std::size_t> exploratory_indices;
    exploratory_indices.reserve(latest_candidates_.size());
    for (std::size_t i = 0; i < latest_candidates_.size(); ++i) {
        const auto status = latest_candidates_[i].status;
        const bool known_path = i < candidate_known_path_flags_.size() && candidate_known_path_flags_[i];
        if ((status == CandidateStatus::FREE || status == CandidateStatus::UNKNOWN) && !known_path) {
            exploratory_indices.push_back(i);
        }
    }
    if (exploratory_indices.empty()) {
        return;
    }

    const auto candidatePriority = [this](const CandidatePoint &c) {
        if (c.status == CandidateStatus::UNKNOWN) {
            return prefer_unknown_over_free_ ? 3 : 2;
        }
        if (c.status == CandidateStatus::FREE) {
            return prefer_unknown_over_free_ ? 2 : 3;
        }
        return 1;
    };

    auto ranked = exploratory_indices;
    std::stable_sort(ranked.begin(), ranked.end(), [&](std::size_t lhs, std::size_t rhs) {
        const auto &a = latest_candidates_[lhs];
        const auto &b = latest_candidates_[rhs];
        const int pa = candidatePriority(a);
        const int pb = candidatePriority(b);
        if (pa != pb) {
            return pa > pb;
        }
        const double ca = std::abs(a.rel_yaw_deg) + 0.7 * std::abs(a.rel_pitch_deg);
        const double cb = std::abs(b.rel_yaw_deg) + 0.7 * std::abs(b.rel_pitch_deg);
        if (std::abs(ca - cb) > 1e-9) {
            return ca < cb;
        }
        return lhs < rhs;
    });

    const bool allow_side_nodes =
        create_side_frontier_nodes_ && branch_detected_ && !side_opening_targets_.empty();
    const int max_nodes = std::max(1, max_frontier_nodes_per_cycle_);
    const int limit = allow_side_nodes ? max_nodes : 1;
    std::vector<std::size_t> chosen;
    chosen.reserve(static_cast<std::size_t>(limit));
    auto pushUnique = [&chosen](std::size_t idx) {
        if (std::find(chosen.begin(), chosen.end(), idx) == chosen.end()) {
            chosen.push_back(idx);
        }
    };

    if (selected_candidate_index_.has_value() && *selected_candidate_index_ < latest_candidates_.size()) {
        const auto selected_status = latest_candidates_[*selected_candidate_index_].status;
        const bool selected_known_path =
            *selected_candidate_index_ < candidate_known_path_flags_.size() &&
            candidate_known_path_flags_[*selected_candidate_index_];
        if (!selected_known_path &&
            (selected_status == CandidateStatus::FREE || selected_status == CandidateStatus::UNKNOWN)) {
            pushUnique(*selected_candidate_index_);
        }
    }

    std::vector<int> chosen_node_ids;
    chosen_node_ids.reserve(static_cast<std::size_t>(limit));

    int side_nodes_added = 0;
    if (allow_side_nodes) {
        const int side_budget = std::max(0, limit - static_cast<int>(chosen.size()));
        const double reuse_radius = std::max(0.0, side_opening_reuse_radius_m_);
        for (const auto &raw_side_target : side_opening_targets_) {
            if (side_nodes_added >= side_budget) {
                break;
            }

            geometry_msgs::msg::Point side_target = raw_side_target;
            if (reuse_radius > 1e-6) {
                const auto nearest = findNearestNodeId(side_target, reuse_radius);
                if (nearest.has_value()) {
                    const auto *nearest_node = findNodeById(*nearest);
                    if (nearest_node && nearest_node->valid &&
                        nearest_node->status == GraphNodeStatus::FRONTIER &&
                        nearest_node->frontier_score >= 2.4) {
                        side_target = nearest_node->position;
                    }
                }
            }

            bool inserted_new = false;
            const int side_node_id = addOrMergeGraphNode(side_target, false, inserted_new);
            if (side_node_id <= 0 || side_node_id == anchor_id) {
                continue;
            }

            auto *side_node = findNodeById(side_node_id);
            if (!side_node) {
                continue;
            }
            side_node->status = GraphNodeStatus::FRONTIER;
            side_node->frontier_score = 2.5;
            side_node->yaw = yaw;
            side_node->stamp = now;
            side_node->valid = true;

            // Collapse nearby duplicate side-frontier nodes into one stable target.
            if (reuse_radius > 1e-6) {
                for (auto &node : graph_nodes_) {
                    if (!node.valid || node.is_transit || node.id == side_node_id) {
                        continue;
                    }
                    if (!(node.status == GraphNodeStatus::FRONTIER || node.status == GraphNodeStatus::UNVISITED)) {
                        continue;
                    }
                    if (distance3(node.position, side_node->position) > reuse_radius) {
                        continue;
                    }
                    node.valid = false;
                    for (auto &edge : graph_edges_) {
                        if (edge.from_id == node.id || edge.to_id == node.id) {
                            edge.valid = false;
                        }
                    }
                }
            }
            chosen_node_ids.push_back(side_node_id);

            const bool had_edge = hasGraphEdge(anchor_id, side_node_id);
            upsertGraphEdge(anchor_id, side_node_id);
            if (!inserted_new && !had_edge) {
                loop_closed = true;
            }
            ++side_nodes_added;
        }
    }

    auto appendRanked = [&]() {
        for (const auto idx : ranked) {
            if ((static_cast<int>(chosen.size()) + side_nodes_added) >= limit) {
                break;
            }
            if (selected_candidate_index_.has_value() && idx == *selected_candidate_index_) {
                continue;
            }
            if (idx < candidate_known_path_flags_.size() && candidate_known_path_flags_[idx]) {
                continue;
            }
            pushUnique(idx);
        }
    };

    appendRanked();

    for (const auto idx : chosen) {
        const auto &candidate = latest_candidates_[idx];
        const bool known_path = idx < candidate_known_path_flags_.size() && candidate_known_path_flags_[idx];

        bool inserted_new = false;
        const int node_id = addOrMergeGraphNode(candidate.point, false, inserted_new);
        if (node_id <= 0) {
            continue;
        }

        auto *node = findNodeById(node_id);
        if (node) {
            const bool selected = selected_candidate_index_.has_value() &&
                                  (*selected_candidate_index_ == idx);
            if (known_path && !inserted_new) {
                node->status = GraphNodeStatus::VISITED;
                node->frontier_score = 0.0;
            } else {
                node->status = selected ? GraphNodeStatus::UNVISITED : GraphNodeStatus::FRONTIER;
                node->frontier_score = (candidate.status == CandidateStatus::UNKNOWN) ? 2.0 : 1.0;
            }
            node->yaw = yaw + candidate.rel_yaw_deg * kDegToRad;
            node->stamp = now;
            node->valid = true;
        }
        chosen_node_ids.push_back(node_id);

        if (node_id == anchor_id) {
            continue;
        }

        const bool had_edge = hasGraphEdge(anchor_id, node_id);
        upsertGraphEdge(anchor_id, node_id);
        if (!inserted_new && !had_edge) {
            loop_closed = true;
        }
    }

    // Mark stale local alternatives as dead-end instead of deleting graph topology.
    const double cleanup_radius = std::max(1.0, frontier_cleanup_radius_m_);
    for (auto &node : graph_nodes_) {
        if (!node.valid || node.is_transit || node.id == anchor_id) {
            continue;
        }
        if (distance3(node.position, origin) > cleanup_radius) {
            continue;
        }
        if (!(node.status == GraphNodeStatus::FRONTIER || node.status == GraphNodeStatus::UNVISITED)) {
            continue;
        }
        const bool keep = std::find(chosen_node_ids.begin(), chosen_node_ids.end(), node.id) !=
            chosen_node_ids.end();
        if (!keep) {
            node.status = GraphNodeStatus::DEAD_END;
            node.frontier_score = 0.0;
            node.stamp = now;
        }
    }

    has_frontier_seed_position_ = true;
    last_frontier_seed_position_ = origin;

    if (loop_closed) {
        force_graph_replan_after_loop_ = true;
        queueEventInfo("EVENT_LOOP_CLOSED", last_loop_event_time_);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "[frontier] anchor=%d add_candidates=%zu nodes=%zu edges=%zu",
                         anchor_id, chosen.size(), graph_nodes_.size(), graph_edges_.size());
}

void PathPlanner::updateLocalPlanAndTrajectory()
{
    // Local planning is active only when planner outputs are allowed.
    if (!canPublishPlannerOutput()) {
        return;
    }

    advanceRouteProgressIfReached();

    if (mode_ == PlannerMode::EXPLORE) {
        if (force_graph_replan_after_loop_) {
            force_graph_replan_after_loop_ = false;
            const bool has_unvisited = hasUnvisitedFrontiers();
            if (has_unvisited && current_anchor_node_id_ > 0) {
                const auto frontier_node = nearestUnvisitedFrontierNodeId(current_anchor_node_id_);
                if (frontier_node.has_value() && startRouteToNode(*frontier_node)) {
                    changeMode(PlannerMode::BACKTRACK, "loop closure, switch to graph backtrack");
                    return;
                }
                queueEventInfo("EVENT_STUCK", last_stuck_event_time_);
                return;
            }
            if (!has_unvisited) {
                if (!exploration_done_reported_) {
                    publishDone("DONE_EXPLORATION_COMPLETE");
                    exploration_done_reported_ = true;
                    RCLCPP_INFO(this->get_logger(), "[explore] complete after loop closure.");
                }
                return;
            }
        }

        const auto explore_goal = selectGoalPoint();
        if (!explore_goal.has_value()) {
            if (hasUnvisitedFrontiers()) {
                if (current_anchor_node_id_ > 0) {
                    const auto frontier_node = nearestUnvisitedFrontierNodeId(current_anchor_node_id_);
                    if (frontier_node.has_value() && startRouteToNode(*frontier_node)) {
                        changeMode(PlannerMode::BACKTRACK, "explore dead-end, backtrack to unvisited");
                    } else {
                        queueEventInfo("EVENT_STUCK", last_stuck_event_time_);
                    }
                } else {
                    queueEventInfo("EVENT_STUCK", last_stuck_event_time_);
                }
            } else if (!exploration_done_reported_) {
                publishDone("DONE_EXPLORATION_COMPLETE");
                exploration_done_reported_ = true;
                RCLCPP_INFO(this->get_logger(), "[explore] complete, no unvisited frontiers left.");
            }
            return;
        }
    }

    if (mode_ == PlannerMode::BACKTRACK) {
        if (active_route_node_ids_.empty()) {
            if (hasUnvisitedFrontiers() && current_anchor_node_id_ > 0) {
                const auto frontier_node = nearestUnvisitedFrontierNodeId(current_anchor_node_id_);
                if (!(frontier_node.has_value() && startRouteToNode(*frontier_node))) {
                    queueEventInfo("EVENT_STUCK", last_stuck_event_time_);
                    return;
                }
            } else {
                changeMode(PlannerMode::EXPLORE, "no reachable unvisited frontier");
                return;
            }
        }
        if (active_route_index_ >= active_route_node_ids_.size()) {
            active_route_node_ids_.clear();
            active_route_index_ = 0;
            changeMode(PlannerMode::EXPLORE, "backtrack route complete");
            return;
        }
    }

    if (mode_ == PlannerMode::RETURN_HOME) {
        if (!ensureReturnHomeNode()) {
            queueEventInfo("EVENT_STUCK", last_stuck_event_time_);
            return;
        }

        if (active_route_node_ids_.empty()) {
            if (!startRouteToNode(home_node_id_)) {
                queueEventInfo("EVENT_STUCK", last_stuck_event_time_);
                return;
            }
        }

        if (active_route_index_ >= active_route_node_ids_.size()) {
            if (!return_home_done_reported_) {
                publishDone("DONE_RETURN_HOME_REACHED");
                return_home_done_reported_ = true;
            }
            changeMode(PlannerMode::HOLD, "return-home route complete");
            return;
        }
    }

    const auto goal_opt = selectGoalPoint();
    if (!goal_opt.has_value()) {
        return;
    }

    if (!shouldReplanForGoal(*goal_opt)) {
        return;
    }

    std::vector<geometry_msgs::msg::Point> path_points;
    if (!computeLocalWaypointPath(*goal_opt, path_points)) {
        queueEventInfo("EVENT_STUCK", last_stuck_event_time_);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[plan] local path generation failed");
        return;
    }

    publishCurrentPlan(path_points);
    if (trajectory_publish_enabled_ && !publishTrajectoryFromPath(path_points)) {
        queueEventInfo("EVENT_STUCK", last_stuck_event_time_);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[plan] trajectory publication failed");
        return;
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "[plan] published waypoints=%zu trajectories=%zu",
                         path_points.size(), published_plan_count_);
}

std::optional<geometry_msgs::msg::Point> PathPlanner::selectGoalPoint() const
{
    if ((mode_ == PlannerMode::BACKTRACK || mode_ == PlannerMode::RETURN_HOME) &&
        active_route_index_ < active_route_node_ids_.size()) {
        const auto *route_node = findNodeById(active_route_node_ids_[active_route_index_]);
        if (route_node && route_node->valid) {
            return route_node->position;
        }
    }

    if (mode_ != PlannerMode::EXPLORE) {
        return std::nullopt;
    }

    // Priority 1: use selected exploratory candidate from current local fan.
    if (selected_candidate_index_.has_value() && *selected_candidate_index_ < latest_candidates_.size()) {
        const bool selected_known_path =
            *selected_candidate_index_ < candidate_known_path_flags_.size() &&
            candidate_known_path_flags_[*selected_candidate_index_];
        const auto &selected = latest_candidates_[*selected_candidate_index_];
        if (!selected_known_path &&
            (selected.status == CandidateStatus::FREE || selected.status == CandidateStatus::UNKNOWN)) {
            return selected.point;
        }
    }

    // Priority 2: nearest graph frontier/unvisited node.
    if (!hasValidOdom()) {
        return std::nullopt;
    }

    const auto &origin = latest_odom_.pose.pose.position;
    const double max_query_dist = std::max(1.0, graph_ahead_query_dist_m_ * 2.0);
    bool found = false;
    geometry_msgs::msg::Point best{};
    int best_id = -1;
    double best_dist = std::numeric_limits<double>::max();

    for (const auto &node : graph_nodes_) {
        if (!node.valid || node.is_transit) {
            continue;
        }
        if (!(node.status == GraphNodeStatus::FRONTIER || node.status == GraphNodeStatus::UNVISITED)) {
            continue;
        }

        const double d = distance3(origin, node.position);
        if (d > max_query_dist) {
            continue;
        }
        if (!found || d < best_dist || (std::abs(d - best_dist) <= 1e-9 && node.id < best_id)) {
            found = true;
            best = node.position;
            best_id = node.id;
            best_dist = d;
        }
    }

    if (!found) {
        return std::nullopt;
    }
    return best;
}

bool PathPlanner::hasUnvisitedFrontiers() const
{
    for (const auto &node : graph_nodes_) {
        if (!node.valid || node.is_transit) {
            continue;
        }
        if (node.status == GraphNodeStatus::FRONTIER || node.status == GraphNodeStatus::UNVISITED) {
            return true;
        }
    }
    return false;
}

std::optional<int> PathPlanner::nearestUnvisitedFrontierNodeId(int start_node_id) const
{
    if (start_node_id <= 0) {
        return std::nullopt;
    }
    const auto *start = findNodeById(start_node_id);
    if (!start || !start->valid) {
        return std::nullopt;
    }

    std::map<int, std::vector<std::pair<int, double>>> adjacency;
    for (const auto &node : graph_nodes_) {
        if (node.valid) {
            adjacency[node.id];
        }
    }
    for (const auto &edge : graph_edges_) {
        if (!edge.valid) {
            continue;
        }
        const auto *from = findNodeById(edge.from_id);
        const auto *to = findNodeById(edge.to_id);
        if (!from || !to || !from->valid || !to->valid) {
            continue;
        }
        adjacency[edge.from_id].push_back({edge.to_id, edge.cost});
        adjacency[edge.to_id].push_back({edge.from_id, edge.cost});
    }
    if (adjacency.find(start_node_id) == adjacency.end()) {
        return std::nullopt;
    }

    const double inf = std::numeric_limits<double>::infinity();
    std::map<int, double> dist;
    for (const auto &kv : adjacency) {
        dist[kv.first] = inf;
    }
    dist[start_node_id] = 0.0;

    using QueueItem = std::pair<double, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;
    pq.push({0.0, start_node_id});

    while (!pq.empty()) {
        const auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u] + 1e-9) {
            continue;
        }
        for (const auto &[v, w] : adjacency[u]) {
            const double alt = d + w;
            if (alt + 1e-9 < dist[v]) {
                dist[v] = alt;
                pq.push({alt, v});
            }
        }
    }

    bool found = false;
    int best_id = -1;
    double best_cost = inf;
    double best_euclid = inf;
    for (const auto &node : graph_nodes_) {
        if (!node.valid || node.is_transit) {
            continue;
        }
        if (!(node.status == GraphNodeStatus::FRONTIER || node.status == GraphNodeStatus::UNVISITED)) {
            continue;
        }
        const auto it = dist.find(node.id);
        if (it == dist.end() || !std::isfinite(it->second)) {
            continue;
        }
        const double euclid = distance3(start->position, node.position);
        if (!found || it->second < best_cost - 1e-9 ||
            (std::abs(it->second - best_cost) <= 1e-9 && euclid < best_euclid - 1e-9) ||
            (std::abs(it->second - best_cost) <= 1e-9 &&
             std::abs(euclid - best_euclid) <= 1e-9 && node.id < best_id)) {
            found = true;
            best_id = node.id;
            best_cost = it->second;
            best_euclid = euclid;
        }
    }

    if (!found) {
        return std::nullopt;
    }
    return best_id;
}

bool PathPlanner::computeShortestPathNodeIds(int start_id, int goal_id, std::vector<int> &path_out) const
{
    path_out.clear();
    if (start_id <= 0 || goal_id <= 0) {
        return false;
    }
    if (start_id == goal_id) {
        path_out.push_back(start_id);
        return true;
    }

    const auto *start = findNodeById(start_id);
    const auto *goal = findNodeById(goal_id);
    if (!start || !goal || !start->valid || !goal->valid) {
        return false;
    }

    std::map<int, std::vector<std::pair<int, double>>> adjacency;
    for (const auto &node : graph_nodes_) {
        if (node.valid) {
            adjacency[node.id];
        }
    }
    for (const auto &edge : graph_edges_) {
        if (!edge.valid) {
            continue;
        }
        const auto *from = findNodeById(edge.from_id);
        const auto *to = findNodeById(edge.to_id);
        if (!from || !to || !from->valid || !to->valid) {
            continue;
        }
        adjacency[edge.from_id].push_back({edge.to_id, edge.cost});
        adjacency[edge.to_id].push_back({edge.from_id, edge.cost});
    }

    if (adjacency.find(start_id) == adjacency.end() || adjacency.find(goal_id) == adjacency.end()) {
        return false;
    }

    const double inf = std::numeric_limits<double>::infinity();
    std::map<int, double> dist;
    std::map<int, int> prev;
    for (const auto &kv : adjacency) {
        dist[kv.first] = inf;
        prev[kv.first] = -1;
    }
    dist[start_id] = 0.0;

    using QueueItem = std::pair<double, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;
    pq.push({0.0, start_id});

    while (!pq.empty()) {
        const auto [d, u] = pq.top();
        pq.pop();
        if (d > dist[u] + 1e-9) {
            continue;
        }
        if (u == goal_id) {
            break;
        }
        for (const auto &[v, w] : adjacency[u]) {
            const double alt = d + w;
            if (alt + 1e-9 < dist[v] ||
                (std::abs(alt - dist[v]) <= 1e-9 && (prev[v] < 0 || u < prev[v]))) {
                dist[v] = alt;
                prev[v] = u;
                pq.push({alt, v});
            }
        }
    }

    if (!std::isfinite(dist[goal_id])) {
        return false;
    }

    std::vector<int> rev;
    for (int at = goal_id; at >= 0; at = prev[at]) {
        rev.push_back(at);
        if (at == start_id) {
            break;
        }
    }
    if (rev.empty() || rev.back() != start_id) {
        return false;
    }

    path_out.assign(rev.rbegin(), rev.rend());
    return !path_out.empty();
}

bool PathPlanner::startRouteToNode(int goal_node_id)
{
    if (goal_node_id <= 0 || current_anchor_node_id_ <= 0) {
        return false;
    }

    std::vector<int> route;
    if (!computeShortestPathNodeIds(current_anchor_node_id_, goal_node_id, route)) {
        return false;
    }
    if (route.size() <= 1) {
        active_route_node_ids_.clear();
        active_route_index_ = 0;
        return true;
    }

    active_route_node_ids_ = std::move(route);
    active_route_index_ = 1; // index 0 is current node
    RCLCPP_INFO(this->get_logger(), "[route] start=%d goal=%d hops=%zu",
                current_anchor_node_id_, goal_node_id, active_route_node_ids_.size());
    return true;
}

bool PathPlanner::ensureReturnHomeNode()
{
    if (home_node_id_ > 0) {
        const auto *home = findNodeById(home_node_id_);
        if (home && home->valid) {
            return true;
        }
    }

    if (has_return_home_target_) {
        bool inserted_new = false;
        const int node_id = addOrMergeGraphNode(return_home_target_, true, inserted_new);
        if (node_id > 0) {
            home_node_id_ = node_id;
            if (auto *home = findNodeById(home_node_id_)) {
                home->status = GraphNodeStatus::VISITED;
                home->is_transit = true;
                home->valid = true;
            }
            return true;
        }
    }

    if (home_node_id_ <= 0 && last_transit_graph_node_id_ > 0) {
        home_node_id_ = last_transit_graph_node_id_;
    }
    if (home_node_id_ > 0) {
        const auto *home = findNodeById(home_node_id_);
        return home && home->valid;
    }
    return false;
}

void PathPlanner::advanceRouteProgressIfReached()
{
    if (!hasValidOdom() || active_route_node_ids_.empty()) {
        return;
    }

    const auto &origin = latest_odom_.pose.pose.position;
    const double reached_dist = std::max(0.2, route_node_reached_dist_m_);
    while (active_route_index_ < active_route_node_ids_.size()) {
        const auto *target = findNodeById(active_route_node_ids_[active_route_index_]);
        if (!target || !target->valid) {
            ++active_route_index_;
            continue;
        }
        if (distance3(origin, target->position) <= reached_dist) {
            ++active_route_index_;
            continue;
        }
        break;
    }
}

bool PathPlanner::shouldReplanForGoal(const geometry_msgs::msg::Point &goal) const
{
    if (last_plan_publish_time_.nanoseconds() == 0) {
        return true;
    }

    const auto now = this->now();
    if (replan_period_sec_ > 0.0 &&
        (now - last_plan_publish_time_).seconds() >= replan_period_sec_) {
        return true;
    }

    if (!has_last_plan_goal_) {
        return true;
    }

    const double goal_eps = std::max(0.05, trajectory_goal_replan_dist_m_);
    return distance3(last_plan_goal_, goal) >= goal_eps;
}

bool PathPlanner::computeLocalWaypointPath(
    const geometry_msgs::msg::Point &goal,
    std::vector<geometry_msgs::msg::Point> &path) const
{
    path.clear();
    if (!hasValidOdom()) {
        return false;
    }

    const auto &origin = latest_odom_.pose.pose.position;
    path.push_back(origin);

    if (distance3(origin, goal) <= 0.05) {
        return false;
    }

    auto isExploratory = [](CandidateStatus status) {
        return (status == CandidateStatus::FREE || status == CandidateStatus::UNKNOWN);
    };

    auto hasLineOfSight = [this](const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
        geometry_msgs::msg::Point hit;
        return !raycast(a, b, hit, false) && !isOccupied(b);
    };

    if (hasLineOfSight(origin, goal)) {
        path.push_back(goal);
        return true;
    }

    // One-hop bridge over candidate rays for simple local obstacle bypass.
    bool bridge_found = false;
    geometry_msgs::msg::Point best_bridge{};
    double best_cost = std::numeric_limits<double>::max();
    for (const auto &candidate : latest_candidates_) {
        if (!isExploratory(candidate.status)) {
            continue;
        }
        const auto &mid = candidate.point;
        if (!hasLineOfSight(origin, mid) || !hasLineOfSight(mid, goal)) {
            continue;
        }

        const double cost =
            distance3(origin, mid) + distance3(mid, goal) +
            0.1 * std::abs(candidate.rel_yaw_deg) + 0.05 * std::abs(candidate.rel_pitch_deg);
        if (!bridge_found || cost < best_cost) {
            bridge_found = true;
            best_bridge = mid;
            best_cost = cost;
        }
    }

    if (!bridge_found) {
        return false;
    }

    path.push_back(best_bridge);
    path.push_back(goal);
    return true;
}

void PathPlanner::publishCurrentPlan(const std::vector<geometry_msgs::msg::Point> &path)
{
    if (!pub_current_plan_ || path.empty()) {
        return;
    }

    nav_msgs::msg::Path msg;
    const auto now = this->now();
    msg.header.frame_id = planning_frame_;
    msg.header.stamp = now;
    msg.poses.reserve(path.size());
    for (const auto &p : path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position = p;
        pose.pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
    }
    pub_current_plan_->publish(msg);

    last_plan_publish_time_ = now;
    last_plan_goal_ = path.back();
    has_last_plan_goal_ = true;
}

bool PathPlanner::publishTrajectoryFromPath(const std::vector<geometry_msgs::msg::Point> &path)
{
    if (!pub_trajectory_ || path.size() < 2) {
        return false;
    }

    mav_planning_msgs::msg::PolynomialTrajectory4D traj;
    const auto now = this->now();
    traj.header.frame_id = planning_frame_;
    traj.header.stamp = now;

    const double nominal_speed = std::max(0.1, trajectory_nominal_speed_mps_);
    const double min_dt = std::max(0.1, trajectory_min_segment_time_sec_);

    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
        const auto &a = path[i];
        const auto &b = path[i + 1];
        const double d = distance3(a, b);
        if (d <= 0.05) {
            continue;
        }

        const double dt = std::max(min_dt, d / nominal_speed);
        mav_planning_msgs::msg::PolynomialSegment4D seg;
        seg.header.frame_id = planning_frame_;
        seg.header.stamp = now;
        seg.num_coeffs = 2;

        seg.segment_time.sec = static_cast<int32_t>(dt);
        seg.segment_time.nanosec = static_cast<uint32_t>((dt - static_cast<double>(seg.segment_time.sec)) * 1e9);

        const double vx = (b.x - a.x) / dt;
        const double vy = (b.y - a.y) / dt;
        const double vz = (b.z - a.z) / dt;
        seg.x = {a.x, vx};
        seg.y = {a.y, vy};
        seg.z = {a.z, vz};

        const double yaw = std::atan2(b.y - a.y, b.x - a.x);
        seg.yaw = {yaw, 0.0};
        traj.segments.push_back(seg);
    }

    if (traj.segments.empty()) {
        return false;
    }

    pub_trajectory_->publish(traj);
    ++published_plan_count_;
    return true;
}

void PathPlanner::publishCandidateMarkers()
{
    if (!pub_candidate_markers_) {
        return;
    }

    visualization_msgs::msg::MarkerArray array;
    const auto now = this->now();

    // Clear previous marker set to keep visualization stable across count changes.
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = planning_frame_;
    clear.header.stamp = now;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    if (latest_candidates_.empty() || !hasValidOdom()) {
        pub_candidate_markers_->publish(array);
        return;
    }

    const auto origin = latest_odom_.pose.pose.position;

    visualization_msgs::msg::Marker free_points;
    free_points.header.frame_id = planning_frame_;
    free_points.header.stamp = now;
    free_points.ns = "candidate_points";
    free_points.id = 1;
    free_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    free_points.action = visualization_msgs::msg::Marker::ADD;
    free_points.scale.x = 0.35;
    free_points.scale.y = 0.35;
    free_points.scale.z = 0.35;
    free_points.color.r = 0.10F;
    free_points.color.g = 0.90F;
    free_points.color.b = 0.20F;
    free_points.color.a = 0.95F;

    visualization_msgs::msg::Marker blocked_points = free_points;
    blocked_points.id = 2;
    blocked_points.color.r = 0.95F;
    blocked_points.color.g = 0.20F;
    blocked_points.color.b = 0.20F;

    visualization_msgs::msg::Marker unknown_points = free_points;
    unknown_points.id = 3;
    unknown_points.color.r = 0.95F;
    unknown_points.color.g = 0.80F;
    unknown_points.color.b = 0.10F;

    visualization_msgs::msg::Marker rays;
    rays.header.frame_id = planning_frame_;
    rays.header.stamp = now;
    rays.ns = "candidate_rays";
    rays.id = 4;
    rays.type = visualization_msgs::msg::Marker::LINE_LIST;
    rays.action = visualization_msgs::msg::Marker::ADD;
    rays.scale.x = 0.05;
    rays.color.r = 0.85F;
    rays.color.g = 0.85F;
    rays.color.b = 0.85F;
    rays.color.a = 0.85F;

    visualization_msgs::msg::Marker selected;
    selected.header.frame_id = planning_frame_;
    selected.header.stamp = now;
    selected.ns = "candidate_selected";
    selected.id = 5;
    selected.type = visualization_msgs::msg::Marker::SPHERE;
    selected.action = visualization_msgs::msg::Marker::ADD;
    selected.scale.x = 0.55;
    selected.scale.y = 0.55;
    selected.scale.z = 0.55;
    selected.color.r = 1.00F;
    selected.color.g = 0.00F;
    selected.color.b = 1.00F;
    selected.color.a = 1.00F;

    visualization_msgs::msg::Marker side_openings = free_points;
    side_openings.id = 6;
    side_openings.color.r = 0.00F;
    side_openings.color.g = 1.00F;
    side_openings.color.b = 1.00F;
    side_openings.color.a = 1.00F;
    side_openings.scale.x = 0.50;
    side_openings.scale.y = 0.50;
    side_openings.scale.z = 0.50;

    for (const auto &candidate : latest_candidates_) {
        switch (candidate.status) {
        case CandidateStatus::FREE:
            free_points.points.push_back(candidate.point);
            break;
        case CandidateStatus::BLOCKED:
            blocked_points.points.push_back(candidate.point);
            break;
        case CandidateStatus::UNKNOWN:
            unknown_points.points.push_back(candidate.point);
            break;
        }

        rays.points.push_back(origin);
        rays.points.push_back(candidate.point);
    }

    for (const auto &p : side_opening_targets_) {
        side_openings.points.push_back(p);
    }

    array.markers.push_back(free_points);
    array.markers.push_back(blocked_points);
    array.markers.push_back(unknown_points);
    array.markers.push_back(side_openings);
    array.markers.push_back(rays);
    if (selected_candidate_index_.has_value() &&
        *selected_candidate_index_ < latest_candidates_.size()) {
        selected.pose.position = latest_candidates_[*selected_candidate_index_].point;
        selected.pose.orientation.w = 1.0;
        array.markers.push_back(selected);
    }
    pub_candidate_markers_->publish(array);
}

bool PathPlanner::isTargetMatch(const std::string &target) const
{
    if (target.empty()) {
        return accept_empty_target_;
    }

    const std::string plain = this->get_name();
    const std::string namespaced = "/" + plain;
    const std::string fq = this->get_fully_qualified_name();

    return (target == plain || target == namespaced || target == fq);
}

bool PathPlanner::isStaleCommand(const statemachine_pkg::msg::Command &msg) const
{
    // No timestamp means no stale/out-of-order check.
    if (msg.stamp.sec == 0 && msg.stamp.nanosec == 0) {
        return false;
    }

    const rclcpp::Time cmd_stamp(msg.stamp);

    // Reject commands that arrive older than already processed commands.
    if (last_command_stamp_.nanoseconds() != 0 && cmd_stamp <= last_command_stamp_) {
        return true;
    }

    // Reject old commands based on current clock if stale filtering is enabled.
    if (command_stale_sec_ > 0.0) {
        const double age_sec = (this->now() - cmd_stamp).seconds();
        if (age_sec > command_stale_sec_) {
            return true;
        }
    }

    return false;
}

std::string PathPlanner::heartbeatInfo() const
{
    // ABORTED is always a fail-safe / help-required status.
    if (mode_ == PlannerMode::ABORTED) {
        return "EVENT_NEED_HELP";
    }

    // Transit recording depends on valid odometry.
    if (mode_ == PlannerMode::TRANSIT_RECORD && !hasValidOdom()) {
        return "EVENT_STUCK";
    }

    // Active mission modes require valid odom + map input.
    const bool active_mode = (mode_ == PlannerMode::EXPLORE ||
                              mode_ == PlannerMode::BACKTRACK ||
                              mode_ == PlannerMode::RETURN_HOME);
    if (active_mode && !hasValidPlanningInputs()) {
        return "EVENT_STUCK";
    }

    return toHeartbeatInfo(mode_);
}

bool PathPlanner::isPlanningFrame(const std::string &frame_id) const
{
    if (frame_id.empty()) {
        return false;
    }

    auto normalize = [](const std::string &s) {
        if (!s.empty() && s.front() == '/') {
            return s.substr(1);
        }
        return s;
    };

    return normalize(frame_id) == normalize(planning_frame_);
}

bool PathPlanner::isFresh(const rclcpp::Time &stamp) const
{
    if (stamp.nanoseconds() == 0) {
        return false;
    }
    if (input_timeout_sec_ <= 0.0) {
        return true;
    }

    const double age_sec = (this->now() - stamp).seconds();
    return age_sec <= input_timeout_sec_;
}

bool PathPlanner::hasValidOdom() const
{
    return has_odom_ && odom_frame_ok_ && isFresh(last_odom_time_);
}

bool PathPlanner::hasValidMap() const
{
    if (!(has_map_ && map_frame_ok_ && static_cast<bool>(octree_))) {
        return false;
    }

    // Octomap updates can be sparse; map freshness check is optional by parameter.
    if (map_timeout_sec_ <= 0.0) {
        return true;
    }

    if (last_map_time_.nanoseconds() == 0) {
        return false;
    }

    const double age_sec = (this->now() - last_map_time_).seconds();
    return age_sec <= map_timeout_sec_;
}

bool PathPlanner::hasValidPlanningInputs() const
{
    return hasValidOdom() && hasValidMap();
}

bool PathPlanner::canPublishPlannerOutput() const
{
    const bool active_mode = (mode_ == PlannerMode::EXPLORE ||
                              mode_ == PlannerMode::BACKTRACK ||
                              mode_ == PlannerMode::RETURN_HOME);
    return active_mode && hasValidPlanningInputs();
}

bool PathPlanner::isOccupied(const geometry_msgs::msg::Point &point) const
{
    if (!hasValidMap()) {
        return false;
    }

    auto rawOccupied = [this](const geometry_msgs::msg::Point &q) {
        const octomap::point3d query(static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z));
        auto *node = octree_->search(query);
        return node && octree_->isNodeOccupied(node);
    };

    if (rawOccupied(point)) {
        return true;
    }

    const double inflation = std::max(0.0, inflation_m_);
    if (inflation <= 1e-6) {
        return false;
    }

    const double step = std::max(static_cast<double>(octree_->getResolution()), 0.1);
    for (double dx = -inflation; dx <= inflation + 1e-6; dx += step) {
        for (double dy = -inflation; dy <= inflation + 1e-6; dy += step) {
            for (double dz = -inflation; dz <= inflation + 1e-6; dz += step) {
                if ((dx * dx + dy * dy + dz * dz) > (inflation * inflation)) {
                    continue;
                }
                geometry_msgs::msg::Point q;
                q.x = point.x + dx;
                q.y = point.y + dy;
                q.z = point.z + dz;
                if (rawOccupied(q)) {
                    return true;
                }
            }
        }
    }

    return false;
}

bool PathPlanner::isUnknown(const geometry_msgs::msg::Point &point) const
{
    if (!hasValidMap()) {
        return true;
    }

    const octomap::point3d query(static_cast<float>(point.x), static_cast<float>(point.y),
                                 static_cast<float>(point.z));
    const auto *node = octree_->search(query);
    return node == nullptr;
}

bool PathPlanner::isFree(const geometry_msgs::msg::Point &point) const
{
    if (!hasValidMap()) {
        return false;
    }
    if (isUnknown(point) || isOccupied(point)) {
        return false;
    }
    if (min_clearance_m_ > 0.0 && clearance(point) < min_clearance_m_) {
        return false;
    }
    return true;
}

bool PathPlanner::raycast(const geometry_msgs::msg::Point &origin, const geometry_msgs::msg::Point &target,
                          geometry_msgs::msg::Point &hit_out, bool stop_on_unknown) const
{
    if (!hasValidMap()) {
        return false;
    }

    const double dx = target.x - origin.x;
    const double dy = target.y - origin.y;
    const double dz = target.z - origin.z;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist <= 1e-6) {
        return false;
    }

    const double step = std::max(static_cast<double>(octree_->getResolution()), 0.1);
    const double ux = dx / dist;
    const double uy = dy / dist;
    const double uz = dz / dist;

    for (double s = step; s <= dist + 1e-6; s += step) {
        geometry_msgs::msg::Point p;
        p.x = origin.x + ux * s;
        p.y = origin.y + uy * s;
        p.z = origin.z + uz * s;

        if (isOccupied(p) || (stop_on_unknown && isUnknown(p))) {
            hit_out = p;
            return true;
        }
    }

    return false;
}

double PathPlanner::clearance(const geometry_msgs::msg::Point &point) const
{
    if (!hasValidMap()) {
        return 0.0;
    }

    auto rawOccupied = [this](const geometry_msgs::msg::Point &q) {
        const octomap::point3d query(static_cast<float>(q.x), static_cast<float>(q.y), static_cast<float>(q.z));
        auto *node = octree_->search(query);
        return node && octree_->isNodeOccupied(node);
    };

    if (rawOccupied(point)) {
        return 0.0;
    }

    const double step = std::max(0.1, clearance_step_m_);
    const double max_range = std::max(step, clearance_search_max_m_);
    const double inflation = std::max(0.0, inflation_m_);

    static const std::array<std::array<double, 3>, 26> kDirs{{
        {{1, 0, 0}},  {{-1, 0, 0}}, {{0, 1, 0}},  {{0, -1, 0}}, {{0, 0, 1}},  {{0, 0, -1}},
        {{1, 1, 0}},  {{1, -1, 0}}, {{-1, 1, 0}}, {{-1, -1, 0}}, {{1, 0, 1}},  {{1, 0, -1}},
        {{-1, 0, 1}}, {{-1, 0, -1}}, {{0, 1, 1}}, {{0, 1, -1}}, {{0, -1, 1}}, {{0, -1, -1}},
        {{1, 1, 1}},  {{1, 1, -1}}, {{1, -1, 1}}, {{1, -1, -1}}, {{-1, 1, 1}}, {{-1, 1, -1}},
        {{-1, -1, 1}}, {{-1, -1, -1}}
    }};

    for (double r = step; r <= max_range + 1e-6; r += step) {
        for (const auto &d : kDirs) {
            const double n = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            geometry_msgs::msg::Point q;
            q.x = point.x + (d[0] / n) * r;
            q.y = point.y + (d[1] / n) * r;
            q.z = point.z + (d[2] / n) * r;

            if (rawOccupied(q)) {
                return std::max(0.0, r - inflation);
            }
        }
    }

    return max_range;
}

std::optional<int> PathPlanner::findNearestNodeId(const geometry_msgs::msg::Point &point, double radius_m) const
{
    if (graph_nodes_.empty()) {
        return std::nullopt;
    }

    const double radius = std::max(0.0, radius_m);
    bool found = false;
    int best_id = -1;
    double best_dist = std::numeric_limits<double>::max();

    for (const auto &node : graph_nodes_) {
        if (!node.valid) {
            continue;
        }

        const double d = distance3(node.position, point);
        if (d > radius) {
            continue;
        }

        if (!found || d < best_dist || (std::abs(d - best_dist) <= 1e-9 && node.id < best_id)) {
            found = true;
            best_dist = d;
            best_id = node.id;
        }
    }

    if (!found) {
        return std::nullopt;
    }
    return best_id;
}

std::vector<int> PathPlanner::findNearbyNodeIds(const geometry_msgs::msg::Point &point, double radius_m) const
{
    std::vector<int> ids;
    const double radius = std::max(0.0, radius_m);
    for (const auto &node : graph_nodes_) {
        if (!node.valid) {
            continue;
        }
        if (distance3(node.position, point) <= radius) {
            ids.push_back(node.id);
        }
    }
    std::sort(ids.begin(), ids.end());
    return ids;
}

int PathPlanner::addOrMergeGraphNode(const geometry_msgs::msg::Point &point, bool is_transit, bool &inserted_new)
{
    inserted_new = false;
    const double merge_radius = std::max(0.05, graph_merge_radius_m_);
    const auto nearest_id = findNearestNodeId(point, merge_radius);

    if (nearest_id.has_value()) {
        auto *node = findNodeById(*nearest_id);
        if (!node) {
            return -1;
        }

        // Weighted update keeps duplicate suppression deterministic and stable.
        const double w_old = static_cast<double>(node->observations);
        const double w_new = w_old + 1.0;
        node->position.x = (node->position.x * w_old + point.x) / w_new;
        node->position.y = (node->position.y * w_old + point.y) / w_new;
        node->position.z = (node->position.z * w_old + point.z) / w_new;
        node->observations += 1;
        node->stamp = this->now();
        node->is_transit = (node->is_transit || is_transit);
        node->valid = true;
        return node->id;
    }

    GraphNode node;
    node.id = next_graph_node_id_++;
    node.position = point;
    node.stamp = this->now();
    node.is_transit = is_transit;
    graph_nodes_.push_back(node);
    inserted_new = true;
    return node.id;
}

void PathPlanner::upsertGraphEdge(int from_id, int to_id)
{
    if (from_id <= 0 || to_id <= 0 || from_id == to_id) {
        return;
    }

    const auto *from = findNodeById(from_id);
    const auto *to = findNodeById(to_id);
    if (!from || !to) {
        return;
    }

    // Store edges canonically as undirected pairs for deterministic duplicate handling.
    int a = from_id;
    int b = to_id;
    if (a > b) {
        std::swap(a, b);
    }

    const double length = distance3(from->position, to->position);
    const double cost = length;

    for (auto &edge : graph_edges_) {
        if (edge.from_id == a && edge.to_id == b) {
            edge.length_m = length;
            edge.cost = cost;
            edge.valid = true;
            return;
        }
    }

    GraphEdge edge;
    edge.from_id = a;
    edge.to_id = b;
    edge.length_m = length;
    edge.cost = cost;
    edge.valid = true;
    graph_edges_.push_back(edge);
}

bool PathPlanner::hasGraphEdge(int from_id, int to_id) const
{
    if (from_id <= 0 || to_id <= 0 || from_id == to_id) {
        return false;
    }

    int a = from_id;
    int b = to_id;
    if (a > b) {
        std::swap(a, b);
    }

    return std::any_of(graph_edges_.begin(), graph_edges_.end(), [&](const GraphEdge &edge) {
        return edge.valid && edge.from_id == a && edge.to_id == b;
    });
}

void PathPlanner::queueEventInfo(const std::string &event_info, rclcpp::Time &last_event_time)
{
    if (event_info.empty()) {
        return;
    }

    const auto now = this->now();
    if (event_cooldown_sec_ > 0.0 && last_event_time.nanoseconds() != 0) {
        const double since_last_sec = (now - last_event_time).seconds();
        if (since_last_sec < event_cooldown_sec_) {
            return;
        }
    }

    if (pending_event_info_.empty()) {
        pending_event_info_ = event_info;
    }
    last_event_time = now;
}

PathPlanner::GraphNode *PathPlanner::findNodeById(int id)
{
    for (auto &node : graph_nodes_) {
        if (node.id == id) {
            return &node;
        }
    }
    return nullptr;
}

const PathPlanner::GraphNode *PathPlanner::findNodeById(int id) const
{
    for (const auto &node : graph_nodes_) {
        if (node.id == id) {
            return &node;
        }
    }
    return nullptr;
}

double PathPlanner::yawFromQuaternion(double x, double y, double z, double w)
{
    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// #################################################################################################
// String Helpers
// #################################################################################################

std::string PathPlanner::toString(PlannerMode mode)
{
    switch (mode) {
    case PlannerMode::IDLE:
        return "IDLE";
    case PlannerMode::TRANSIT_RECORD:
        return "TRANSIT_RECORD";
    case PlannerMode::EXPLORE:
        return "EXPLORE";
    case PlannerMode::BACKTRACK:
        return "BACKTRACK";
    case PlannerMode::RETURN_HOME:
        return "RETURN_HOME";
    case PlannerMode::HOLD:
        return "HOLD";
    case PlannerMode::ABORTED:
        return "ABORTED";
    }
    return "UNKNOWN";
}

std::string PathPlanner::toHeartbeatInfo(PlannerMode mode)
{
    switch (mode) {
    case PlannerMode::IDLE:
        return "RUNNING_IDLE";
    case PlannerMode::TRANSIT_RECORD:
        return "RUNNING_TRANSIT_RECORD";
    case PlannerMode::EXPLORE:
        return "RUNNING_EXPLORE";
    case PlannerMode::BACKTRACK:
        return "RUNNING_BACKTRACK";
    case PlannerMode::RETURN_HOME:
        return "RUNNING_RETURN_HOME";
    case PlannerMode::HOLD:
        return "RUNNING_HOLD";
    case PlannerMode::ABORTED:
        return "EVENT_NEED_HELP";
    }
    return "RUNNING_IDLE";
}

std::string PathPlanner::toString(GraphNodeStatus status)
{
    switch (status) {
    case GraphNodeStatus::UNVISITED:
        return "UNVISITED";
    case GraphNodeStatus::VISITED:
        return "VISITED";
    case GraphNodeStatus::FRONTIER:
        return "FRONTIER";
    case GraphNodeStatus::DEAD_END:
        return "DEAD_END";
    }
    return "UNKNOWN";
}

std::string PathPlanner::toString(CandidateStatus status)
{
    switch (status) {
    case CandidateStatus::FREE:
        return "FREE";
    case CandidateStatus::BLOCKED:
        return "BLOCKED";
    case CandidateStatus::UNKNOWN:
        return "UNKNOWN";
    }
    return "UNKNOWN";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
