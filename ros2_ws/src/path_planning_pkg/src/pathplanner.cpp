#include "path_planning_pkg/path_planner.h"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
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

    // Timing
    loop_period_sec_ = declare_parameter<double>("time.loop_period_sec", 0.2);
    heartbeat_period_sec_ = declare_parameter<double>("time.heartbeat_period_sec", 1.0);
    input_timeout_sec_ = declare_parameter<double>("time.input_timeout_sec", 1.0);
    command_stale_sec_ = declare_parameter<double>("time.command_stale_sec", 5.0);
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

    // Publishers ###################################################################################
    pub_heartbeat_ = create_publisher<statemachine_pkg::msg::Answer>(topic_heartbeat_, 10);
    pub_trajectory_ =
        create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(topic_trajectory_, 10);
    pub_graph_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(topic_graph_markers_, 10);

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

    // Step 3: Publish graph visualization for runtime inspection.
    publishGraphMarkers();
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
    hb.info = heartbeatInfo();

    const auto now = this->now();
    hb.timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
    hb.timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
    pub_heartbeat_->publish(hb);
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
            m.color.r = 1.00F;
            m.color.g = 0.50F;
            m.color.b = 0.10F;
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
    return has_map_ && map_frame_ok_ && static_cast<bool>(octree_) && isFresh(last_map_time_);
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
                          geometry_msgs::msg::Point &hit_out) const
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

        if (isOccupied(p) || isUnknown(p)) {
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
