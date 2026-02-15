#include "path_planning_pkg/path_planner.h"

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

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

    // Publishers ###################################################################################
    pub_heartbeat_ = create_publisher<statemachine_pkg::msg::Answer>(topic_heartbeat_, 10);
    pub_trajectory_ =
        create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(topic_trajectory_, 10);

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
    if (!hasValidOdom()) {
        return;
    }

    const auto &pos = latest_odom_.pose.pose.position;

    // Step 1: Always record first node.
    if (transit_nodes_.empty()) {
        transit_nodes_.push_back(TransitNode{next_transit_node_id_++, pos, this->now()});
        return;
    }

    // Step 2: Record only if spacing threshold is reached.
    const auto &last = transit_nodes_.back().position;
    if (distance3(last, pos) < std::max(0.1, transit_node_spacing_m_)) {
        return;
    }

    transit_nodes_.push_back(TransitNode{next_transit_node_id_++, pos, this->now()});
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

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathPlanner>());
    rclcpp::shutdown();
    return 0;
}
