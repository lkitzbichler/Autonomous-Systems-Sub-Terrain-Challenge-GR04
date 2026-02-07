#include "statemachine_pkg/statemachine.h"

#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace {

double distance3(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

StateMachine::StateMachine()
: rclcpp::Node("state_machine_node")
{
    // Topics the other packages should subscribe/publish to are all prefixed with
    // "statemachine/" for commands and status, and package-specific "/ready" or "/done" flags.
    auto_start_ = declare_parameter<bool>("auto_start", true);
    loop_hz_ = declare_parameter<double>("loop_hz", 10.0);
    startup_grace_sec_ = declare_parameter<double>("startup_grace_sec", 5.0);
    takeoff_duration_sec_ = declare_parameter<double>("takeoff_duration_sec", 3.0);
    entrance_timeout_sec_ = declare_parameter<double>("entrance_timeout_sec", 60.0);
    return_timeout_sec_ = declare_parameter<double>("return_timeout_sec", 90.0);
    land_duration_sec_ = declare_parameter<double>("land_duration_sec", 5.0);
    mission_time_limit_sec_ = declare_parameter<double>("mission_time_limit_sec", 0.0);
    lantern_merge_dist_m_ = declare_parameter<double>("lantern_merge_dist_m", 1.0);
    lantern_log_path_ = declare_parameter<std::string>("lantern_log_path", "lanterns_log.csv");
    event_log_path_ = declare_parameter<std::string>("event_log_path", "statemachine_events.log");
    username_ = declare_parameter<std::string>("username", "unknown");
    if (username_.empty() || username_ == "unknown") {
        const char *env_user = std::getenv("USER");
        if (!env_user || std::string(env_user).empty()) {
            env_user = std::getenv("USERNAME");
        }
        if (env_user && *env_user) {
            username_ = env_user;
        }
    }
    signal_timeout_sec_ = declare_parameter<double>("signal_timeout_sec", 5.0);

    topic_state_ = declare_parameter<std::string>("topic_state", "statemachine/state");
    topic_cmd_basic_waypoint_ = declare_parameter<std::string>("topic_cmd_basic_waypoint", "statemachine/cmd/basic_waypoint");
    topic_cmd_path_planning_ = declare_parameter<std::string>("topic_cmd_path_planning", "statemachine/cmd/path_planning");
    topic_cmd_mapping_ = declare_parameter<std::string>("topic_cmd_mapping", "statemachine/cmd/mapping");
    topic_cmd_controller_ = declare_parameter<std::string>("topic_cmd_controller", "statemachine/cmd/controller");
    topic_cmd_lantern_detector_ = declare_parameter<std::string>("topic_cmd_lantern_detector", "statemachine/cmd/lantern_detector");
    topic_start_ = declare_parameter<std::string>("topic_start", "statemachine/start");
    topic_abort_ = declare_parameter<std::string>("topic_abort", "statemachine/abort");
    topic_mapping_ready_ = declare_parameter<std::string>("topic_mapping_ready", "mapping/ready");
    topic_path_ready_ = declare_parameter<std::string>("topic_path_ready", "path_planning/ready");
    topic_waypoint_done_ = declare_parameter<std::string>("topic_waypoint_done", "basic_waypoint/done");
    topic_goal_reached_ = declare_parameter<std::string>("topic_goal_reached", "path_planning/goal_reached");
    topic_lantern_detections_ = declare_parameter<std::string>("topic_lantern_detections", "detected_lanterns");

    pub_state_ = create_publisher<std_msgs::msg::String>(topic_state_, 10);
    pub_cmd_basic_waypoint_ = create_publisher<std_msgs::msg::UInt8>(topic_cmd_basic_waypoint_, 10);
    pub_cmd_path_planning_ = create_publisher<std_msgs::msg::UInt8>(topic_cmd_path_planning_, 10);
    pub_cmd_mapping_ = create_publisher<std_msgs::msg::UInt8>(topic_cmd_mapping_, 10);
    pub_cmd_controller_ = create_publisher<std_msgs::msg::UInt8>(topic_cmd_controller_, 10);
    pub_cmd_lantern_detector_ = create_publisher<std_msgs::msg::UInt8>(topic_cmd_lantern_detector_, 10);

    sub_start_ = create_subscription<std_msgs::msg::Bool>(
        topic_start_, 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (start_requested_ != msg->data) {
                logEvent(topic_start_ + " -> " + (msg->data ? "true" : "false"));
            }
            start_requested_ = msg->data;
        });
    sub_abort_ = create_subscription<std_msgs::msg::Bool>(
        topic_abort_, 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (abort_requested_ != msg->data) {
                logEvent(topic_abort_ + " -> " + (msg->data ? "true" : "false"));
            }
            abort_requested_ = msg->data;
        });
    sub_mapping_ready_ = create_subscription<std_msgs::msg::Bool>(
        topic_mapping_ready_, 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (mapping_ready_ != msg->data) {
                logEvent(topic_mapping_ready_ + " -> " + (msg->data ? "true" : "false"));
            }
            mapping_ready_ = msg->data;
            last_mapping_ready_ = this->now();
            mapping_stale_reported_ = false;
        });
    sub_path_ready_ = create_subscription<std_msgs::msg::Bool>(
        topic_path_ready_, 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (path_ready_ != msg->data) {
                logEvent(topic_path_ready_ + " -> " + (msg->data ? "true" : "false"));
            }
            path_ready_ = msg->data;
            last_path_ready_ = this->now();
            path_stale_reported_ = false;
        });
    sub_waypoint_done_ = create_subscription<std_msgs::msg::Bool>(
        topic_waypoint_done_, 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (waypoint_done_ != msg->data) {
                logEvent(topic_waypoint_done_ + " -> " + (msg->data ? "true" : "false"));
            }
            waypoint_done_ = msg->data;
            last_waypoint_done_ = this->now();
            waypoint_stale_reported_ = false;
        });
    sub_goal_reached_ = create_subscription<std_msgs::msg::Bool>(
        topic_goal_reached_, 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (goal_reached_ != msg->data) {
                logEvent(topic_goal_reached_ + " -> " + (msg->data ? "true" : "false"));
            }
            goal_reached_ = msg->data;
            last_goal_reached_ = this->now();
            goal_stale_reported_ = false;
        });
    sub_lantern_ = create_subscription<geometry_msgs::msg::PoseArray>(
        topic_lantern_detections_, 10,
        std::bind(&StateMachine::onLanternDetections, this, std::placeholders::_1));

    lanterns_.header.frame_id = "world";

    transitionTo(MissionState::BOOT, "node start");
    logEvent("state machine started");

    const auto period = std::chrono::duration<double>(1.0 / loop_hz_);
    timer_ = create_wall_timer(period, std::bind(&StateMachine::onTimer, this));
}

void StateMachine::onTimer()
{
    if (abort_requested_ && state_ != MissionState::ABORTED && state_ != MissionState::DONE) {
        transitionTo(MissionState::ABORTED, "abort requested");
        return;
    }

    const auto now = this->now();
    const auto state_elapsed = (now - state_enter_time_).seconds();

    if (mission_start_time_.nanoseconds() > 0 && mission_time_limit_sec_ > 0.0) {
        const auto mission_elapsed = (now - mission_start_time_).seconds();
        if (mission_elapsed >= mission_time_limit_sec_ &&
            state_ != MissionState::RETURN_HOME &&
            state_ != MissionState::LAND &&
            state_ != MissionState::DONE) {
            transitionTo(MissionState::RETURN_HOME, "mission time limit reached");
            return;
        }
    }

    switch (state_) {
    case MissionState::BOOT:
        transitionTo(MissionState::WAIT_FOR_SYSTEM, "boot complete");
        break;
    case MissionState::WAIT_FOR_SYSTEM: {
        const bool ready = mapping_ready_ && path_ready_;
        const bool grace_elapsed = state_elapsed >= startup_grace_sec_;
        const bool start_ok = start_requested_ || auto_start_;
        if (start_ok && (ready || grace_elapsed)) {
            transitionTo(MissionState::TAKEOFF, "system ready");
        }
        break;
    }
    case MissionState::TAKEOFF:
        if (state_elapsed >= takeoff_duration_sec_) {
            transitionTo(MissionState::GOTO_ENTRANCE, "takeoff done");
        }
        break;
    case MissionState::GOTO_ENTRANCE:
        if (waypoint_done_) {
            transitionTo(MissionState::EXPLORE, "entrance reached");
        } else if (state_elapsed >= entrance_timeout_sec_) {
            transitionTo(MissionState::ERROR, "entrance timeout");
        }
        break;
    case MissionState::EXPLORE:
        if (goal_reached_) {
            transitionTo(MissionState::RETURN_HOME, "exploration goal reached");
        }
        break;
    case MissionState::RETURN_HOME:
        if (state_elapsed >= return_timeout_sec_ || goal_reached_) {
            transitionTo(MissionState::LAND, "return completed");
        }
        break;
    case MissionState::LAND:
        if (state_elapsed >= land_duration_sec_) {
            transitionTo(MissionState::DONE, "landed");
        }
        break;
    case MissionState::DONE:
    case MissionState::ERROR:
    case MissionState::ABORTED:
        break;
    }

    publishState();
    checkSignalTimeouts();
}

void StateMachine::transitionTo(MissionState next_state, const std::string &reason)
{
    if (state_ == next_state) {
        return;
    }

    state_ = next_state;
    state_enter_time_ = this->now();

    if (mission_start_time_.nanoseconds() == 0 && next_state == MissionState::TAKEOFF) {
        mission_start_time_ = state_enter_time_;
    }

    RCLCPP_INFO(get_logger(), "State -> %s (%s)", toString(state_).c_str(), reason.c_str());
    logEvent(std::string("State -> ") + toString(state_) + " (" + reason + ")");
    handleStateEntry(state_);
}

void StateMachine::handleStateEntry(MissionState state)
{
    std_msgs::msg::UInt8 cmd_msg;
    cmd_msg.data = static_cast<uint8_t>(Command::IDLE);

    switch (state) {
    case MissionState::WAIT_FOR_SYSTEM:
        cmd_msg.data = static_cast<uint8_t>(Command::STOP);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand(topic_cmd_controller_, Command::STOP);
        pub_cmd_basic_waypoint_->publish(cmd_msg);
        logCommand(topic_cmd_basic_waypoint_, Command::STOP);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand(topic_cmd_path_planning_, Command::STOP);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand(topic_cmd_mapping_, Command::STOP);
        pub_cmd_lantern_detector_->publish(cmd_msg);
        logCommand(topic_cmd_lantern_detector_, Command::STOP);
        break;
    case MissionState::TAKEOFF:
        cmd_msg.data = static_cast<uint8_t>(Command::TAKEOFF);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand(topic_cmd_controller_, Command::TAKEOFF);
        break;
    case MissionState::GOTO_ENTRANCE:
        waypoint_done_ = false;
        cmd_msg.data = static_cast<uint8_t>(Command::START);
        pub_cmd_basic_waypoint_->publish(cmd_msg);
        logCommand(topic_cmd_basic_waypoint_, Command::START);
        break;
    case MissionState::EXPLORE:
        goal_reached_ = false;
        cmd_msg.data = static_cast<uint8_t>(Command::START);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand(topic_cmd_path_planning_, Command::START);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand(topic_cmd_mapping_, Command::START);
        cmd_msg.data = static_cast<uint8_t>(Command::SCAN);
        pub_cmd_lantern_detector_->publish(cmd_msg);
        logCommand(topic_cmd_lantern_detector_, Command::SCAN);
        break;
    case MissionState::RETURN_HOME:
        goal_reached_ = false;
        cmd_msg.data = static_cast<uint8_t>(Command::RETURN_HOME);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand(topic_cmd_path_planning_, Command::RETURN_HOME);
        break;
    case MissionState::LAND:
        cmd_msg.data = static_cast<uint8_t>(Command::LAND);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand(topic_cmd_controller_, Command::LAND);
        break;
    case MissionState::DONE:
        cmd_msg.data = static_cast<uint8_t>(Command::STOP);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand(topic_cmd_controller_, Command::STOP);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand(topic_cmd_path_planning_, Command::STOP);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand(topic_cmd_mapping_, Command::STOP);
        pub_cmd_lantern_detector_->publish(cmd_msg);
        logCommand(topic_cmd_lantern_detector_, Command::STOP);
        break;
    case MissionState::ERROR:
        cmd_msg.data = static_cast<uint8_t>(Command::STOP);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand(topic_cmd_controller_, Command::STOP);
        pub_cmd_basic_waypoint_->publish(cmd_msg);
        logCommand(topic_cmd_basic_waypoint_, Command::STOP);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand(topic_cmd_path_planning_, Command::STOP);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand(topic_cmd_mapping_, Command::STOP);
        pub_cmd_lantern_detector_->publish(cmd_msg);
        logCommand(topic_cmd_lantern_detector_, Command::STOP);
        break;
    case MissionState::ABORTED:
        cmd_msg.data = static_cast<uint8_t>(Command::STOP);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand(topic_cmd_controller_, Command::STOP);
        pub_cmd_basic_waypoint_->publish(cmd_msg);
        logCommand(topic_cmd_basic_waypoint_, Command::STOP);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand(topic_cmd_path_planning_, Command::STOP);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand(topic_cmd_mapping_, Command::STOP);
        pub_cmd_lantern_detector_->publish(cmd_msg);
        logCommand(topic_cmd_lantern_detector_, Command::STOP);
        break;
    case MissionState::BOOT:
        break;
    }
}

void StateMachine::logLanternPose(const geometry_msgs::msg::PoseStamped &pose, int id, size_t count)
{
    if (lantern_log_path_.empty()) {
        return;
    }

    std::vector<std::string> lines;
    {
        std::ifstream in(lantern_log_path_);
        std::string line;
        if (in.good()) {
            while (std::getline(in, line)) {
                if (line.empty()) {
                    continue;
                }
                if (line.rfind("stamp_sec,", 0) == 0 ||
                    line.rfind("timestamp,", 0) == 0 ||
                    line.rfind("id,zeit,", 0) == 0) {
                    continue;
                }
                lines.push_back(line);
            }
        }
    }

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

    const std::string header = "id,zeit,username,anzahl,x,y,z";
    lines.insert(lines.begin(), header);

    const std::string id_str = std::to_string(id);
    const std::string date_str = date.str();
    const std::string new_line = id_str + "," + date_str + "," +
        username_ + "," + std::to_string(count) + "," +
        std::to_string(pose.pose.position.x) + "," +
        std::to_string(pose.pose.position.y) + "," +
        std::to_string(pose.pose.position.z);

    bool updated = false;
    for (size_t i = 1; i < lines.size(); ++i) {
        const auto &row = lines[i];
        const auto first_comma = row.find(',');
        if (first_comma == std::string::npos) {
            continue;
        }
        const auto second_comma = row.find(',', first_comma + 1);
        if (second_comma == std::string::npos) {
            continue;
        }
        const auto third_comma = row.find(',', second_comma + 1);
        if (third_comma == std::string::npos) {
            continue;
        }
        const std::string row_id = row.substr(0, first_comma);
        const std::string row_date = row.substr(first_comma + 1, second_comma - first_comma - 1);
        const std::string row_user = row.substr(second_comma + 1, third_comma - second_comma - 1);
        if (row_id == id_str && row_date == date_str && row_user == username_) {
            lines[i] = new_line;
            updated = true;
            break;
        }
    }

    if (!updated) {
        lines.push_back(new_line);
    }

    std::ofstream out(lantern_log_path_, std::ios::trunc);
    if (!out) {
        RCLCPP_WARN(get_logger(), "Failed to open lantern log file: %s", lantern_log_path_.c_str());
        return;
    }

    for (const auto &row : lines) {
        out << row << "\n";
    }
}

void StateMachine::logEvent(const std::string &message)
{
    if (event_log_path_.empty()) {
        return;
    }

    std::ofstream out(event_log_path_, std::ios::app);
    if (!out) {
        RCLCPP_WARN(get_logger(), "Failed to open event log file: %s", event_log_path_.c_str());
        return;
    }

    out << this->now().seconds() << " | " << message << "\n";
}

void StateMachine::logCommand(const std::string &topic, Command cmd)
{
    logEvent(std::string("publish ") + topic + " -> " + toString(cmd));
}

void StateMachine::checkSignalTimeouts()
{
    if (signal_timeout_sec_ <= 0.0) {
        return;
    }

    switch (state_) {
    case MissionState::WAIT_FOR_SYSTEM:
        checkSignalTimeout(topic_mapping_ready_, last_mapping_ready_, mapping_stale_reported_);
        checkSignalTimeout(topic_path_ready_, last_path_ready_, path_stale_reported_);
        break;
    case MissionState::GOTO_ENTRANCE:
        checkSignalTimeout(topic_waypoint_done_, last_waypoint_done_, waypoint_stale_reported_);
        break;
    case MissionState::EXPLORE:
        checkSignalTimeout(topic_goal_reached_, last_goal_reached_, goal_stale_reported_);
        checkSignalTimeout(topic_lantern_detections_, last_lantern_, lantern_stale_reported_);
        break;
    case MissionState::RETURN_HOME:
        checkSignalTimeout(topic_goal_reached_, last_goal_reached_, goal_stale_reported_);
        break;
    default:
        break;
    }
}

void StateMachine::checkSignalTimeout(const std::string &name, const rclcpp::Time &last_time, bool &reported)
{
    if (reported) {
        return;
    }

    const auto now = this->now();
    const auto elapsed_since_state = (now - state_enter_time_).seconds();
    if (elapsed_since_state < signal_timeout_sec_) {
        return;
    }

    if (last_time.nanoseconds() == 0) {
        logEvent(name + " -> no signal received (timeout)");
        reported = true;
        return;
    }

    const auto elapsed = (now - last_time).seconds();
    if (elapsed >= signal_timeout_sec_) {
        logEvent(name + " -> stale (timeout)");
        reported = true;
    }
}

void StateMachine::publishState()
{
    if (lantern_pending_) {
        bool is_new = true;
        for (const auto &pose : lanterns_.poses) {
            if (distance3(pose.position, pending_lantern_.pose.position) <= lantern_merge_dist_m_) {
                is_new = false;
                break;
            }
        }
        if (is_new) {
            lanterns_.poses.push_back(pending_lantern_.pose);
            RCLCPP_INFO(get_logger(),
                "Lantern detected at [%.2f, %.2f, %.2f] -> %s",
                pending_lantern_.pose.position.x,
                pending_lantern_.pose.position.y,
                pending_lantern_.pose.position.z,
                lantern_log_path_.c_str());
            logEvent("lantern stored in list");
        }
    }

    std_msgs::msg::String msg;
    msg.data = toString(state_);
    pub_state_->publish(msg);
}

void StateMachine::onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    if (!msg) {
        return;
    }

    if (msg->poses.empty()) {
        logEvent(topic_lantern_detections_ + " -> received empty pose array");
        return;
    }

    if (!msg->header.frame_id.empty()) {
        lanterns_.header.frame_id = msg->header.frame_id;
    }

    logEvent(topic_lantern_detections_ + " -> received pose array (size=" + std::to_string(msg->poses.size()) + ")");

    bool any_new = false;
    bool pending_set = false;
    for (const auto &pose : msg->poses) {
        bool is_new = false;
        geometry_msgs::msg::Point mean;
        size_t count = 0;
        const int id = associateLantern(pose.position, is_new, mean, count);

        geometry_msgs::msg::PoseStamped new_pose;
        new_pose.header = msg->header;
        new_pose.pose.position = mean;
        new_pose.pose.orientation.w = 1.0;
        logLanternPose(new_pose, id, count);

        if (is_new) {
            any_new = true;
            logEvent("lantern id " + std::to_string(id) + " created");
        }

        logEvent(
            topic_lantern_detections_ + " -> received pose array of lantern " + std::to_string(id) +
            " at [" + std::to_string(pose.position.x) + ", " +
            std::to_string(pose.position.y) + ", " +
            std::to_string(pose.position.z) + "]");
        if (is_new && !pending_set) {
            pending_lantern_.header = msg->header;
            pending_lantern_.pose.position = mean;
            pending_lantern_.pose.orientation.w = 1.0;
            pending_set = true;
        }
    }

    lanterns_.poses.clear();
    for (const auto &track : lantern_tracks_) {
        geometry_msgs::msg::Pose mean_pose;
        mean_pose.position = track.mean;
        mean_pose.orientation.w = 1.0;
        lanterns_.poses.push_back(mean_pose);
    }

    lantern_pending_ = any_new;
    last_lantern_ = this->now();
    lantern_stale_reported_ = false;
}

int StateMachine::associateLantern(const geometry_msgs::msg::Point &pos, bool &is_new, geometry_msgs::msg::Point &mean_out, size_t &count_out)
{
    int best_index = -1;
    double best_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < lantern_tracks_.size(); ++i) {
        const double dist = distance3(lantern_tracks_[i].mean, pos);
        if (dist <= lantern_merge_dist_m_ && dist < best_dist) {
            best_dist = dist;
            best_index = static_cast<int>(i);
        }
    }

    if (best_index < 0) {
        LanternTrack track;
        track.id = next_lantern_id_++;
        track.mean = pos;
        track.samples.push_back(pos);
        track.count = 1;
        lantern_tracks_.push_back(track);
        is_new = true;
        mean_out = pos;
        count_out = track.count;
        return track.id;
    }

    auto &track = lantern_tracks_[best_index];
    track.mean.x = (track.mean.x * track.count + pos.x) / (track.count + 1);
    track.mean.y = (track.mean.y * track.count + pos.y) / (track.count + 1);
    track.mean.z = (track.mean.z * track.count + pos.z) / (track.count + 1);
    track.count++;
    track.samples.push_back(pos);

    is_new = false;
    mean_out = track.mean;
    count_out = track.count;
    return track.id;
}

std::string StateMachine::toString(MissionState state)
{
    switch (state) {
    case MissionState::BOOT: return "BOOT";
    case MissionState::WAIT_FOR_SYSTEM: return "WAIT_FOR_SYSTEM";
    case MissionState::TAKEOFF: return "TAKEOFF";
    case MissionState::GOTO_ENTRANCE: return "GOTO_ENTRANCE";
    case MissionState::EXPLORE: return "EXPLORE";
    case MissionState::RETURN_HOME: return "RETURN_HOME";
    case MissionState::LAND: return "LAND";
    case MissionState::DONE: return "DONE";
    case MissionState::ERROR: return "ERROR";
    case MissionState::ABORTED: return "ABORTED";
    }
    return "UNKNOWN";
}

std::string StateMachine::toString(Command cmd)
{
    switch (cmd) {
    case Command::IDLE: return "IDLE";
    case Command::START: return "START";
    case Command::STOP: return "STOP";
    case Command::HOLD: return "HOLD";
    case Command::TAKEOFF: return "TAKEOFF";
    case Command::LAND: return "LAND";
    case Command::RETURN_HOME: return "RETURN_HOME";
    case Command::SCAN: return "SCAN";
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
