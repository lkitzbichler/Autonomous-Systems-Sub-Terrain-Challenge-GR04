#include "statemachine.h"

#include <cmath>
#include <fstream>
#include <limits>

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
    scan_duration_sec_ = declare_parameter<double>("scan_duration_sec", 8.0);
    return_timeout_sec_ = declare_parameter<double>("return_timeout_sec", 90.0);
    land_duration_sec_ = declare_parameter<double>("land_duration_sec", 5.0);
    mission_time_limit_sec_ = declare_parameter<double>("mission_time_limit_sec", 0.0);
    lantern_merge_dist_m_ = declare_parameter<double>("lantern_merge_dist_m", 1.0);
    lantern_log_path_ = declare_parameter<std::string>("lantern_log_path", "lanterns_log.csv");
    event_log_path_ = declare_parameter<std::string>("event_log_path", "statemachine_events.log");
    signal_timeout_sec_ = declare_parameter<double>("signal_timeout_sec", 5.0);

    pub_state_ = create_publisher<std_msgs::msg::String>("statemachine/state", 10);
    pub_cmd_basic_waypoint_ = create_publisher<std_msgs::msg::UInt8>("statemachine/cmd/basic_waypoint", 10);
    pub_cmd_path_planning_ = create_publisher<std_msgs::msg::UInt8>("statemachine/cmd/path_planning", 10);
    pub_cmd_mapping_ = create_publisher<std_msgs::msg::UInt8>("statemachine/cmd/mapping", 10);
    pub_cmd_controller_ = create_publisher<std_msgs::msg::UInt8>("statemachine/cmd/controller", 10);
    pub_lantern_target_ = create_publisher<geometry_msgs::msg::PoseStamped>("statemachine/lantern_target", 10);
    pub_lanterns_ = create_publisher<geometry_msgs::msg::PoseArray>("statemachine/lanterns", 10);

    sub_start_ = create_subscription<std_msgs::msg::Bool>(
        "statemachine/start", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (start_requested_ != msg->data) {
                logEvent(std::string("statemachine/start -> ") + (msg->data ? "true" : "false"));
            }
            start_requested_ = msg->data;
        });
    sub_abort_ = create_subscription<std_msgs::msg::Bool>(
        "statemachine/abort", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (abort_requested_ != msg->data) {
                logEvent(std::string("statemachine/abort -> ") + (msg->data ? "true" : "false"));
            }
            abort_requested_ = msg->data;
        });
    sub_mapping_ready_ = create_subscription<std_msgs::msg::Bool>(
        "mapping/ready", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (mapping_ready_ != msg->data) {
                logEvent(std::string("mapping/ready -> ") + (msg->data ? "true" : "false"));
            }
            mapping_ready_ = msg->data;
            last_mapping_ready_ = this->now();
            mapping_stale_reported_ = false;
        });
    sub_path_ready_ = create_subscription<std_msgs::msg::Bool>(
        "path_planning/ready", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (path_ready_ != msg->data) {
                logEvent(std::string("path_planning/ready -> ") + (msg->data ? "true" : "false"));
            }
            path_ready_ = msg->data;
            last_path_ready_ = this->now();
            path_stale_reported_ = false;
        });
    sub_waypoint_done_ = create_subscription<std_msgs::msg::Bool>(
        "basic_waypoint/done", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (waypoint_done_ != msg->data) {
                logEvent(std::string("basic_waypoint/done -> ") + (msg->data ? "true" : "false"));
            }
            waypoint_done_ = msg->data;
            last_waypoint_done_ = this->now();
            waypoint_stale_reported_ = false;
        });
    sub_goal_reached_ = create_subscription<std_msgs::msg::Bool>(
        "path_planning/goal_reached", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (goal_reached_ != msg->data) {
                logEvent(std::string("path_planning/goal_reached -> ") + (msg->data ? "true" : "false"));
            }
            goal_reached_ = msg->data;
            last_goal_reached_ = this->now();
            goal_stale_reported_ = false;
        });
    sub_lantern_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "detection/lantern", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            pending_lantern_ = *msg;
            lantern_pending_ = true;
            last_lantern_ = this->now();
            lantern_stale_reported_ = false;
            logEvent("detection/lantern -> received pose");
        });

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
        } else if (lantern_pending_) {
            transitionTo(MissionState::SCAN_LANTERN, "lantern detected");
        }
        break;
    case MissionState::SCAN_LANTERN:
        if (state_elapsed >= scan_duration_sec_) {
            lantern_pending_ = false;
            transitionTo(MissionState::EXPLORE, "scan complete");
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
        logCommand("statemachine/cmd/controller", Command::STOP);
        pub_cmd_basic_waypoint_->publish(cmd_msg);
        logCommand("statemachine/cmd/basic_waypoint", Command::STOP);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand("statemachine/cmd/path_planning", Command::STOP);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand("statemachine/cmd/mapping", Command::STOP);
        break;
    case MissionState::TAKEOFF:
        cmd_msg.data = static_cast<uint8_t>(Command::TAKEOFF);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand("statemachine/cmd/controller", Command::TAKEOFF);
        break;
    case MissionState::GOTO_ENTRANCE:
        waypoint_done_ = false;
        cmd_msg.data = static_cast<uint8_t>(Command::START);
        pub_cmd_basic_waypoint_->publish(cmd_msg);
        logCommand("statemachine/cmd/basic_waypoint", Command::START);
        break;
    case MissionState::EXPLORE:
        goal_reached_ = false;
        cmd_msg.data = static_cast<uint8_t>(Command::START);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand("statemachine/cmd/path_planning", Command::START);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand("statemachine/cmd/mapping", Command::START);
        break;
    case MissionState::SCAN_LANTERN:
        cmd_msg.data = static_cast<uint8_t>(Command::HOLD);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand("statemachine/cmd/controller", Command::HOLD);
        cmd_msg.data = static_cast<uint8_t>(Command::STOP);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand("statemachine/cmd/path_planning", Command::STOP);
        cmd_msg.data = static_cast<uint8_t>(Command::SCAN);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand("statemachine/cmd/mapping", Command::SCAN);
        active_lantern_ = pending_lantern_;
        pub_lantern_target_->publish(active_lantern_);
        lanterns_.header.stamp = this->now();
        pub_lanterns_->publish(lanterns_);
        break;
    case MissionState::RETURN_HOME:
        goal_reached_ = false;
        cmd_msg.data = static_cast<uint8_t>(Command::RETURN_HOME);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand("statemachine/cmd/path_planning", Command::RETURN_HOME);
        break;
    case MissionState::LAND:
        cmd_msg.data = static_cast<uint8_t>(Command::LAND);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand("statemachine/cmd/controller", Command::LAND);
        break;
    case MissionState::DONE:
        cmd_msg.data = static_cast<uint8_t>(Command::STOP);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand("statemachine/cmd/controller", Command::STOP);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand("statemachine/cmd/path_planning", Command::STOP);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand("statemachine/cmd/mapping", Command::STOP);
        break;
    case MissionState::ERROR:
        cmd_msg.data = static_cast<uint8_t>(Command::STOP);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand("statemachine/cmd/controller", Command::STOP);
        pub_cmd_basic_waypoint_->publish(cmd_msg);
        logCommand("statemachine/cmd/basic_waypoint", Command::STOP);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand("statemachine/cmd/path_planning", Command::STOP);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand("statemachine/cmd/mapping", Command::STOP);
        break;
    case MissionState::ABORTED:
        cmd_msg.data = static_cast<uint8_t>(Command::STOP);
        pub_cmd_controller_->publish(cmd_msg);
        logCommand("statemachine/cmd/controller", Command::STOP);
        pub_cmd_basic_waypoint_->publish(cmd_msg);
        logCommand("statemachine/cmd/basic_waypoint", Command::STOP);
        pub_cmd_path_planning_->publish(cmd_msg);
        logCommand("statemachine/cmd/path_planning", Command::STOP);
        pub_cmd_mapping_->publish(cmd_msg);
        logCommand("statemachine/cmd/mapping", Command::STOP);
        break;
    case MissionState::BOOT:
        break;
    }
}

void StateMachine::logLanternPose(const geometry_msgs::msg::PoseStamped &pose)
{
    if (lantern_log_path_.empty()) {
        return;
    }

    bool file_empty = true;
    {
        std::ifstream in(lantern_log_path_, std::ios::ate | std::ios::binary);
        if (in.good() && in.tellg() > 0) {
            file_empty = false;
        }
    }

    std::ofstream out(lantern_log_path_, std::ios::app);
    if (!out) {
        RCLCPP_WARN(get_logger(), "Failed to open lantern log file: %s", lantern_log_path_.c_str());
        return;
    }

    if (file_empty) {
        out << "stamp_sec,x,y,z\n";
    }

    double stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9;
    if (stamp <= 0.0) {
        stamp = this->now().seconds();
    }

    out << stamp << ","
        << pose.pose.position.x << ","
        << pose.pose.position.y << ","
        << pose.pose.position.z << "\n";
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
        checkSignalTimeout("mapping/ready", last_mapping_ready_, mapping_stale_reported_);
        checkSignalTimeout("path_planning/ready", last_path_ready_, path_stale_reported_);
        break;
    case MissionState::GOTO_ENTRANCE:
        checkSignalTimeout("basic_waypoint/done", last_waypoint_done_, waypoint_stale_reported_);
        break;
    case MissionState::EXPLORE:
        checkSignalTimeout("path_planning/goal_reached", last_goal_reached_, goal_stale_reported_);
        checkSignalTimeout("detection/lantern", last_lantern_, lantern_stale_reported_);
        break;
    case MissionState::RETURN_HOME:
        checkSignalTimeout("path_planning/goal_reached", last_goal_reached_, goal_stale_reported_);
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
            logLanternPose(pending_lantern_);
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

std::string StateMachine::toString(MissionState state)
{
    switch (state) {
    case MissionState::BOOT: return "BOOT";
    case MissionState::WAIT_FOR_SYSTEM: return "WAIT_FOR_SYSTEM";
    case MissionState::TAKEOFF: return "TAKEOFF";
    case MissionState::GOTO_ENTRANCE: return "GOTO_ENTRANCE";
    case MissionState::EXPLORE: return "EXPLORE";
    case MissionState::SCAN_LANTERN: return "SCAN_LANTERN";
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
