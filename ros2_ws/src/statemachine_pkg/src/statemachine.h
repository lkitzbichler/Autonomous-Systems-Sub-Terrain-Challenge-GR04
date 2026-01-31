#pragma once

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

class StateMachine : public rclcpp::Node
{
public:
    StateMachine();

private:
    struct LanternTrack {
        int id{0};
        geometry_msgs::msg::Point mean;
        std::vector<geometry_msgs::msg::Point> samples;
        size_t count{0};
    };

    // High-level mission phases for the whole system.
    enum class MissionState {
        BOOT,             // Nodes booting
        WAIT_FOR_SYSTEM,  // Waiting for ready signals and start command
        TAKEOFF,          // Takeoff from start position
        GOTO_ENTRANCE,    // Use basic_waypoint to reach cave entrance
        EXPLORE,          // Autonomous exploration (path planning)
        SCAN_LANTERN,     // Pause flight, scan lamp, store pose
        RETURN_HOME,      // Return to exit/home position
        LAND,             // Land and stop
        DONE,             // Mission finished
        ERROR,            // Error state
        ABORTED           // Manual abort
    };

    // Generic command codes that other packages can interpret.
    enum class Command : uint8_t {
        IDLE = 0,
        START = 1,
        STOP = 2,
        HOLD = 3,
        TAKEOFF = 4,
        LAND = 5,
        RETURN_HOME = 6,
        SCAN = 7
    };

    void onTimer();
    void transitionTo(MissionState next_state, const std::string &reason);
    void handleStateEntry(MissionState state);
    void publishState();
    void logLanternPose(const geometry_msgs::msg::PoseStamped &pose, int id);
    void logEvent(const std::string &message);
    void logCommand(const std::string &topic, Command cmd);
    void checkSignalTimeouts();
    void checkSignalTimeout(const std::string &name, const rclcpp::Time &last_time, bool &reported);
    void onLanternDetections(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    int associateLantern(const geometry_msgs::msg::Point &pos, bool &is_new, geometry_msgs::msg::Point &mean_out);

    static std::string toString(MissionState state);
    static std::string toString(Command cmd);

    // ROS publishers (commands + state).
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_cmd_basic_waypoint_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_cmd_path_planning_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_cmd_mapping_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_cmd_controller_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_lantern_target_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_lanterns_;

    // ROS subscribers (events + ready signals).
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_abort_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mapping_ready_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_path_ready_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_waypoint_done_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_goal_reached_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_lantern_;

    rclcpp::TimerBase::SharedPtr timer_;

    // State machine variables.
    MissionState state_{MissionState::BOOT};
    rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time mission_start_time_{0, 0, RCL_ROS_TIME};

    bool start_requested_{false};
    bool abort_requested_{false};
    bool mapping_ready_{false};
    bool path_ready_{false};
    bool waypoint_done_{false};
    bool goal_reached_{false};

    geometry_msgs::msg::PoseStamped pending_lantern_;
    bool lantern_pending_{false};
    geometry_msgs::msg::PoseStamped active_lantern_;
    geometry_msgs::msg::PoseArray lanterns_;
    std::vector<LanternTrack> lantern_tracks_;
    int next_lantern_id_{1};

    // Parameters.
    bool auto_start_{true};
    double loop_hz_{10.0};
    double startup_grace_sec_{5.0};
    double takeoff_duration_sec_{3.0};
    double entrance_timeout_sec_{60.0};
    double scan_duration_sec_{8.0};
    double return_timeout_sec_{90.0};
    double land_duration_sec_{5.0};
    double mission_time_limit_sec_{0.0};
    double lantern_merge_dist_m_{1.0};
    std::string lantern_log_path_;
    std::string event_log_path_;
    double signal_timeout_sec_{5.0};

    // Topic parameters.
    std::string topic_state_;
    std::string topic_cmd_basic_waypoint_;
    std::string topic_cmd_path_planning_;
    std::string topic_cmd_mapping_;
    std::string topic_cmd_controller_;
    std::string topic_lantern_target_;
    std::string topic_lanterns_;
    std::string topic_start_;
    std::string topic_abort_;
    std::string topic_mapping_ready_;
    std::string topic_path_ready_;
    std::string topic_waypoint_done_;
    std::string topic_goal_reached_;
    std::string topic_lantern_detections_;

    rclcpp::Time last_mapping_ready_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_path_ready_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_waypoint_done_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_goal_reached_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_lantern_{0, 0, RCL_ROS_TIME};

    bool mapping_stale_reported_{false};
    bool path_stale_reported_{false};
    bool waypoint_stale_reported_{false};
    bool goal_stale_reported_{false};
    bool lantern_stale_reported_{false};
};
