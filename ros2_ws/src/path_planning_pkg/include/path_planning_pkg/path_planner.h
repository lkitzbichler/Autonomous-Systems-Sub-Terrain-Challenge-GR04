#pragma once

#include "statemachine_pkg/msg/answer.hpp"
#include "statemachine_pkg/msg/command.hpp"
#include "protocol.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <optional>
#include <cstddef>
#include <string>
#include <vector>

namespace octomap
{
class OcTree;
}

class PathPlanner : public rclcpp::Node
{
public: // PROTOCOL TYPES
    using Commands = statemachine_pkg::protocol::Commands;
    using AnswerStates = statemachine_pkg::protocol::AnswerStates;

private: // INTERNAL TYPES
    enum class PlannerMode : uint8_t {
        IDLE = 0,
        TRANSIT_RECORD = 1,
        EXPLORE = 2,
        BACKTRACK = 3,
        RETURN_HOME = 4,
        HOLD = 5,
        ABORTED = 6
    };

    struct TransitNode {
        int id{0};
        geometry_msgs::msg::Point position{};
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    };

    enum class GraphNodeStatus : uint8_t {
        UNVISITED = 0,
        VISITED = 1,
        FRONTIER = 2,
        DEAD_END = 3
    };

    struct GraphNode {
        int id{0};
        geometry_msgs::msg::Point position{};
        double yaw{0.0};
        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
        GraphNodeStatus status{GraphNodeStatus::UNVISITED};
        double frontier_score{0.0};
        int loop_id{0};
        bool valid{true};
        bool is_transit{false};
        std::size_t observations{1};
    };

    struct GraphEdge {
        int from_id{0};
        int to_id{0};
        double length_m{0.0};
        double cost{0.0};
        bool valid{true};
    };

    enum class CandidateStatus : uint8_t {
        FREE = 0,
        BLOCKED = 1,
        UNKNOWN = 2
    };

    struct CandidatePoint {
        geometry_msgs::msg::Point point{};
        double rel_yaw_deg{0.0};
        double rel_pitch_deg{0.0};
        CandidateStatus status{CandidateStatus::UNKNOWN};
    };

private: // STATE
    PlannerMode mode_{PlannerMode::IDLE};
    std::string statemachine_state_{"UNKNOWN"};

    bool has_odom_{false};
    bool has_map_{false};
    bool transit_record_enabled_{false};
    bool heartbeat_on_mode_change_{true};
    bool odom_frame_ok_{false};
    bool map_frame_ok_{false};

    rclcpp::Time last_heartbeat_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_command_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_map_time_{0, 0, RCL_ROS_TIME};

    Commands last_command_{Commands::NONE};
    uint64_t processed_command_count_{0};
    uint64_t ignored_command_count_{0};

    nav_msgs::msg::Odometry latest_odom_{};
    std::shared_ptr<octomap::OcTree> octree_;
    std::vector<TransitNode> transit_nodes_;
    int next_transit_node_id_{1};
    std::vector<GraphNode> graph_nodes_;
    std::vector<GraphEdge> graph_edges_;
    int next_graph_node_id_{1};
    int last_transit_graph_node_id_{-1};
    std::vector<CandidatePoint> latest_candidates_;
    std::optional<std::size_t> selected_candidate_index_;
    bool branch_detected_{false};
    bool last_branch_detected_{false};
    std::size_t free_candidate_count_{0};
    std::size_t unknown_candidate_count_{0};
    bool has_frontier_seed_position_{false};
    geometry_msgs::msg::Point last_frontier_seed_position_{};
    rclcpp::Time last_branch_event_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_loop_event_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_stuck_event_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_plan_publish_time_{0, 0, RCL_ROS_TIME};
    std::string pending_event_info_;
    bool has_last_plan_goal_{false};
    geometry_msgs::msg::Point last_plan_goal_{};
    std::size_t published_plan_count_{0};

    std::string odom_frame_id_;
    std::string map_frame_id_;

private: // PUBS / SUBS / TIMER
    rclcpp::Publisher<statemachine_pkg::msg::Answer>::SharedPtr pub_heartbeat_;
    rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr pub_trajectory_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_graph_markers_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_candidate_markers_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_current_plan_;

    rclcpp::Subscription<statemachine_pkg::msg::Command>::SharedPtr sub_cmd_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_sm_state_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap_;

    rclcpp::TimerBase::SharedPtr timer_;

private: // PARAMETERS
    // Topics
    std::string topic_cmd_;
    std::string topic_state_;
    std::string topic_heartbeat_;
    std::string topic_odom_;
    std::string topic_octomap_;
    std::string topic_trajectory_;
    std::string topic_graph_markers_;
    std::string topic_candidate_markers_;
    std::string topic_current_plan_;

    // Timing
    double loop_period_sec_{0.2};
    double heartbeat_period_sec_{1.0};
    double input_timeout_sec_{1.0};
    double map_timeout_sec_{0.0};
    double command_stale_sec_{5.0};
    double replan_period_sec_{0.5};

    // Command handling
    bool accept_empty_target_{false};
    bool accept_switch_to_explore_cmd_{true};

    // Frames
    std::string planning_frame_{"world"};

    // Safety
    double inflation_m_{1.0};
    double min_clearance_m_{1.5};
    double clearance_step_m_{0.5};
    double clearance_search_max_m_{8.0};

    // Graph recording
    double transit_node_spacing_m_{10.0};
    double graph_merge_radius_m_{3.0};
    double graph_ahead_query_dist_m_{12.0};

    // Explore candidate generation
    double candidate_distance_m_{8.0};
    double candidate_half_fov_deg_{60.0};
    int candidate_bin_count_{7};
    double candidate_vertical_half_fov_deg_{30.0};
    int candidate_vertical_bin_count_{3};
    int branch_min_free_candidates_{2};
    bool branch_count_unknown_{true};
    bool prefer_unknown_over_free_{true};
    double frontier_node_spacing_m_{6.0};
    int max_frontier_nodes_per_cycle_{3};
    bool create_side_frontier_nodes_{true};
    double event_cooldown_sec_{2.0};

    // Trajectory output
    bool trajectory_publish_enabled_{true};
    double trajectory_nominal_speed_mps_{2.5};
    double trajectory_min_segment_time_sec_{0.8};
    double trajectory_goal_replan_dist_m_{1.0};

public: // LIFECYCLE
    PathPlanner();
    ~PathPlanner() override;

private: // CALLBACKS
    void onCommand(const statemachine_pkg::msg::Command::SharedPtr msg);
    void onState(const std_msgs::msg::String::SharedPtr msg);
    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void onTimer();

private: // HELPERS
    void changeMode(PlannerMode new_mode, const std::string &reason);
    void updateTransitGateFromState(const std::string &sm_state);
    void tryRecordTransitNode();
    void publishHeartbeat();
    void publishGraphMarkers();
    void updateBranchCandidates();
    void updateExploreGraphFromCandidates();
    void updateLocalPlanAndTrajectory();
    void publishCandidateMarkers();
    bool isTargetMatch(const std::string &target) const;
    bool isStaleCommand(const statemachine_pkg::msg::Command &msg) const;
    std::string heartbeatInfo() const;
    bool isPlanningFrame(const std::string &frame_id) const;
    bool isFresh(const rclcpp::Time &stamp) const;
    bool hasValidOdom() const;
    bool hasValidMap() const;
    bool hasValidPlanningInputs() const;
    bool canPublishPlannerOutput() const;

    bool isFree(const geometry_msgs::msg::Point &point) const;
    bool isOccupied(const geometry_msgs::msg::Point &point) const;
    bool isUnknown(const geometry_msgs::msg::Point &point) const;
    bool raycast(const geometry_msgs::msg::Point &origin, const geometry_msgs::msg::Point &target,
                 geometry_msgs::msg::Point &hit_out, bool stop_on_unknown = true) const;
    double clearance(const geometry_msgs::msg::Point &point) const;
    std::optional<int> findNearestNodeId(const geometry_msgs::msg::Point &point, double radius_m) const;
    std::vector<int> findNearbyNodeIds(const geometry_msgs::msg::Point &point, double radius_m) const;
    int addOrMergeGraphNode(const geometry_msgs::msg::Point &point, bool is_transit, bool &inserted_new);
    void upsertGraphEdge(int from_id, int to_id);
    bool hasGraphEdge(int from_id, int to_id) const;
    void queueEventInfo(const std::string &event_info, rclcpp::Time &last_event_time);
    std::optional<geometry_msgs::msg::Point> selectGoalPoint() const;
    bool shouldReplanForGoal(const geometry_msgs::msg::Point &goal) const;
    bool computeLocalWaypointPath(const geometry_msgs::msg::Point &goal,
                                  std::vector<geometry_msgs::msg::Point> &path) const;
    void publishCurrentPlan(const std::vector<geometry_msgs::msg::Point> &path);
    bool publishTrajectoryFromPath(const std::vector<geometry_msgs::msg::Point> &path);
    GraphNode *findNodeById(int id);
    const GraphNode *findNodeById(int id) const;
    static double yawFromQuaternion(double x, double y, double z, double w);

private: // STRING HELPERS
    static std::string toString(PlannerMode mode);
    static std::string toHeartbeatInfo(PlannerMode mode);
    static std::string toString(GraphNodeStatus status);
    static std::string toString(CandidateStatus status);
};
