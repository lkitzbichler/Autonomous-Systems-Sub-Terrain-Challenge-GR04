/**
 * @file exploration_node.cpp
 * @brief ROS2 Node for frontier-based exploration with OMPL path planning
 * 
 * This node:
 * - Subscribes to 3D occupancy grid maps from Unity simulation
 * - Calls frontier_exploration service to detect frontier regions
 * - Converts 2D frontiers to 3D targets for OMPL planning
 * - Uses multi-slice logic to handle exploration at different heights
 * - Publishes frontier targets for OMPL planner
 * 
 * @author Your Name
 * @date 2026
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "frontier_interfaces/srv/frontier_goal.hpp"
#include "multi_slice_logic.hpp"
#include "frontier_to_ompl.hpp"

#include <memory>
#include <vector>
#include <thread>
#include <mutex>

class ExplorationNode : public rclcpp::Node {
public:
    ExplorationNode()
        : Node("exploration_node"),
          current_drone_height_(0.0),
          exploration_height_(2.0),
          height_margin_(3.0),
          slice_thickness_(0.5),
          frontier_rank_(0),
          is_exploring_(false)
    {
        // Declare parameters
        this->declare_parameter("exploration_height", 2.0);
        this->declare_parameter("height_margin", 3.0);
        this->declare_parameter("slice_thickness", 0.5);
        this->declare_parameter("map_topic", "/map_3d");
        this->declare_parameter("drone_height_topic", "/drone_height");
        this->declare_parameter("frontier_rank", 0);
        this->declare_parameter("max_frontier_cost", 50.0);
        this->declare_parameter("min_exploration_height", 0.5);
        this->declare_parameter("max_exploration_height", 10.0);

        // Get parameters
        exploration_height_ = this->get_parameter("exploration_height").as_double();
        height_margin_ = this->get_parameter("height_margin").as_double();
        slice_thickness_ = this->get_parameter("slice_thickness").as_double();
        std::string map_topic = this->get_parameter("map_topic").as_string();
        std::string drone_height_topic = this->get_parameter("drone_height_topic").as_string();
        frontier_rank_ = this->get_parameter("frontier_rank").as_int();
        max_frontier_cost_ = this->get_parameter("max_frontier_cost").as_double();
        min_exploration_height_ = this->get_parameter("min_exploration_height").as_double();
        max_exploration_height_ = this->get_parameter("max_exploration_height").as_double();

        // Create subscriptions
        map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic, 10,
            std::bind(&ExplorationNode::mapCallback, this, std::placeholders::_1));

        height_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            drone_height_topic, 10,
            std::bind(&ExplorationNode::heightCallback, this, std::placeholders::_1));

        // Create frontier exploration client
        frontier_client_ = this->create_client<frontier_interfaces::srv::FrontierGoal>(
            "frontier_pose");

        // Create publishers
        frontier_target_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "frontier_target", 10);

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "frontier_targets_markers", 10);

        slice_maps_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "exploration_slices", 10);

        // Create timer for exploration loop
        exploration_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&ExplorationNode::explorationLoop, this));

        RCLCPP_INFO(this->get_logger(), "ExplorationNode initialized");
        RCLCPP_INFO(this->get_logger(), "  Exploration height: %.2f m", exploration_height_);
        RCLCPP_INFO(this->get_logger(), "  Height margin: %.2f m", height_margin_);
        RCLCPP_INFO(this->get_logger(), "  Slice thickness: %.2f m", slice_thickness_);
    }

private:
    // Callbacks
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(map_mutex_);
        current_map_ = *msg;
        has_map_ = true;
    }

    void heightCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(height_mutex_);
        current_drone_height_ = msg->data;
    }

    // Main exploration loop
    void explorationLoop() {
        std::lock_guard<std::mutex> lock(map_mutex_);

        if (!has_map_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Waiting for map data...");
            return;
        }

        if (is_exploring_) {
            RCLCPP_DEBUG(this->get_logger(), "Exploration already in progress");
            return;
        }

        is_exploring_ = true;

        try {
            // Request frontier from frontier_exploration service
            auto request = std::make_shared<frontier_interfaces::srv::FrontierGoal::Request>();
            request->goal_rank = frontier_rank_;

            // Wait for service
            if (!frontier_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_WARN(this->get_logger(),
                    "Frontier exploration service not available");
                is_exploring_ = false;
                return;
            }

            // Call service asynchronously
            auto future = frontier_client_->async_send_request(request,
                std::bind(&ExplorationNode::frontierResponseCallback, this, std::placeholders::_1));

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in exploration loop: %s", e.what());
            is_exploring_ = false;
        }
    }

    void frontierResponseCallback(
        rclcpp::Client<frontier_interfaces::srv::FrontierGoal>::SharedFuture future) {
        try {
            auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Received frontier goal at (%.2f, %.2f)",
                response->goal_pose.pose.position.x, response->goal_pose.pose.position.y);

            // Convert 2D frontier to 3D target using current exploration height
            double target_height = std::max(
                min_exploration_height_,
                std::min(max_exploration_height_, exploration_height_));

            FrontierToOMPL::FrontierTarget target =
                FrontierToOMPL::convertFrontierToTarget(
                    response->goal_pose,
                    target_height,
                    "map");

            // Check if target is within acceptable cost
            {
                std::lock_guard<std::mutex> lock(height_mutex_);
                geometry_msgs::msg::Point current_pos;
                current_pos.x = 0.0;  // Will be updated with actual robot position
                current_pos.y = 0.0;
                current_pos.z = current_drone_height_;

                target.cost = FrontierToOMPL::calculateDistance(current_pos, target.position);
            }

            if (target.cost <= max_frontier_cost_) {
                // Publish target for OMPL planner
                publishFrontierTarget(target);

                RCLCPP_INFO(this->get_logger(),
                    "Publishing frontier target: (%.2f, %.2f, %.2f), cost: %.2f",
                    target.position.x, target.position.y, target.position.z, target.cost);
            } else {
                RCLCPP_WARN(this->get_logger(),
                    "Frontier target exceeds max cost (%.2f > %.2f)",
                    target.cost, max_frontier_cost_);
            }

            // Process multi-slice logic if we have 3D data
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                if (current_map_.data.size() > 0) {
                    processMultiSliceLogic();
                }
            }

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                "Error processing frontier response: %s", e.what());
        }

        is_exploring_ = false;
    }

    void publishFrontierTarget(const FrontierToOMPL::FrontierTarget& target) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position = target.position;
        pose.pose.orientation.w = 1.0;  // Identity quaternion

        frontier_target_publisher_->publish(pose);

        // Also publish as marker for visualization
        publishTargetMarker(target);
    }

    void publishTargetMarker(const FrontierToOMPL::FrontierTarget& target) {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;

        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
        marker.id = target.frontier_id;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position = target.position;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.3;

        // Color by height (gradient from red to green)
        marker.color.r = 1.0f - (target.height / max_exploration_height_);
        marker.color.g = target.height / max_exploration_height_;
        marker.color.b = 0.5f;
        marker.color.a = 0.8f;

        marker_array.markers.push_back(marker);
        marker_publisher_->publish(marker_array);
    }

    void processMultiSliceLogic() {
        // Note: This assumes current_map_ contains 3D data
        // In practice, you may need to subscribe to a 3D occupancy grid
        // or reconstruct 3D representation from multiple 2D slices

        RCLCPP_DEBUG(this->get_logger(),
            "Processing multi-slice logic at drone height: %.2f",
            current_drone_height_);

        // For now, publish the current map as a slice
        // Full 3D implementation would generate multiple slices
        nav_msgs::msg::OccupancyGrid slice_map = current_map_;
        slice_map.header.stamp = this->now();
        slice_maps_publisher_->publish(slice_map);
    }

    // ROS2 interface members
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr height_subscription_;
    rclcpp::Client<frontier_interfaces::srv::FrontierGoal>::SharedPtr frontier_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr frontier_target_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr slice_maps_publisher_;
    rclcpp::TimerBase::SharedPtr exploration_timer_;

    // Synchronization
    std::mutex map_mutex_;
    std::mutex height_mutex_;

    // State
    nav_msgs::msg::OccupancyGrid current_map_;
    bool has_map_ = false;
    double current_drone_height_;
    bool is_exploring_;

    // Parameters
    double exploration_height_;
    double height_margin_;
    double slice_thickness_;
    int frontier_rank_;
    double max_frontier_cost_;
    double min_exploration_height_;
    double max_exploration_height_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExplorationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
