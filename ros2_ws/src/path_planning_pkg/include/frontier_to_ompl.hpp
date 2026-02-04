#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>

/**
 * @brief Interface between frontier_exploration and OMPL path planner
 * 
 * Converts 2D frontier goals from frontier_exploration to 3D targets
 * suitable for OMPL path planning in 3D space.
 */
class FrontierToOMPL {
public:
    struct FrontierTarget {
        geometry_msgs::msg::Point position;  // 3D position in world frame
        double cost;                          // Cost/distance metric
        int frontier_id;                      // Identifier for the frontier
        double height;                        // Z-coordinate (height)
    };

    /**
     * @brief Convert 2D frontier pose to 3D OMPL goal target
     * 
     * @param frontier_pose_2d 2D pose from frontier_exploration service
     * @param desired_height Target height (Z) for the drone
     * @param frame_id Target frame ID
     * @return FrontierTarget suitable for OMPL planning
     */
    static FrontierTarget convertFrontierToTarget(
        const geometry_msgs::msg::PoseStamped& frontier_pose_2d,
        double desired_height,
        const std::string& frame_id = "map");

    /**
     * @brief Rank multiple frontier targets by cost
     * 
     * @param targets Vector of frontier targets
     * @param current_position Current robot/drone position
     * @return Sorted vector of targets by distance cost
     */
    static std::vector<FrontierTarget> rankTargets(
        const std::vector<FrontierTarget>& targets,
        const geometry_msgs::msg::Point& current_position);

    /**
     * @brief Filter frontier targets by height constraints
     * 
     * @param targets Vector of frontier targets
     * @param min_height Minimum allowed height
     * @param max_height Maximum allowed height
     * @return Filtered targets within height constraints
     */
    static std::vector<FrontierTarget> filterByHeight(
        const std::vector<FrontierTarget>& targets,
        double min_height,
        double max_height);

    /**
     * @brief Calculate Euclidean distance between two 3D points
     */
    static double calculateDistance(
        const geometry_msgs::msg::Point& p1,
        const geometry_msgs::msg::Point& p2);
};
