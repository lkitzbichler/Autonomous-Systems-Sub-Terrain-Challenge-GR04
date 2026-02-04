#include "frontier_to_ompl.hpp"
#include <cmath>

FrontierToOMPL::FrontierTarget FrontierToOMPL::convertFrontierToTarget(
    const geometry_msgs::msg::PoseStamped& frontier_pose_2d,
    double desired_height,
    const std::string& frame_id)
{
    FrontierTarget target;

    // Copy XY from frontier pose and set Z to desired height
    target.position.x = frontier_pose_2d.pose.position.x;
    target.position.y = frontier_pose_2d.pose.position.y;
    target.position.z = desired_height;
    target.height = desired_height;

    // Initialize cost (will be set by ranking)
    target.cost = 0.0;

    // Initialize ID (can be set by caller)
    target.frontier_id = 0;

    return target;
}

std::vector<FrontierToOMPL::FrontierTarget> FrontierToOMPL::rankTargets(
    const std::vector<FrontierTarget>& targets,
    const geometry_msgs::msg::Point& current_position)
{
    std::vector<FrontierTarget> ranked_targets = targets;

    // Calculate cost (distance) for each target
    for (auto& target : ranked_targets) {
        target.cost = calculateDistance(current_position, target.position);
    }

    // Sort by cost (ascending - closest first)
    std::sort(ranked_targets.begin(), ranked_targets.end(),
        [](const FrontierTarget& a, const FrontierTarget& b) {
            return a.cost < b.cost;
        });

    return ranked_targets;
}

std::vector<FrontierToOMPL::FrontierTarget> FrontierToOMPL::filterByHeight(
    const std::vector<FrontierTarget>& targets,
    double min_height,
    double max_height)
{
    std::vector<FrontierTarget> filtered;

    for (const auto& target : targets) {
        if (target.height >= min_height && target.height <= max_height) {
            filtered.push_back(target);
        }
    }

    return filtered;
}

double FrontierToOMPL::calculateDistance(
    const geometry_msgs::msg::Point& p1,
    const geometry_msgs::msg::Point& p2)
{
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}
