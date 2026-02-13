#include "path_planning_pkg/path_planner_node.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

void PathPlannerNode::publishVisualization(
    const std::vector<Eigen::Vector3d> &path,
    const std::vector<FrontierCluster> &clusters,
    const octomap::OcTreeKey *goal_key) {
  // --- Visualization ---
  // RViz markers for path, frontier centroids, and current goal.
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = world_frame_;
  path_marker.header.stamp = get_clock()->now();
  path_marker.ns = "path";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.scale.x = 0.05;
  path_marker.color.r = 0.1f;
  path_marker.color.g = 0.9f;
  path_marker.color.b = 0.1f;
  path_marker.color.a = 1.0f;
  for (const auto &p : path) {
    geometry_msgs::msg::Point pt;
    pt.x = p.x();
    pt.y = p.y();
    pt.z = p.z();
    path_marker.points.push_back(pt);
  }
  markers.markers.push_back(path_marker);

  visualization_msgs::msg::Marker frontier_marker;
  frontier_marker.header.frame_id = world_frame_;
  frontier_marker.header.stamp = get_clock()->now();
  frontier_marker.ns = "frontiers";
  frontier_marker.id = 1;
  frontier_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  frontier_marker.action = visualization_msgs::msg::Marker::ADD;
  frontier_marker.scale.x = 0.4;
  frontier_marker.scale.y = 0.4;
  frontier_marker.scale.z = 0.4;
  frontier_marker.color.r = 0.2f;
  frontier_marker.color.g = 0.4f;
  frontier_marker.color.b = 1.0f;
  frontier_marker.color.a = 0.8f;
  for (const auto &cluster : clusters) {
    geometry_msgs::msg::Point pt;
    pt.x = cluster.centroid.x();
    pt.y = cluster.centroid.y();
    pt.z = cluster.centroid.z();
    frontier_marker.points.push_back(pt);
  }
  markers.markers.push_back(frontier_marker);

  if (goal_key) {
    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header.frame_id = world_frame_;
    goal_marker.header.stamp = get_clock()->now();
    goal_marker.ns = "goal";
    goal_marker.id = 2;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.scale.x = 0.6;
    goal_marker.scale.y = 0.6;
    goal_marker.scale.z = 0.6;
    goal_marker.color.r = 1.0f;
    goal_marker.color.g = 0.2f;
    goal_marker.color.b = 0.2f;
    goal_marker.color.a = 0.9f;

    std::shared_ptr<octomap::OcTree> map;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      map = map_;
    }
    if (map) {
      const octomap::point3d coord = map->keyToCoord(*goal_key);
      goal_marker.pose.position.x = coord.x();
      goal_marker.pose.position.y = coord.y();
      goal_marker.pose.position.z = coord.z();
      markers.markers.push_back(goal_marker);
    }
  }

  markers_pub_->publish(markers);
}

void PathPlannerNode::publishPlannedPath(
    const std::vector<Eigen::Vector3d> &path) {
  if (!planned_path_pub_) {
    return;
  }
  const rclcpp::Time now = get_clock()->now();
  planned_path_history_.header.stamp = now;
  for (const auto &p : path) {
    if (have_planned_path_ && planned_path_min_distance_ > 0.0) {
      const double dist = (p - last_planned_path_position_).norm();
      if (dist < planned_path_min_distance_) {
        continue;
      }
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = world_frame_;
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.position.z = p.z();
    pose.pose.orientation.w = 1.0;
    planned_path_history_.poses.push_back(pose);
    last_planned_path_position_ = p;
    have_planned_path_ = true;
  }
  planned_path_pub_->publish(planned_path_history_);
}
