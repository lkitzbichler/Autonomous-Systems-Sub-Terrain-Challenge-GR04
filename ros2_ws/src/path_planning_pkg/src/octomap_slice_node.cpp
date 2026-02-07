/**
 * @file octomap_slice_node.cpp
 * @brief Convert OctoMap into a 2D occupancy grid slice for frontier exploration.
 *
 * Subscribes:
 *  - /octomap_binary (octomap_msgs/Octomap)
 *  - /drone_height  (std_msgs/Float64) [optional]
 *
 * Publishes:
 *  - /map_3d (nav_msgs/OccupancyGrid) 2D slice derived from OctoMap
 */

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <memory>
#include <mutex>
#include <vector>

class OctomapSliceNode : public rclcpp::Node {
public:
  OctomapSliceNode() : Node("octomap_slice_node") {
    octomap_topic_ = declare_parameter<std::string>("octomap_topic", "/octomap_binary");
    map_topic_ = declare_parameter<std::string>("map_topic", "/map_3d");
    drone_height_topic_ = declare_parameter<std::string>("drone_height_topic", "/drone_height");
    frame_id_ = declare_parameter<std::string>("frame_id", "");

    use_drone_height_ = declare_parameter<bool>("use_drone_height", true);
    fixed_height_ = declare_parameter<double>("fixed_height", 2.0);
    height_margin_ = declare_parameter<double>("height_margin", 0.0);
    slice_thickness_ = declare_parameter<double>("slice_thickness", 0.5);
    publish_rate_ = declare_parameter<double>("publish_rate", 1.0);
    map_padding_ = declare_parameter<double>("map_padding", 1.0);

    occupied_value_ = declare_parameter<int>("occupied_value", 100);
    free_value_ = declare_parameter<int>("free_value", 0);

    if (slice_thickness_ <= 0.0) {
      slice_thickness_ = 0.5;
    }
    if (height_margin_ < 0.0) {
      height_margin_ = 0.0;
    }
    if (publish_rate_ <= 0.0) {
      publish_rate_ = 1.0;
    }

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, rclcpp::QoS(1).transient_local(),
      std::bind(&OctomapSliceNode::onOctomap, this, std::placeholders::_1));

    if (use_drone_height_) {
      height_sub_ = create_subscription<std_msgs::msg::Float64>(
        drone_height_topic_, 10,
        std::bind(&OctomapSliceNode::onHeight, this, std::placeholders::_1));
    }

    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, 1);

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_),
      std::bind(&OctomapSliceNode::publishSlice, this));

    RCLCPP_INFO(get_logger(), "octomap_slice_node ready (octomap: %s -> map: %s)",
                octomap_topic_.c_str(), map_topic_.c_str());
  }

private:
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    if (!msg) {
      return;
    }

    octomap::AbstractOcTree *tree = nullptr;
    if (msg->binary) {
      tree = octomap_msgs::binaryMsgToMap(*msg);
    } else {
      tree = octomap_msgs::fullMsgToMap(*msg);
    }

    if (!tree) {
      RCLCPP_WARN(get_logger(), "Failed to convert octomap message.");
      return;
    }

    auto *octree = dynamic_cast<octomap::OcTree *>(tree);
    if (!octree) {
      RCLCPP_WARN(get_logger(), "Octomap message was not an OcTree.");
      delete tree;
      return;
    }

    std::shared_ptr<octomap::OcTree> tree_ptr(octree);
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      octree_ = tree_ptr;
      last_frame_id_ = msg->header.frame_id;
    }
  }

  void onHeight(const std_msgs::msg::Float64::SharedPtr msg) {
    if (!msg) {
      return;
    }
    std::lock_guard<std::mutex> lock(height_mutex_);
    current_height_ = msg->data;
    have_height_ = true;
  }

  void publishSlice() {
    std::shared_ptr<octomap::OcTree> tree_snapshot;
    std::string frame_snapshot;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      tree_snapshot = octree_;
      frame_snapshot = last_frame_id_;
    }

    if (!tree_snapshot) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Waiting for octomap data...");
      return;
    }

    const double z_center = getSliceCenter();
    double z_min = 0.0;
    double z_max = 0.0;
    if (height_margin_ > 0.0) {
      z_min = z_center - height_margin_;
      z_max = z_center + height_margin_;
    } else {
      const double half_thickness = slice_thickness_ * 0.5;
      z_min = z_center - half_thickness;
      z_max = z_center + half_thickness;
    }

    double min_x = 0.0;
    double min_y = 0.0;
    double min_z = 0.0;
    double max_x = 0.0;
    double max_y = 0.0;
    double max_z = 0.0;
    tree_snapshot->getMetricMin(min_x, min_y, min_z);
    tree_snapshot->getMetricMax(max_x, max_y, max_z);

    min_x -= map_padding_;
    min_y -= map_padding_;
    max_x += map_padding_;
    max_y += map_padding_;

    const double resolution = tree_snapshot->getResolution();
    const int width = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / resolution)));
    const int height = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / resolution)));

    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = now();
    grid.header.frame_id = !frame_id_.empty() ? frame_id_ : frame_snapshot;
    if (grid.header.frame_id.empty()) {
      grid.header.frame_id = "world";
    }

    grid.info.resolution = static_cast<float>(resolution);
    grid.info.width = static_cast<uint32_t>(width);
    grid.info.height = static_cast<uint32_t>(height);
    grid.info.origin.position.x = min_x;
    grid.info.origin.position.y = min_y;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.assign(static_cast<size_t>(width * height), -1);

    octomap::point3d min_point(min_x, min_y, z_min);
    octomap::point3d max_point(max_x, max_y, z_max);

    for (auto it = tree_snapshot->begin_leafs_bbx(min_point, max_point);
         it != tree_snapshot->end_leafs_bbx(); ++it) {
      const double x = it.getX();
      const double y = it.getY();

      const int ix = static_cast<int>(std::floor((x - min_x) / resolution));
      const int iy = static_cast<int>(std::floor((y - min_y) / resolution));
      if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
        continue;
      }

      const size_t idx = static_cast<size_t>(iy * width + ix);
      if (tree_snapshot->isNodeOccupied(*it)) {
        grid.data[idx] = static_cast<int8_t>(occupied_value_);
      } else if (grid.data[idx] != static_cast<int8_t>(occupied_value_)) {
        grid.data[idx] = static_cast<int8_t>(free_value_);
      }
    }

    map_pub_->publish(grid);
  }

  double getSliceCenter() const {
    if (!use_drone_height_) {
      return fixed_height_;
    }

    std::lock_guard<std::mutex> lock(height_mutex_);
    if (!have_height_) {
      return fixed_height_;
    }
    return current_height_;
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr height_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::mutex map_mutex_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::string last_frame_id_;

  mutable std::mutex height_mutex_;
  double current_height_{0.0};
  bool have_height_{false};

  std::string octomap_topic_;
  std::string map_topic_;
  std::string drone_height_topic_;
  std::string frame_id_;

  bool use_drone_height_{true};
  double fixed_height_{2.0};
  double height_margin_{0.0};
  double slice_thickness_{0.5};
  double publish_rate_{1.0};
  double map_padding_{1.0};
  int occupied_value_{100};
  int free_value_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapSliceNode>());
  rclcpp::shutdown();
  return 0;
}
