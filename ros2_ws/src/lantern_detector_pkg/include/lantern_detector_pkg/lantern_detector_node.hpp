#ifndef LANTERN_DETECTOR_PKG__LANTERN_DETECTOR_NODE_HPP_
#define LANTERN_DETECTOR_PKG__LANTERN_DETECTOR_NODE_HPP_

#include <mutex>
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace lantern_detector {

struct Lantern {
  geometry_msgs::msg::Point position;
  int count{0};
};

class LanternDetectorNode : public rclcpp::Node {
public:
  explicit LanternDetectorNode(const rclcpp::NodeOptions& options);

private:
  void synchronized_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

  void update_lanterns(const geometry_msgs::msg::Point& new_pos);
  void publish_lanterns();

  std::string world_frame_;
  double distance_threshold_;
  double min_depth_;
  double max_depth_;
  double marker_scale_;
  std::vector<double> marker_color_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Synchronized subscribers
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_semantic_filter_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_depth_filter_;
  
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_depth_info_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr depth_info_;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lantern_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::vector<Lantern> detected_lanterns_;
};

}  // namespace lantern_detector

#endif  // LANTERN_DETECTOR_PKG__LANTERN_DETECTOR_NODE_HPP_
