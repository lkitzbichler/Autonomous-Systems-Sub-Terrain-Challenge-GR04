#include "lantern_detector_pkg/lantern_detector_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lantern_detector {

LanternDetectorNode::LanternDetectorNode(const rclcpp::NodeOptions& options)
    : Node("lantern_detector", options) {
  const auto depth_topic = this->declare_parameter("depth_topic", "/realsense/depth/image");
  const auto depth_info_topic = this->declare_parameter("depth_info_topic", "/realsense/depth/camera_info");
  const auto semantic_topic = this->declare_parameter("semantic_topic", "/Quadrotor/Sensors/SemanticCamera/image_raw");
  const auto semantic_info_topic = this->declare_parameter("semantic_info_topic", "/Quadrotor/Sensors/SemanticCamera/camera_info");
  const auto output_topic = this->declare_parameter("output_topic", "detected_lanterns");
  const auto marker_topic = this->declare_parameter("marker_topic", "lantern_markers");

  world_frame_ = this->declare_parameter("world_frame", "world");
  distance_threshold_ = this->declare_parameter("distance_threshold", 2.0);
  min_depth_ = this->declare_parameter("min_depth", 0.1);
  max_depth_ = this->declare_parameter("max_depth", 50.0);
  marker_scale_ = this->declare_parameter("marker_scale", 0.5);
  marker_color_ = this->declare_parameter("marker_color", std::vector<double>{1.0, 1.0, 0.0, 1.0});

  const auto sync_queue_size = this->declare_parameter("sync_queue_size", 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  lantern_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(output_topic, 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);

  auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  sub_semantic_image_.subscribe(this, semantic_topic, qos);
  sub_depth_image_.subscribe(this, depth_topic, qos);

  sub_semantic_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      semantic_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { semantic_info_ = msg; });
  
  sub_depth_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { depth_info_ = msg; });

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_size), sub_semantic_image_, sub_depth_image_);
  sync_->registerCallback(&LanternDetectorNode::callback, this);
}

void LanternDetectorNode::callback(const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
                                   const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
  if (!semantic_info_ || !depth_info_) return;

  cv::Mat semantic, depth;
  try {
    semantic = cv_bridge::toCvCopy(semantic_msg, "bgr8")->image;
    if (depth_msg->encoding == "16UC1") {
      cv::Mat depth_16 = cv_bridge::toCvCopy(depth_msg, "16UC1")->image;
      depth_16.convertTo(depth, CV_32F, 0.001);
    } else if (depth_msg->encoding == "32FC1") {
      depth = cv_bridge::toCvCopy(depth_msg, "32FC1")->image;
    } else {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Unsupported depth encoding: %s",
                            depth_msg->encoding.c_str());
      return;
    }
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Why: Quick check for non-black pixels representing lanterns in semantic camera
  cv::Mat mask;
  cv::cvtColor(semantic, mask, cv::COLOR_BGR2GRAY);
  cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY);

  std::vector<cv::Point> locations;
  cv::findNonZero(mask, locations);

  if (locations.empty()) return;

  // Use depth camera intrinsics to project depth pixels to 3D
  const double fx = depth_info_->p[0] != 0 ? depth_info_->p[0] : depth_info_->k[0];
  const double fy = depth_info_->p[5] != 0 ? depth_info_->p[5] : depth_info_->k[4];
  const double cx = depth_info_->p[2] != 0 ? depth_info_->p[2] : depth_info_->k[2];
  const double cy = depth_info_->p[6] != 0 ? depth_info_->p[6] : depth_info_->k[5];

  if (fx == 0 || fy == 0) return;

  double sum_x{0}, sum_y{0}, sum_z{0};
  int valid_count{0};

  for (const auto& p : locations) {
    // Assuming depth and semantic images are aligned. 
    // If not, a mapping from semantic to depth coordinates would be needed here.
    if (p.x >= static_cast<int>(depth.cols) || p.y >= static_cast<int>(depth.rows)) continue;
    
    const float d = depth.at<float>(p.y, p.x);
    if (std::isfinite(d) && d > min_depth_ && d < max_depth_) {
      sum_z += d;
      sum_x += (p.x - cx) * d / fx;
      sum_y += (p.y - cy) * d / fy;
      valid_count++;
    }
  }

  if (valid_count == 0) return;

  geometry_msgs::msg::PointStamped pt_depth_frame;
  pt_depth_frame.header = depth_msg->header;
  pt_depth_frame.point.x = sum_x / valid_count;
  pt_depth_frame.point.y = sum_y / valid_count;
  pt_depth_frame.point.z = sum_z / valid_count;

  try {
    const auto pt_world = tf_buffer_->transform(pt_depth_frame, world_frame_);
    update_lanterns(pt_world.point);
    publish_lanterns();
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "%s", ex.what());
  }
}

void LanternDetectorNode::update_lanterns(const geometry_msgs::msg::Point& new_pos) {
  auto it = std::find_if(detected_lanterns_.begin(), detected_lanterns_.end(), [&](const auto& l) {
    return std::hypot(l.position.x - new_pos.x, l.position.y - new_pos.y, l.position.z - new_pos.z) < distance_threshold_;
  });

  if (it != detected_lanterns_.end()) {
    // Why: Smooth out detections over time
    it->position.x = (it->position.x * it->count + new_pos.x) / (it->count + 1);
    it->position.y = (it->position.y * it->count + new_pos.y) / (it->count + 1);
    it->position.z = (it->position.z * it->count + new_pos.z) / (it->count + 1);
    it->count++;
  } else {
    detected_lanterns_.push_back({new_pos, 1});
    RCLCPP_INFO(this->get_logger(), "Found lantern: [%.1f, %.1f, %.1f]", new_pos.x, new_pos.y, new_pos.z);
  }
}

void LanternDetectorNode::publish_lanterns() {
  auto now = this->get_clock()->now();
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = now;
  pose_array.header.frame_id = world_frame_;

  visualization_msgs::msg::MarkerArray markers;

  for (size_t i = 0; i < detected_lanterns_.size(); ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position = detected_lanterns_[i].position;
    pose.orientation.w = 1.0;
    pose_array.poses.push_back(pose);

    visualization_msgs::msg::Marker m;
    m.header = pose_array.header;
    m.ns = "lanterns";
    m.id = i;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose = pose;
    m.scale.x = m.scale.y = m.scale.z = marker_scale_;
    if (marker_color_.size() >= 4) {
      m.color.r = marker_color_[0];
      m.color.g = marker_color_[1];
      m.color.b = marker_color_[2];
      m.color.a = marker_color_[3];
    }
    markers.markers.push_back(m);
  }

  lantern_pub_->publish(pose_array);
  marker_pub_->publish(markers);
}

}  // namespace lantern_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lantern_detector::LanternDetectorNode)
