#include "lantern_detector_pkg/lantern_detector_node.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lantern_detector {

LanternDetectorNode::LanternDetectorNode(const rclcpp::NodeOptions& options)
    : Node("lantern_detector", options) {
  this->declare_parameter("depth_topic", "/realsense/depth/image");
  this->declare_parameter("depth_info_topic", "/realsense/depth/camera_info");
  this->declare_parameter("semantic_topic", "/Quadrotor/Sensors/SemanticCamera/image_raw");
  this->declare_parameter("semantic_info_topic", "/Quadrotor/Sensors/SemanticCamera/camera_info");
  this->declare_parameter("world_frame", "world");
  this->declare_parameter("distance_threshold", 2.0);

  const auto depth_topic = this->get_parameter("depth_topic").as_string();
  const auto depth_info_topic = this->get_parameter("depth_info_topic").as_string();
  const auto semantic_topic = this->get_parameter("semantic_topic").as_string();
  const auto semantic_info_topic = this->get_parameter("semantic_info_topic").as_string();
  world_frame_ = this->get_parameter("world_frame").as_string();
  distance_threshold_ = this->get_parameter("distance_threshold").as_double();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  lantern_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("detected_lanterns", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lantern_markers", 10);

  auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  sub_semantic_image_.subscribe(this, semantic_topic, qos);
  sub_depth_image_.subscribe(this, depth_topic, qos);
  sub_semantic_info_.subscribe(this, semantic_info_topic, qos);

  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), sub_semantic_image_, sub_depth_image_, sub_semantic_info_);
  sync_->registerCallback(&LanternDetectorNode::callback, this);
}

void LanternDetectorNode::callback(const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
                                   const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
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

  const double fx = info_msg->p[0] != 0 ? info_msg->p[0] : info_msg->k[0];
  const double fy = info_msg->p[5] != 0 ? info_msg->p[5] : info_msg->k[4];
  const double cx = info_msg->p[2] != 0 ? info_msg->p[2] : info_msg->k[2];
  const double cy = info_msg->p[6] != 0 ? info_msg->p[6] : info_msg->k[5];

  if (fx == 0 || fy == 0) return;

  double sum_x{0}, sum_y{0}, sum_z{0};
  int valid_count{0};

  for (const auto& p : locations) {
    const float d = depth.at<float>(p.y, p.x);
    if (std::isfinite(d) && d > 0.1f && d < 50.0f) {
      sum_z += d;
      sum_x += (p.x - cx) * d / fx;
      sum_y += (p.y - cy) * d / fy;
      valid_count++;
    }
  }

  if (valid_count == 0) return;

  geometry_msgs::msg::PointStamped pt_camera;
  pt_camera.header = semantic_msg->header;
  pt_camera.point.x = sum_x / valid_count;
  pt_camera.point.y = sum_y / valid_count;
  pt_camera.point.z = sum_z / valid_count;

  try {
    const auto pt_world = tf_buffer_->transform(pt_camera, world_frame_);
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
    m.scale.x = m.scale.y = m.scale.z = 0.5;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.a = 1.0;
    markers.markers.push_back(m);
  }

  lantern_pub_->publish(pose_array);
  marker_pub_->publish(markers);
}

}  // namespace lantern_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lantern_detector::LanternDetectorNode)
