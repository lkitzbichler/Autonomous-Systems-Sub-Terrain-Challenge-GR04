#include "lantern_detector_pkg/lantern_detector_node.hpp"
#include "protocol.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <algorithm>
#include <chrono>

namespace lantern_detector {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;

LanternDetectorNode::LanternDetectorNode(const rclcpp::NodeOptions& options)
    : Node("lantern_detector", options) {
  RCLCPP_INFO(this->get_logger(), "Lantern Detector Node is starting up...");
  
  const auto depth_topic = this->declare_parameter("depth_topic", "/realsense/depth/image");
  const auto semantic_topic = this->declare_parameter("semantic_topic", "/Quadrotor/Sensors/SemanticCamera/image_raw");
  const auto depth_info_topic = this->declare_parameter("depth_info_topic", "/realsense/depth/camera_info");
  const auto output_topic = this->declare_parameter("output_topic", "detected_lanterns");
  const auto marker_topic = this->declare_parameter("marker_topic", "lantern_markers");
  
  world_frame_ = this->declare_parameter("world_frame", "world");
  distance_threshold_ = this->declare_parameter("distance_threshold", 2.0);
  min_depth_ = this->declare_parameter("min_depth", 0.1);
  max_depth_ = this->declare_parameter("max_depth", 50.0);
  marker_scale_ = this->declare_parameter("marker_scale", 0.5);
  marker_color_ = this->declare_parameter("marker_color", std::vector<double>{1.0, 1.0, 0.0, 1.0});
  heartbeat_topic_ = this->declare_parameter("heartbeat_topic", std::string("heartbeat"));
  heartbeat_period_sec_ = this->declare_parameter("heartbeat_period_sec", 1.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  lantern_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(output_topic, 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
  heartbeat_pub_ = this->create_publisher<statemachine_pkg::msg::Answer>(heartbeat_topic_, 10);
  heartbeat_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(heartbeat_period_sec_),
      std::bind(&LanternDetectorNode::publish_heartbeat, this));

  auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
  
  sub_semantic_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, semantic_topic, qos);
  sub_depth_filter_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, depth_topic, qos);

  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), *sub_semantic_filter_, *sub_depth_filter_);
  sync_->registerCallback(std::bind(&LanternDetectorNode::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));

  sub_depth_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      depth_info_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { 
        depth_info_ = msg; 
      });
  
  RCLCPP_INFO(this->get_logger(), "Lantern Detector Node initialized with ApproximateTime synchronization.");
}

void LanternDetectorNode::publish_heartbeat() {
  statemachine_pkg::msg::Answer hb;
  hb.node_name = this->get_name();
  hb.state = static_cast<uint8_t>(statemachine_pkg::protocol::AnswerStates::RUNNING);
  hb.info = "RUNNING";
  const auto now = this->now();
  hb.timestamp.sec = static_cast<int32_t>(now.nanoseconds() / 1000000000LL);
  hb.timestamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000LL);
  heartbeat_pub_->publish(hb);
}

void LanternDetectorNode::synchronized_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) 
{
  if (!depth_info_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for depth camera info...");
    return;
  }

  cv::Mat semantic, depth, mask;
  try {
    semantic = cv_bridge::toCvShare(semantic_msg, "bgr8")->image;
    
    if (depth_msg->encoding == "16UC1") {
      cv_bridge::toCvShare(depth_msg, "16UC1")->image.convertTo(depth, CV_32F, 0.001);
    } else {
      depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image.clone();
    }

    cv::cvtColor(semantic, mask, cv::COLOR_BGR2GRAY);
    cv::threshold(mask, mask, 0, 255, cv::THRESH_BINARY); 
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  const double fx = depth_info_->p[0] != 0 ? depth_info_->p[0] : depth_info_->k[0];
  const double fy = depth_info_->p[5] != 0 ? depth_info_->p[5] : depth_info_->k[4];
  const double cx = depth_info_->p[2] != 0 ? depth_info_->p[2] : depth_info_->k[2];
  const double cy = depth_info_->p[6] != 0 ? depth_info_->p[6] : depth_info_->k[5];

  double sum_x = 0, sum_y = 0, sum_z = 0;
  int count = 0;

  for (int y = 0; y < mask.rows; ++y) {
    for (int x = 0; x < mask.cols; ++x) {
      if (mask.at<uchar>(y, x) > 0) {
        int dx = (semantic.cols != depth.cols) ? (x * depth.cols / semantic.cols) : x;
        int dy = (semantic.rows != depth.rows) ? (y * depth.rows / semantic.rows) : y;
        
        if (dx >= depth.cols || dy >= depth.rows) continue;
        
        const float d = depth.at<float>(dy, dx);
        if (std::isfinite(d) && d > min_depth_ && d < max_depth_) {
          sum_z += d;
          sum_x += (dx - cx) * d / fx;
          sum_y += (dy - cy) * d / fy;
          count++;
        }
      }
    }
  }

  if (count > 0) {
    geometry_msgs::msg::PointStamped pt_depth_frame;
    pt_depth_frame.header = depth_msg->header;
    pt_depth_frame.point.x = sum_x / count;
    pt_depth_frame.point.y = sum_y / count;
    pt_depth_frame.point.z = sum_z / count;

    try {
      const auto pt_world = tf_buffer_->transform(pt_depth_frame, world_frame_, tf2::durationFromSec(0.2));
      update_lanterns(pt_world.point);
      publish_lanterns();
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF: %s", ex.what());
    }
  }
}

void LanternDetectorNode::update_lanterns(const geometry_msgs::msg::Point& new_pos) {
  auto it = std::find_if(detected_lanterns_.begin(), detected_lanterns_.end(), [&](const auto& l) {
    double dist = std::hypot(l.position.x - new_pos.x, l.position.y - new_pos.y, l.position.z - new_pos.z);
    return dist < distance_threshold_;
  });

  if (it != detected_lanterns_.end()) {
    // Why: Smooth out detections over time
    it->position.x = (it->position.x * it->count + new_pos.x) / (it->count + 1);
    it->position.y = (it->position.y * it->count + new_pos.y) / (it->count + 1);
    it->position.z = (it->position.z * it->count + new_pos.z) / (it->count + 1);
    it->count++;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                         "Updated lantern at [%.2f, %.2f, %.2f] (Detections: %d)",
                         it->position.x, it->position.y, it->position.z, it->count);
  } else {
    detected_lanterns_.push_back({new_pos, 1});
    RCLCPP_INFO(this->get_logger(), "Found new lantern: [%.1f, %.1f, %.1f] (Total: %zu)", 
                new_pos.x, new_pos.y, new_pos.z, detected_lanterns_.size());
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
