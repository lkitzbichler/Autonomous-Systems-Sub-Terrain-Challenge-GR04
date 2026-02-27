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

/**
 * @brief Construct a new LanternDetectorNode object and initialize all
 *        publishers, subscribers, parameters, and synchronization.
 *
 * Parameters are read from the node's parameter server with sensible
 * defaults.  The semantic and depth image topics are synchronized using an
 * approximate-time policy. A heartbeat timer is also configured.
 *
 * @param options ROS2 node options supplied by component infrastructure.
 */
LanternDetectorNode::LanternDetectorNode(const rclcpp::NodeOptions& options)
    : Node("lantern_detector", options) {
  RCLCPP_INFO(this->get_logger(), "Lantern Detector Node is starting up...");
  
  const auto depth_topic = this->declare_parameter("depth_topic", "/realsense/depth/image");
  const auto semantic_topic = this->declare_parameter("semantic_topic", "/Quadrotor/Sensors/SemanticCamera/image_raw");
  const auto depth_info_topic = this->declare_parameter("depth_info_topic", "/realsense/depth/camera_info");
  const auto output_topic = this->declare_parameter("output_topic", "detected_lanterns");
  const auto count_topic = this->declare_parameter("count_topic", "detected_lanterns/counts");
  const auto marker_topic = this->declare_parameter("marker_topic", "lantern_markers");
  
  world_frame_ = this->declare_parameter("world_frame", "world");
  distance_threshold_m_ = this->declare_parameter("distance_threshold", 2.0);
  min_depth_m_ = this->declare_parameter("min_depth", 0.1);
  max_depth_m_ = this->declare_parameter("max_depth", 50.0);
  min_sightings_ = this->declare_parameter("min_num_sightings", 50);
  marker_scale_m_ = this->declare_parameter("marker_scale", 0.5);
  marker_color_ = this->declare_parameter("marker_color", std::vector<double>{1.0, 1.0, 0.0, 1.0});
  heartbeat_topic_ = this->declare_parameter("heartbeat_topic", std::string("heartbeat"));
  heartbeat_period_s_ = this->declare_parameter("heartbeat_period_sec", 1.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  lantern_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(output_topic, 10);
  lantern_counts_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(count_topic, 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
  heartbeat_pub_ = this->create_publisher<statemachine_pkg::msg::Answer>(heartbeat_topic_, 10);
  heartbeat_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(heartbeat_period_s_),
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
/**
 * @brief Send a heartbeat message indicating the node is alive.
 *
 * The message includes the node name, current state, and a timestamp.
 */
void LanternDetectorNode::publish_heartbeat() {
  statemachine_pkg::msg::Answer heartbeat_msg;
  heartbeat_msg.node_name = this->get_name();
  heartbeat_msg.state = static_cast<uint8_t>(statemachine_pkg::protocol::AnswerStates::RUNNING);
  heartbeat_msg.info = "RUNNING";
  const auto current_time = this->now();
  heartbeat_msg.timestamp.sec = static_cast<int32_t>(current_time.nanoseconds() / 1000000000LL);
  heartbeat_msg.timestamp.nanosec = static_cast<uint32_t>(current_time.nanoseconds() % 1000000000LL);
  heartbeat_pub_->publish(heartbeat_msg);
}
/**
 * @brief Process a pair of synchronized semantic and depth images.
 *
 * Extracts a binary mask from the semantic image, projects valid depth
 * pixels into the camera frame, computes their average position and
 * transforms that point into the world frame for downstream tracking.
 *
 * @param semantic_msg Semantic image where lanterns are highlighted.
 * @param depth_msg Corresponding depth image (float meters).
 */
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

  const double focal_length_x = depth_info_->p[0] != 0 ? depth_info_->p[0] : depth_info_->k[0];
  const double focal_length_y = depth_info_->p[5] != 0 ? depth_info_->p[5] : depth_info_->k[4];
  const double principal_point_x = depth_info_->p[2] != 0 ? depth_info_->p[2] : depth_info_->k[2];
  const double principal_point_y = depth_info_->p[6] != 0 ? depth_info_->p[6] : depth_info_->k[5];

  double sum_x_m = 0, sum_y_m = 0, sum_z_m = 0;
  int valid_pixel_count = 0;

  for (int y = 0; y < mask.rows; ++y) {
    for (int x = 0; x < mask.cols; ++x) {
      if (mask.at<uchar>(y, x) > 0) {
        int depth_x = (semantic.cols != depth.cols) ? (x * depth.cols / semantic.cols) : x;
        int depth_y = (semantic.rows != depth.rows) ? (y * depth.rows / semantic.rows) : y;
        
        if (depth_x >= depth.cols || depth_y >= depth.rows) continue;
        
        const float depth_value_m = depth.at<float>(depth_y, depth_x);
        if (std::isfinite(depth_value_m) && depth_value_m > min_depth_m_ && depth_value_m < max_depth_m_) {
          sum_z_m += depth_value_m;
          sum_x_m += (depth_x - principal_point_x) * depth_value_m / focal_length_x;
          sum_y_m += (depth_y - principal_point_y) * depth_value_m / focal_length_y;
          valid_pixel_count++;
        }
      }
    }
  }

  if (valid_pixel_count > 0) {
    geometry_msgs::msg::PointStamped point_depth_frame;
    point_depth_frame.header = depth_msg->header;
    point_depth_frame.point.x = sum_x_m / valid_pixel_count;
    point_depth_frame.point.y = sum_y_m / valid_pixel_count;
    point_depth_frame.point.z = sum_z_m / valid_pixel_count;

    try {
      const auto point_world = tf_buffer_->transform(point_depth_frame, world_frame_, tf2::durationFromSec(0.2));
      update_lanterns(point_world.point);
      publish_lanterns();
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "TF: %s", ex.what());
    }
  }
}
/**
 * @brief Integrate a new lantern position into the tracked set.
 *
 * If an existing lantern is within @c distance_threshold_m_ of the new
 * position, the function updates the stored position using a running
 * average.  Otherwise a new entry is appended.
 *
 * @param new_position Newly observed lantern location in world frame.
 */
void LanternDetectorNode::update_lanterns(const geometry_msgs::msg::Point& new_position) {
  auto iter = std::find_if(detected_lanterns_.begin(), detected_lanterns_.end(), [&](const auto& l) {
    double distance_m = std::hypot(l.position.x - new_position.x,
                                   l.position.y - new_position.y,
                                   l.position.z - new_position.z);
    return distance_m < distance_threshold_m_;
  });

  if (iter != detected_lanterns_.end()) {
    // Smooth out detections over time by averaging with previous value
    iter->position.x = (iter->position.x * iter->count + new_position.x) / (iter->count + 1);
    iter->position.y = (iter->position.y * iter->count + new_position.y) / (iter->count + 1);
    iter->position.z = (iter->position.z * iter->count + new_position.z) / (iter->count + 1);
    iter->count++;
  } else {
    detected_lanterns_.push_back({new_position, 1});
  }
}

/**
 * @brief Publish the current set of lantern detections and associated data.
 *
 * Only lanterns with at least @c min_sightings_ observations are emitted.  A
 * PoseArray, an Int32MultiArray of counts, and a MarkerArray used for
 * visualization are published.
 */
void LanternDetectorNode::publish_lanterns() {
  auto current_time = this->get_clock()->now();
  geometry_msgs::msg::PoseArray pose_array_msg;
  pose_array_msg.header.stamp = current_time;
  pose_array_msg.header.frame_id = world_frame_;
  std_msgs::msg::Int32MultiArray count_array_msg;

  visualization_msgs::msg::MarkerArray marker_array_msg;

  for (size_t idx = 0; idx < detected_lanterns_.size(); ++idx) {
    if (detected_lanterns_[idx].count < min_sightings_) {
      continue; // Skip weak detections
    }
    geometry_msgs::msg::Pose pose;
    pose.position = detected_lanterns_[idx].position;
    pose.orientation.w = 1.0;
    pose_array_msg.poses.push_back(pose);
    count_array_msg.data.push_back(detected_lanterns_[idx].count);

    visualization_msgs::msg::Marker m;
    m.header = pose_array_msg.header;
    m.ns = "lanterns";
    m.id = static_cast<int>(idx);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.pose = pose;
    m.scale.x = m.scale.y = m.scale.z = marker_scale_m_;
    if (marker_color_.size() >= 4) {
      m.color.r = marker_color_[0];
      m.color.g = marker_color_[1];
      m.color.b = marker_color_[2];
      m.color.a = marker_color_[3];
    }
    marker_array_msg.markers.push_back(m);
  }

  lantern_pub_->publish(pose_array_msg);
  lantern_counts_pub_->publish(count_array_msg);
  marker_pub_->publish(marker_array_msg);
}
}  // namespace lantern_detector

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lantern_detector::LanternDetectorNode)
