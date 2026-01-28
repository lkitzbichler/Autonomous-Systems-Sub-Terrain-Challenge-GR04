#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

struct Lantern {
    geometry_msgs::msg::Point position;
    int count;
};

class LanternDetectorNode : public rclcpp::Node {
public:
    LanternDetectorNode() : Node("lantern_detector") {
        // Parameters
        this->declare_parameter("depth_topic", "/realsense/depth/image");
        this->declare_parameter("depth_info_topic", "/realsense/depth/camera_info");
        this->declare_parameter("semantic_topic", "/Quadrotor/Sensors/SemanticCamera/image_raw");
        this->declare_parameter("semantic_info_topic", "/Quadrotor/Sensors/SemanticCamera/camera_info");
        this->declare_parameter("world_frame", "world");
        this->declare_parameter("camera_frame", "Quadrotor/Sensors/SemanticCamera");
        this->declare_parameter("distance_threshold", 2.0); // Distance to merge lanterns

        depth_topic_ = this->get_parameter("depth_topic").as_string();
        depth_info_topic_ = this->get_parameter("depth_info_topic").as_string();
        semantic_topic_ = this->get_parameter("semantic_topic").as_string();
        semantic_info_topic_ = this->get_parameter("semantic_info_topic").as_string();
        world_frame_ = this->get_parameter("world_frame").as_string();
        camera_frame_ = this->get_parameter("camera_frame").as_string();
        distance_threshold_ = this->get_parameter("distance_threshold").as_double();

        RCLCPP_INFO(this->get_logger(), "Starting Lantern Detector Node...");

        // TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publishers
        lantern_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("detected_lanterns", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("lantern_markers", 10);

        // Synchronized Subscribers
        auto qos = rclcpp::SensorDataQoS();
        
        sub_semantic_image_.subscribe(this, semantic_topic_, qos.get_rmw_qos_profile());
        sub_depth_image_.subscribe(this, depth_topic_, qos.get_rmw_qos_profile());
        sub_semantic_info_.subscribe(this, semantic_info_topic_, qos.get_rmw_qos_profile());

        sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), sub_semantic_image_, sub_depth_image_, sub_semantic_info_);
        
        sync_->registerCallback(std::bind(&LanternDetectorNode::callback, this, 
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        RCLCPP_INFO(this->get_logger(), "Lantern Detector Node Started");
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
                  const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
        
        cv_bridge::CvImagePtr cv_semantic;
        cv::Mat depth_float;
        try {
            cv_semantic = cv_bridge::toCvCopy(semantic_msg, sensor_msgs::image_encodings::BGR8);
            
            // Handle depth conversion manually to ensure unit consistency (meters)
            if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
                cv_bridge::CvImagePtr cv_depth_16 = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
                cv_depth_16->image.convertTo(depth_float, CV_32F, 0.001); // mm to meters
            } else if (depth_msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                depth_float = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Unsupported depth encoding: %s", depth_msg->encoding.c_str());
                return;
            }
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // 1. Locate lantern pixels (any non-black pixel)
        cv::Mat gray;
        cv::cvtColor(cv_semantic->image, gray, cv::COLOR_BGR2GRAY);
        cv::Mat mask = gray > 0;

        std::vector<cv::Point> locations;
        cv::findNonZero(mask, locations);

        if (locations.empty()) {
            return;
        }

        // 2 & 3. Project all valid depth pixels to 3D in camera frame and calculate centroid
        // Use Projection matrix P for rectified images (fx = P[0], cx = P[2], fy = P[5], cy = P[6])
        double fx = info_msg->p[0];
        double cx = info_msg->p[2];
        double fy = info_msg->p[5];
        double cy = info_msg->p[6];

        // Safety fallback to K matrix if P is empty (common in some simulators)
        if (fx == 0.0) {
            fx = info_msg->k[0];
            cx = info_msg->k[2];
            fy = info_msg->k[4];
            cy = info_msg->k[5];
        }

        if (fx == 0.0 || fy == 0.0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Invalid CameraInfo (fx/fy are 0)");
            return;
        }

        double sum_x = 0, sum_y = 0, sum_z = 0;
        int valid_count = 0;

        for (const auto& p : locations) {
            float depth = depth_float.at<float>(p.y, p.x);
            if (!std::isnan(depth) && depth > 0.1 && depth < 50.0) { // Limit to valid range
                sum_z += depth;
                sum_x += (p.x - cx) * depth / fx;
                sum_y += (p.y - cy) * depth / fy;
                valid_count++;
            }
        }

        if (valid_count == 0) {
            return;
        }

        geometry_msgs::msg::PointStamped pt_camera;
        pt_camera.header = semantic_msg->header;
        pt_camera.point.x = sum_x / valid_count;
        pt_camera.point.y = sum_y / valid_count;
        pt_camera.point.z = sum_z / valid_count;

        // 5. Transform to world frame
        geometry_msgs::msg::PointStamped pt_world;
        try {
            // Look up the transform at the time of the image
            pt_world = tf_buffer_->transform(pt_camera, world_frame_);
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "Could not transform to world: %s", ex.what());
            return;
        }

        // 6. Update internal state
        update_lanterns(pt_world.point);

        // 7. Publish
        publish_lanterns();
    }

    void update_lanterns(const geometry_msgs::msg::Point& new_pos) {
        bool found = false;
        for (auto& lantern : detected_lanterns_) {
            double dist = std::sqrt(
                std::pow(lantern.position.x - new_pos.x, 2) +
                std::pow(lantern.position.y - new_pos.y, 2) +
                std::pow(lantern.position.z - new_pos.z, 2)
            );

            if (dist < distance_threshold_) {
                // Moving average update
                lantern.position.x = (lantern.position.x * lantern.count + new_pos.x) / (lantern.count + 1);
                lantern.position.y = (lantern.position.y * lantern.count + new_pos.y) / (lantern.count + 1);
                lantern.position.z = (lantern.position.z * lantern.count + new_pos.z) / (lantern.count + 1);
                lantern.count++;
                found = true;
                break;
            }
        }

        if (!found) {
            Lantern new_lantern;
            new_lantern.position = new_pos;
            new_lantern.count = 1;
            detected_lanterns_.push_back(new_lantern);
            RCLCPP_INFO(this->get_logger(), "New lantern detected at: [%.2f, %.2f, %.2f]", 
                new_pos.x, new_pos.y, new_pos.z);
        }
    }

    void publish_lanterns() {
        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->get_clock()->now();
        pose_array.header.frame_id = world_frame_;

        visualization_msgs::msg::MarkerArray markers;

        for (size_t i = 0; i < detected_lanterns_.size(); ++i) {
            geometry_msgs::msg::Pose pose;
            pose.position = detected_lanterns_[i].position;
            pose.orientation.w = 1.0;
            pose_array.poses.push_back(pose);

            visualization_msgs::msg::Marker marker;
            marker.header = pose_array.header;
            marker.ns = "lanterns";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            markers.markers.push_back(marker);
        }

        lantern_pub_->publish(pose_array);
        marker_pub_->publish(markers);
    }

    std::string depth_topic_, depth_info_topic_, semantic_topic_, semantic_info_topic_, world_frame_, camera_frame_;
    double distance_threshold_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    message_filters::Subscriber<sensor_msgs::msg::Image> sub_semantic_image_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_depth_image_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_semantic_info_;

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo> SyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lantern_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::vector<Lantern> detected_lanterns_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LanternDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
