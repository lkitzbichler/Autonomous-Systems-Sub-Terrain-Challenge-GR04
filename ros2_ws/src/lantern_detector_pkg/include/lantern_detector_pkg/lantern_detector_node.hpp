#ifndef LANTERN_DETECTOR_PKG__LANTERN_DETECTOR_NODE_HPP_
#define LANTERN_DETECTOR_PKG__LANTERN_DETECTOR_NODE_HPP_

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <mutex>
#include <opencv2/opencv.hpp>

#include "geometry_msgs/msg/pose_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "statemachine_pkg/msg/answer.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace lantern_detector {

/**
 * @brief Represents a single detected lantern with running average position.
 */
struct Lantern {
    geometry_msgs::msg::Point position; /**< Current averaged position in world frame (meters). */
    int count{0};                       /**< Number of times the lantern has been observed. */
};

/**
 * @brief ROS2 node that fuses semantic camera data with depth images to
 *        detect and track lantern positions in the world frame.
 *
 * The node subscribes to a semantic image and a depth image, performs
 * approximate time synchronization, projects valid depth pixels to 3D
 * coordinates, and smooths observations over time.  Detections are published
 * as PoseArray, counts, and visualization markers. A periodic heartbeat
 * message is also emitted.
 */
class LanternDetectorNode : public rclcpp::Node {
   public:
    /**
     * @brief Construct a new LanternDetectorNode object.
     *
     * @param options NodeOptions provided by the component infrastructure.
     */
    explicit LanternDetectorNode(const rclcpp::NodeOptions& options);

   private:
    /**
     * @brief Callback invoked with synchronized semantic and depth images.
     *
     * @param semantic_msg Semantic camera image containing lantern labels.
     * @param depth_msg Depth image corresponding to the semantic view.
     */
    void synchronized_callback(const sensor_msgs::msg::Image::ConstSharedPtr& semantic_msg,
                               const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg);

    /**
     * @brief Update the list of detected lanterns with a new observation.
     *
     * The function averages the incoming position with an existing nearby
     * detection or adds a new entry if none are sufficiently close.
     *
     * @param new_position Position of the newly detected lantern in world
     *        frame (meters).
     */
    void update_lanterns(const geometry_msgs::msg::Point& new_position);

    /**
     * @brief Publish current lantern detections, counts and visualization
     *        markers to their respective topics.
     *
     * Only detections with at least @c min_sightings_ observations are
     * published.
     */
    void publish_lanterns();

    /**
     * @brief Publish a heartbeat message indicating the node is running.
     */
    void publish_heartbeat();

    // configuration parameters
    std::string world_frame_;          /**< Target world frame for output poses. */
    double distance_threshold_m_;      /**< Merge threshold for lanterns (meters). */
    double min_depth_m_;               /**< Minimum valid depth value (meters). */
    double max_depth_m_;               /**< Maximum valid depth value (meters). */
    int min_sightings_;                /**< Minimum observations to publish a lantern. */
    double marker_scale_m_;            /**< Visualization marker diameter (meters). */
    std::vector<double> marker_color_; /**< RGBA color to use for markers. */

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;              /**< Buffer used for TF queries. */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; /**< Listener that feeds the TF buffer. */

    // Synchronized subscribers for semantic and depth images
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_semantic_filter_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> sub_depth_filter_;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>
        SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_; /**< Policy-based synchronizer. */

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
        sub_depth_info_;                                      /**< CameraInfo subscription. */
    sensor_msgs::msg::CameraInfo::ConstSharedPtr depth_info_; /**< Cached depth camera intrinsics. */

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr lantern_pub_; /**< Detected lantern poses. */
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr
        lantern_counts_pub_; /**< Observation counts. */
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;                                                            /**< Visualization markers. */
    rclcpp::Publisher<statemachine_pkg::msg::Answer>::SharedPtr heartbeat_pub_; /**< Heartbeat messages. */
    rclcpp::TimerBase::SharedPtr heartbeat_timer_; /**< Timer driving heartbeat publication. */

    std::string heartbeat_topic_;    /**< Topic for heartbeat publications. */
    double heartbeat_period_s_{1.0}; /**< Heartbeat interval (seconds). */

    std::vector<Lantern> detected_lanterns_; /**< Current set of smoothed lantern detections. */
};

}  // namespace lantern_detector

#endif  // LANTERN_DETECTOR_PKG__LANTERN_DETECTOR_NODE_HPP_
