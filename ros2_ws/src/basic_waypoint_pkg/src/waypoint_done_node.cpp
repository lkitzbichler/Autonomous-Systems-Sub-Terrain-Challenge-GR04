#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <string>
#include <vector>

class WaypointDoneNode : public rclcpp::Node {
public:
  WaypointDoneNode()
  : rclcpp::Node("waypoint_done_node")
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "current_state_est");
    done_topic_ = declare_parameter<std::string>("done_topic", "basic_waypoint/done");
    distance_threshold_ = declare_parameter<double>("distance_threshold", 1.0);
    speed_threshold_ = declare_parameter<double>("speed_threshold", 0.3);
    stop_index_ = declare_parameter<int>("stop_index", -1);
    waypoint_list_ = declare_parameter<std::vector<double>>("waypoints", std::vector<double>());

    if (waypoint_list_.size() < 3 || (waypoint_list_.size() % 3) != 0) {
      RCLCPP_ERROR(get_logger(), "Invalid waypoints parameter (size=%zu)", waypoint_list_.size());
      valid_target_ = false;
    } else {
      const size_t waypoint_count = waypoint_list_.size() / 3;
      size_t target_index = waypoint_count - 1;
      if (stop_index_ >= 0 && static_cast<size_t>(stop_index_) < waypoint_count) {
        target_index = static_cast<size_t>(stop_index_);
      }
      target_ = Eigen::Vector3d(
        waypoint_list_[target_index * 3],
        waypoint_list_[target_index * 3 + 1],
        waypoint_list_[target_index * 3 + 2]);
      valid_target_ = true;
      RCLCPP_INFO(get_logger(), "Waypoint done target = [%.2f, %.2f, %.2f]",
                  target_.x(), target_.y(), target_.z());
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    done_pub_ = create_publisher<std_msgs::msg::Bool>(done_topic_, qos);

    // Publish initial false so late-joining subscribers get a known state.
    std_msgs::msg::Bool msg;
    msg.data = false;
    done_pub_->publish(msg);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&WaypointDoneNode::onOdom, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "waypoint_done_node listening on '%s', publishing '%s'",
                odom_topic_.c_str(), done_topic_.c_str());
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!valid_target_ || done_published_) {
      return;
    }

    const Eigen::Vector3d pos(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    const Eigen::Vector3d vel(
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z);

    const double distance = (pos - target_).norm();
    const double speed = vel.norm();
    if (distance <= distance_threshold_ && speed <= speed_threshold_) {
      std_msgs::msg::Bool done;
      done.data = true;
      done_pub_->publish(done);
      done_published_ = true;
      RCLCPP_INFO(get_logger(), "basic_waypoint/done published (dist=%.2f, speed=%.2f)",
                  distance, speed);
    }
  }

  std::string odom_topic_;
  std::string done_topic_;
  double distance_threshold_;
  double speed_threshold_;
  int stop_index_ = -1;
  std::vector<double> waypoint_list_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr done_pub_;

  Eigen::Vector3d target_ = Eigen::Vector3d::Zero();
  bool valid_target_ = false;
  bool done_published_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointDoneNode>());
  rclcpp::shutdown();
  return 0;
}
