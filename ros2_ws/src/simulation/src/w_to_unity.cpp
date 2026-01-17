#include <array>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <mav_msgs/msg/actuators.hpp>  // ensure mav_msgs exists in your ROS 2 distro

#include "libsocket/inetclientdgram.hpp"
#include "libsocket/exception.hpp"

class UDPPoseStreamerNode final : public rclcpp::Node
{
public:
  UDPPoseStreamerNode()
  : rclcpp::Node("w_to_unity"), dgram_client_(LIBSOCKET_IPv4)
  {
    // Declare & get parameters (with defaults)
    ip_address_ = this->declare_parameter<std::string>("ip_address", "127.0.0.1");
    port_       = this->declare_parameter<std::string>("port",       "12346");
    offset_x_   = this->declare_parameter<double>("offset_x", 0.0);
    offset_y_   = this->declare_parameter<double>("offset_y", 0.0);

    RCLCPP_INFO(get_logger(),
      "UDP streamer to %s:%s at 1 kHz (offsets x=%.3f, y=%.3f)",
      ip_address_.c_str(), port_.c_str(), offset_x_, offset_y_);

    // Start with NaNs so we don't send until first valid message arrives
    last_w_.fill(std::numeric_limits<float>::quiet_NaN());

    // Subscriber: rotor speeds (ROS 1: "rotor_speed_cmds")
    using mav_msgs::msg::Actuators;
    sub_ = this->create_subscription<Actuators>(
      "rotor_speed_cmds", rclcpp::QoS(1),
      [this](const Actuators::SharedPtr msg) {
        if (msg->angular_velocities.size() < 4) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Actuators.angular_velocities has size %zu (<4); ignoring",
                               msg->angular_velocities.size());
          return;
        }
        last_w_[0] = static_cast<float>(msg->angular_velocities[0]);
        last_w_[1] = static_cast<float>(msg->angular_velocities[1]);
        last_w_[2] = static_cast<float>(msg->angular_velocities[2]);
        last_w_[3] = static_cast<float>(msg->angular_velocities[3]);

        // Debug print (optional)
        RCLCPP_DEBUG(this->get_logger(), "Received w: %.3f %.3f %.3f %.3f",
                     last_w_[0], last_w_[1], last_w_[2], last_w_[3]);
      });

    // Timer at 1 kHz: send latest 4 floats if valid
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      [this]() {
        // Copy to local buffer to avoid partial reads if callback updates in between
        std::array<float,4> w = last_w_;

        bool ok = true;
        for (float v : w) {
          if (std::isnan(v)) { ok = false; break; }
        }
        if (!ok) return;  // No valid data yet

        // If you intended to apply offsets, do that here (currently unused)
        // w[0] += static_cast<float>(offset_x_);
        // w[1] += static_cast<float>(offset_y_);

        try {
          // Pack as raw bytes: 4 floats (16 bytes)
          constexpr size_t pose_size = sizeof(float) * 4;
          uint8_t packet_data[pose_size];
          std::memcpy(packet_data, w.data(), pose_size);

          dgram_client_.sndto(packet_data, pose_size, ip_address_, port_);
        } catch (const libsocket::socket_exception & e) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "UDP send failed: %s", e.mesg.c_str());
        }
      });
  }

  ~UDPPoseStreamerNode() override
  {
    try {
      dgram_client_.destroy();
    } catch (...) {
      // ignore
    }
  }

private:
  // Params
  std::string ip_address_;
  std::string port_;
  double offset_x_{0.0}, offset_y_{0.0};

  // State
  std::array<float,4> last_w_;
  libsocket::inet_dgram_client dgram_client_;

  // ROS 2 handles
  rclcpp::Subscription<mav_msgs::msg::Actuators>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UDPPoseStreamerNode>());
  rclcpp::shutdown();
  return 0;
}
