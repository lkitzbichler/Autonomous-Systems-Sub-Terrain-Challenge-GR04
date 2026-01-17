// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_msgs:msg/RollPitchYawrateThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/roll_pitch_yawrate_thrust.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__ROLL_PITCH_YAWRATE_THRUST__BUILDER_HPP_
#define MAV_MSGS__MSG__DETAIL__ROLL_PITCH_YAWRATE_THRUST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_msgs/msg/detail/roll_pitch_yawrate_thrust__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_msgs
{

namespace msg
{

namespace builder
{

class Init_RollPitchYawrateThrust_thrust
{
public:
  explicit Init_RollPitchYawrateThrust_thrust(::mav_msgs::msg::RollPitchYawrateThrust & msg)
  : msg_(msg)
  {}
  ::mav_msgs::msg::RollPitchYawrateThrust thrust(::mav_msgs::msg::RollPitchYawrateThrust::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_msgs::msg::RollPitchYawrateThrust msg_;
};

class Init_RollPitchYawrateThrust_yaw_rate
{
public:
  explicit Init_RollPitchYawrateThrust_yaw_rate(::mav_msgs::msg::RollPitchYawrateThrust & msg)
  : msg_(msg)
  {}
  Init_RollPitchYawrateThrust_thrust yaw_rate(::mav_msgs::msg::RollPitchYawrateThrust::_yaw_rate_type arg)
  {
    msg_.yaw_rate = std::move(arg);
    return Init_RollPitchYawrateThrust_thrust(msg_);
  }

private:
  ::mav_msgs::msg::RollPitchYawrateThrust msg_;
};

class Init_RollPitchYawrateThrust_pitch
{
public:
  explicit Init_RollPitchYawrateThrust_pitch(::mav_msgs::msg::RollPitchYawrateThrust & msg)
  : msg_(msg)
  {}
  Init_RollPitchYawrateThrust_yaw_rate pitch(::mav_msgs::msg::RollPitchYawrateThrust::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_RollPitchYawrateThrust_yaw_rate(msg_);
  }

private:
  ::mav_msgs::msg::RollPitchYawrateThrust msg_;
};

class Init_RollPitchYawrateThrust_roll
{
public:
  explicit Init_RollPitchYawrateThrust_roll(::mav_msgs::msg::RollPitchYawrateThrust & msg)
  : msg_(msg)
  {}
  Init_RollPitchYawrateThrust_pitch roll(::mav_msgs::msg::RollPitchYawrateThrust::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_RollPitchYawrateThrust_pitch(msg_);
  }

private:
  ::mav_msgs::msg::RollPitchYawrateThrust msg_;
};

class Init_RollPitchYawrateThrust_header
{
public:
  Init_RollPitchYawrateThrust_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RollPitchYawrateThrust_roll header(::mav_msgs::msg::RollPitchYawrateThrust::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RollPitchYawrateThrust_roll(msg_);
  }

private:
  ::mav_msgs::msg::RollPitchYawrateThrust msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_msgs::msg::RollPitchYawrateThrust>()
{
  return mav_msgs::msg::builder::Init_RollPitchYawrateThrust_header();
}

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__ROLL_PITCH_YAWRATE_THRUST__BUILDER_HPP_
