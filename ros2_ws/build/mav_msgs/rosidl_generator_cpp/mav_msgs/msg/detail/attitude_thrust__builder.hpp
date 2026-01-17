// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_msgs:msg/AttitudeThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/attitude_thrust.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__BUILDER_HPP_
#define MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_msgs/msg/detail/attitude_thrust__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_msgs
{

namespace msg
{

namespace builder
{

class Init_AttitudeThrust_thrust
{
public:
  explicit Init_AttitudeThrust_thrust(::mav_msgs::msg::AttitudeThrust & msg)
  : msg_(msg)
  {}
  ::mav_msgs::msg::AttitudeThrust thrust(::mav_msgs::msg::AttitudeThrust::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_msgs::msg::AttitudeThrust msg_;
};

class Init_AttitudeThrust_attitude
{
public:
  explicit Init_AttitudeThrust_attitude(::mav_msgs::msg::AttitudeThrust & msg)
  : msg_(msg)
  {}
  Init_AttitudeThrust_thrust attitude(::mav_msgs::msg::AttitudeThrust::_attitude_type arg)
  {
    msg_.attitude = std::move(arg);
    return Init_AttitudeThrust_thrust(msg_);
  }

private:
  ::mav_msgs::msg::AttitudeThrust msg_;
};

class Init_AttitudeThrust_header
{
public:
  Init_AttitudeThrust_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AttitudeThrust_attitude header(::mav_msgs::msg::AttitudeThrust::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_AttitudeThrust_attitude(msg_);
  }

private:
  ::mav_msgs::msg::AttitudeThrust msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_msgs::msg::AttitudeThrust>()
{
  return mav_msgs::msg::builder::Init_AttitudeThrust_header();
}

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__BUILDER_HPP_
