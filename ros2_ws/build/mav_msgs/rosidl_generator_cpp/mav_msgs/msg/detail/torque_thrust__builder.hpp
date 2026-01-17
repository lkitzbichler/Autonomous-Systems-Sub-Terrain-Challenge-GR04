// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_msgs:msg/TorqueThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/torque_thrust.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__TORQUE_THRUST__BUILDER_HPP_
#define MAV_MSGS__MSG__DETAIL__TORQUE_THRUST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_msgs/msg/detail/torque_thrust__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_msgs
{

namespace msg
{

namespace builder
{

class Init_TorqueThrust_thrust
{
public:
  explicit Init_TorqueThrust_thrust(::mav_msgs::msg::TorqueThrust & msg)
  : msg_(msg)
  {}
  ::mav_msgs::msg::TorqueThrust thrust(::mav_msgs::msg::TorqueThrust::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_msgs::msg::TorqueThrust msg_;
};

class Init_TorqueThrust_torque
{
public:
  explicit Init_TorqueThrust_torque(::mav_msgs::msg::TorqueThrust & msg)
  : msg_(msg)
  {}
  Init_TorqueThrust_thrust torque(::mav_msgs::msg::TorqueThrust::_torque_type arg)
  {
    msg_.torque = std::move(arg);
    return Init_TorqueThrust_thrust(msg_);
  }

private:
  ::mav_msgs::msg::TorqueThrust msg_;
};

class Init_TorqueThrust_header
{
public:
  Init_TorqueThrust_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TorqueThrust_torque header(::mav_msgs::msg::TorqueThrust::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TorqueThrust_torque(msg_);
  }

private:
  ::mav_msgs::msg::TorqueThrust msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_msgs::msg::TorqueThrust>()
{
  return mav_msgs::msg::builder::Init_TorqueThrust_header();
}

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__TORQUE_THRUST__BUILDER_HPP_
