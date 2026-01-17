// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_msgs:msg/Actuators.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/actuators.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__ACTUATORS__BUILDER_HPP_
#define MAV_MSGS__MSG__DETAIL__ACTUATORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_msgs/msg/detail/actuators__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_msgs
{

namespace msg
{

namespace builder
{

class Init_Actuators_normalized
{
public:
  explicit Init_Actuators_normalized(::mav_msgs::msg::Actuators & msg)
  : msg_(msg)
  {}
  ::mav_msgs::msg::Actuators normalized(::mav_msgs::msg::Actuators::_normalized_type arg)
  {
    msg_.normalized = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_msgs::msg::Actuators msg_;
};

class Init_Actuators_angular_velocities
{
public:
  explicit Init_Actuators_angular_velocities(::mav_msgs::msg::Actuators & msg)
  : msg_(msg)
  {}
  Init_Actuators_normalized angular_velocities(::mav_msgs::msg::Actuators::_angular_velocities_type arg)
  {
    msg_.angular_velocities = std::move(arg);
    return Init_Actuators_normalized(msg_);
  }

private:
  ::mav_msgs::msg::Actuators msg_;
};

class Init_Actuators_angles
{
public:
  explicit Init_Actuators_angles(::mav_msgs::msg::Actuators & msg)
  : msg_(msg)
  {}
  Init_Actuators_angular_velocities angles(::mav_msgs::msg::Actuators::_angles_type arg)
  {
    msg_.angles = std::move(arg);
    return Init_Actuators_angular_velocities(msg_);
  }

private:
  ::mav_msgs::msg::Actuators msg_;
};

class Init_Actuators_header
{
public:
  Init_Actuators_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Actuators_angles header(::mav_msgs::msg::Actuators::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Actuators_angles(msg_);
  }

private:
  ::mav_msgs::msg::Actuators msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_msgs::msg::Actuators>()
{
  return mav_msgs::msg::builder::Init_Actuators_header();
}

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__ACTUATORS__BUILDER_HPP_
