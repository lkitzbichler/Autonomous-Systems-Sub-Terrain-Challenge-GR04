// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_msgs:msg/RateThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/rate_thrust.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__RATE_THRUST__BUILDER_HPP_
#define MAV_MSGS__MSG__DETAIL__RATE_THRUST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_msgs/msg/detail/rate_thrust__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_msgs
{

namespace msg
{

namespace builder
{

class Init_RateThrust_thrust
{
public:
  explicit Init_RateThrust_thrust(::mav_msgs::msg::RateThrust & msg)
  : msg_(msg)
  {}
  ::mav_msgs::msg::RateThrust thrust(::mav_msgs::msg::RateThrust::_thrust_type arg)
  {
    msg_.thrust = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_msgs::msg::RateThrust msg_;
};

class Init_RateThrust_angular_rates
{
public:
  explicit Init_RateThrust_angular_rates(::mav_msgs::msg::RateThrust & msg)
  : msg_(msg)
  {}
  Init_RateThrust_thrust angular_rates(::mav_msgs::msg::RateThrust::_angular_rates_type arg)
  {
    msg_.angular_rates = std::move(arg);
    return Init_RateThrust_thrust(msg_);
  }

private:
  ::mav_msgs::msg::RateThrust msg_;
};

class Init_RateThrust_header
{
public:
  Init_RateThrust_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RateThrust_angular_rates header(::mav_msgs::msg::RateThrust::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RateThrust_angular_rates(msg_);
  }

private:
  ::mav_msgs::msg::RateThrust msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_msgs::msg::RateThrust>()
{
  return mav_msgs::msg::builder::Init_RateThrust_header();
}

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__RATE_THRUST__BUILDER_HPP_
