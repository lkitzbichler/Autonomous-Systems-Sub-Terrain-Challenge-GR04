// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:msg/PolynomialTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polynomial_trajectory.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__BUILDER_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/msg/detail/polynomial_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_PolynomialTrajectory_segments
{
public:
  explicit Init_PolynomialTrajectory_segments(::mav_planning_msgs::msg::PolynomialTrajectory & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::msg::PolynomialTrajectory segments(::mav_planning_msgs::msg::PolynomialTrajectory::_segments_type arg)
  {
    msg_.segments = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialTrajectory msg_;
};

class Init_PolynomialTrajectory_header
{
public:
  Init_PolynomialTrajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolynomialTrajectory_segments header(::mav_planning_msgs::msg::PolynomialTrajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PolynomialTrajectory_segments(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialTrajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::msg::PolynomialTrajectory>()
{
  return mav_planning_msgs::msg::builder::Init_PolynomialTrajectory_header();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__BUILDER_HPP_
