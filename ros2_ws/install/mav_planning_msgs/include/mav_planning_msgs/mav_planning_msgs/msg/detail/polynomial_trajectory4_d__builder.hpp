// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:msg/PolynomialTrajectory4D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polynomial_trajectory4_d.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY4_D__BUILDER_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY4_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/msg/detail/polynomial_trajectory4_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_PolynomialTrajectory4D_segments
{
public:
  explicit Init_PolynomialTrajectory4D_segments(::mav_planning_msgs::msg::PolynomialTrajectory4D & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::msg::PolynomialTrajectory4D segments(::mav_planning_msgs::msg::PolynomialTrajectory4D::_segments_type arg)
  {
    msg_.segments = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialTrajectory4D msg_;
};

class Init_PolynomialTrajectory4D_header
{
public:
  Init_PolynomialTrajectory4D_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolynomialTrajectory4D_segments header(::mav_planning_msgs::msg::PolynomialTrajectory4D::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PolynomialTrajectory4D_segments(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialTrajectory4D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::msg::PolynomialTrajectory4D>()
{
  return mav_planning_msgs::msg::builder::Init_PolynomialTrajectory4D_header();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY4_D__BUILDER_HPP_
