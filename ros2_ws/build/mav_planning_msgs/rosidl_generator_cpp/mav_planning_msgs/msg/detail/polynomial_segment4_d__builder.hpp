// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:msg/PolynomialSegment4D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polynomial_segment4_d.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT4_D__BUILDER_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT4_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/msg/detail/polynomial_segment4_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_PolynomialSegment4D_yaw
{
public:
  explicit Init_PolynomialSegment4D_yaw(::mav_planning_msgs::msg::PolynomialSegment4D & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::msg::PolynomialSegment4D yaw(::mav_planning_msgs::msg::PolynomialSegment4D::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment4D msg_;
};

class Init_PolynomialSegment4D_z
{
public:
  explicit Init_PolynomialSegment4D_z(::mav_planning_msgs::msg::PolynomialSegment4D & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment4D_yaw z(::mav_planning_msgs::msg::PolynomialSegment4D::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_PolynomialSegment4D_yaw(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment4D msg_;
};

class Init_PolynomialSegment4D_y
{
public:
  explicit Init_PolynomialSegment4D_y(::mav_planning_msgs::msg::PolynomialSegment4D & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment4D_z y(::mav_planning_msgs::msg::PolynomialSegment4D::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_PolynomialSegment4D_z(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment4D msg_;
};

class Init_PolynomialSegment4D_x
{
public:
  explicit Init_PolynomialSegment4D_x(::mav_planning_msgs::msg::PolynomialSegment4D & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment4D_y x(::mav_planning_msgs::msg::PolynomialSegment4D::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PolynomialSegment4D_y(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment4D msg_;
};

class Init_PolynomialSegment4D_segment_time
{
public:
  explicit Init_PolynomialSegment4D_segment_time(::mav_planning_msgs::msg::PolynomialSegment4D & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment4D_x segment_time(::mav_planning_msgs::msg::PolynomialSegment4D::_segment_time_type arg)
  {
    msg_.segment_time = std::move(arg);
    return Init_PolynomialSegment4D_x(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment4D msg_;
};

class Init_PolynomialSegment4D_num_coeffs
{
public:
  explicit Init_PolynomialSegment4D_num_coeffs(::mav_planning_msgs::msg::PolynomialSegment4D & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment4D_segment_time num_coeffs(::mav_planning_msgs::msg::PolynomialSegment4D::_num_coeffs_type arg)
  {
    msg_.num_coeffs = std::move(arg);
    return Init_PolynomialSegment4D_segment_time(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment4D msg_;
};

class Init_PolynomialSegment4D_header
{
public:
  Init_PolynomialSegment4D_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolynomialSegment4D_num_coeffs header(::mav_planning_msgs::msg::PolynomialSegment4D::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PolynomialSegment4D_num_coeffs(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment4D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::msg::PolynomialSegment4D>()
{
  return mav_planning_msgs::msg::builder::Init_PolynomialSegment4D_header();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT4_D__BUILDER_HPP_
