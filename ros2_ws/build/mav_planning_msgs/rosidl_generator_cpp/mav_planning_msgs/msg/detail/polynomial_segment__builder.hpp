// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:msg/PolynomialSegment.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polynomial_segment.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT__BUILDER_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/msg/detail/polynomial_segment__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_PolynomialSegment_yaw
{
public:
  explicit Init_PolynomialSegment_yaw(::mav_planning_msgs::msg::PolynomialSegment & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::msg::PolynomialSegment yaw(::mav_planning_msgs::msg::PolynomialSegment::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

class Init_PolynomialSegment_rz
{
public:
  explicit Init_PolynomialSegment_rz(::mav_planning_msgs::msg::PolynomialSegment & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment_yaw rz(::mav_planning_msgs::msg::PolynomialSegment::_rz_type arg)
  {
    msg_.rz = std::move(arg);
    return Init_PolynomialSegment_yaw(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

class Init_PolynomialSegment_ry
{
public:
  explicit Init_PolynomialSegment_ry(::mav_planning_msgs::msg::PolynomialSegment & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment_rz ry(::mav_planning_msgs::msg::PolynomialSegment::_ry_type arg)
  {
    msg_.ry = std::move(arg);
    return Init_PolynomialSegment_rz(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

class Init_PolynomialSegment_rx
{
public:
  explicit Init_PolynomialSegment_rx(::mav_planning_msgs::msg::PolynomialSegment & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment_ry rx(::mav_planning_msgs::msg::PolynomialSegment::_rx_type arg)
  {
    msg_.rx = std::move(arg);
    return Init_PolynomialSegment_ry(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

class Init_PolynomialSegment_z
{
public:
  explicit Init_PolynomialSegment_z(::mav_planning_msgs::msg::PolynomialSegment & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment_rx z(::mav_planning_msgs::msg::PolynomialSegment::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_PolynomialSegment_rx(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

class Init_PolynomialSegment_y
{
public:
  explicit Init_PolynomialSegment_y(::mav_planning_msgs::msg::PolynomialSegment & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment_z y(::mav_planning_msgs::msg::PolynomialSegment::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_PolynomialSegment_z(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

class Init_PolynomialSegment_x
{
public:
  explicit Init_PolynomialSegment_x(::mav_planning_msgs::msg::PolynomialSegment & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment_y x(::mav_planning_msgs::msg::PolynomialSegment::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PolynomialSegment_y(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

class Init_PolynomialSegment_segment_time
{
public:
  explicit Init_PolynomialSegment_segment_time(::mav_planning_msgs::msg::PolynomialSegment & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment_x segment_time(::mav_planning_msgs::msg::PolynomialSegment::_segment_time_type arg)
  {
    msg_.segment_time = std::move(arg);
    return Init_PolynomialSegment_x(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

class Init_PolynomialSegment_num_coeffs
{
public:
  explicit Init_PolynomialSegment_num_coeffs(::mav_planning_msgs::msg::PolynomialSegment & msg)
  : msg_(msg)
  {}
  Init_PolynomialSegment_segment_time num_coeffs(::mav_planning_msgs::msg::PolynomialSegment::_num_coeffs_type arg)
  {
    msg_.num_coeffs = std::move(arg);
    return Init_PolynomialSegment_segment_time(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

class Init_PolynomialSegment_header
{
public:
  Init_PolynomialSegment_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolynomialSegment_num_coeffs header(::mav_planning_msgs::msg::PolynomialSegment::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PolynomialSegment_num_coeffs(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolynomialSegment msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::msg::PolynomialSegment>()
{
  return mav_planning_msgs::msg::builder::Init_PolynomialSegment_header();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT__BUILDER_HPP_
