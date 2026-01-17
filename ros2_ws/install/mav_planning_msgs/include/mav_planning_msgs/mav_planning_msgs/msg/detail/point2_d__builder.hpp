// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:msg/Point2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/point2_d.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__BUILDER_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/msg/detail/point2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_Point2D_y
{
public:
  explicit Init_Point2D_y(::mav_planning_msgs::msg::Point2D & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::msg::Point2D y(::mav_planning_msgs::msg::Point2D::_y_type arg)
  {
    msg_.y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::msg::Point2D msg_;
};

class Init_Point2D_x
{
public:
  Init_Point2D_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Point2D_y x(::mav_planning_msgs::msg::Point2D::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Point2D_y(msg_);
  }

private:
  ::mav_planning_msgs::msg::Point2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::msg::Point2D>()
{
  return mav_planning_msgs::msg::builder::Init_Point2D_x();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__BUILDER_HPP_
