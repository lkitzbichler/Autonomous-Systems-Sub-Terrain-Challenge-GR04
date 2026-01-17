// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:msg/Polygon2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon2_d.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__BUILDER_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/msg/detail/polygon2_d__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_Polygon2D_points
{
public:
  Init_Polygon2D_points()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mav_planning_msgs::msg::Polygon2D points(::mav_planning_msgs::msg::Polygon2D::_points_type arg)
  {
    msg_.points = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::msg::Polygon2D msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::msg::Polygon2D>()
{
  return mav_planning_msgs::msg::builder::Init_Polygon2D_points();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__BUILDER_HPP_
