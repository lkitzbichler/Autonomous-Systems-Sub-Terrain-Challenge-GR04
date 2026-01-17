// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:msg/PolygonWithHoles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon_with_holes.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__BUILDER_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/msg/detail/polygon_with_holes__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_PolygonWithHoles_holes
{
public:
  explicit Init_PolygonWithHoles_holes(::mav_planning_msgs::msg::PolygonWithHoles & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::msg::PolygonWithHoles holes(::mav_planning_msgs::msg::PolygonWithHoles::_holes_type arg)
  {
    msg_.holes = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolygonWithHoles msg_;
};

class Init_PolygonWithHoles_hull
{
public:
  Init_PolygonWithHoles_hull()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolygonWithHoles_holes hull(::mav_planning_msgs::msg::PolygonWithHoles::_hull_type arg)
  {
    msg_.hull = std::move(arg);
    return Init_PolygonWithHoles_holes(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolygonWithHoles msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::msg::PolygonWithHoles>()
{
  return mav_planning_msgs::msg::builder::Init_PolygonWithHoles_hull();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__BUILDER_HPP_
