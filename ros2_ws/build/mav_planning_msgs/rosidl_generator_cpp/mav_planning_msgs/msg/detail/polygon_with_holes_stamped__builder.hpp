// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_planning_msgs:msg/PolygonWithHolesStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon_with_holes_stamped.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES_STAMPED__BUILDER_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_planning_msgs/msg/detail/polygon_with_holes_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_planning_msgs
{

namespace msg
{

namespace builder
{

class Init_PolygonWithHolesStamped_polygon
{
public:
  explicit Init_PolygonWithHolesStamped_polygon(::mav_planning_msgs::msg::PolygonWithHolesStamped & msg)
  : msg_(msg)
  {}
  ::mav_planning_msgs::msg::PolygonWithHolesStamped polygon(::mav_planning_msgs::msg::PolygonWithHolesStamped::_polygon_type arg)
  {
    msg_.polygon = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolygonWithHolesStamped msg_;
};

class Init_PolygonWithHolesStamped_altitude
{
public:
  explicit Init_PolygonWithHolesStamped_altitude(::mav_planning_msgs::msg::PolygonWithHolesStamped & msg)
  : msg_(msg)
  {}
  Init_PolygonWithHolesStamped_polygon altitude(::mav_planning_msgs::msg::PolygonWithHolesStamped::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_PolygonWithHolesStamped_polygon(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolygonWithHolesStamped msg_;
};

class Init_PolygonWithHolesStamped_header
{
public:
  Init_PolygonWithHolesStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PolygonWithHolesStamped_altitude header(::mav_planning_msgs::msg::PolygonWithHolesStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PolygonWithHolesStamped_altitude(msg_);
  }

private:
  ::mav_planning_msgs::msg::PolygonWithHolesStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_planning_msgs::msg::PolygonWithHolesStamped>()
{
  return mav_planning_msgs::msg::builder::Init_PolygonWithHolesStamped_header();
}

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES_STAMPED__BUILDER_HPP_
