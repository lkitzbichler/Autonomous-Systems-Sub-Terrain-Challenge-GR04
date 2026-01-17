// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_msgs:msg/GpsWaypoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/gps_waypoint.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__BUILDER_HPP_
#define MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_msgs/msg/detail/gps_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_msgs
{

namespace msg
{

namespace builder
{

class Init_GpsWaypoint_max_acc
{
public:
  explicit Init_GpsWaypoint_max_acc(::mav_msgs::msg::GpsWaypoint & msg)
  : msg_(msg)
  {}
  ::mav_msgs::msg::GpsWaypoint max_acc(::mav_msgs::msg::GpsWaypoint::_max_acc_type arg)
  {
    msg_.max_acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_msgs::msg::GpsWaypoint msg_;
};

class Init_GpsWaypoint_max_speed
{
public:
  explicit Init_GpsWaypoint_max_speed(::mav_msgs::msg::GpsWaypoint & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_max_acc max_speed(::mav_msgs::msg::GpsWaypoint::_max_speed_type arg)
  {
    msg_.max_speed = std::move(arg);
    return Init_GpsWaypoint_max_acc(msg_);
  }

private:
  ::mav_msgs::msg::GpsWaypoint msg_;
};

class Init_GpsWaypoint_heading
{
public:
  explicit Init_GpsWaypoint_heading(::mav_msgs::msg::GpsWaypoint & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_max_speed heading(::mav_msgs::msg::GpsWaypoint::_heading_type arg)
  {
    msg_.heading = std::move(arg);
    return Init_GpsWaypoint_max_speed(msg_);
  }

private:
  ::mav_msgs::msg::GpsWaypoint msg_;
};

class Init_GpsWaypoint_altitude
{
public:
  explicit Init_GpsWaypoint_altitude(::mav_msgs::msg::GpsWaypoint & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_heading altitude(::mav_msgs::msg::GpsWaypoint::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_GpsWaypoint_heading(msg_);
  }

private:
  ::mav_msgs::msg::GpsWaypoint msg_;
};

class Init_GpsWaypoint_longitude
{
public:
  explicit Init_GpsWaypoint_longitude(::mav_msgs::msg::GpsWaypoint & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_altitude longitude(::mav_msgs::msg::GpsWaypoint::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GpsWaypoint_altitude(msg_);
  }

private:
  ::mav_msgs::msg::GpsWaypoint msg_;
};

class Init_GpsWaypoint_latitude
{
public:
  explicit Init_GpsWaypoint_latitude(::mav_msgs::msg::GpsWaypoint & msg)
  : msg_(msg)
  {}
  Init_GpsWaypoint_longitude latitude(::mav_msgs::msg::GpsWaypoint::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GpsWaypoint_longitude(msg_);
  }

private:
  ::mav_msgs::msg::GpsWaypoint msg_;
};

class Init_GpsWaypoint_header
{
public:
  Init_GpsWaypoint_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsWaypoint_latitude header(::mav_msgs::msg::GpsWaypoint::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GpsWaypoint_latitude(msg_);
  }

private:
  ::mav_msgs::msg::GpsWaypoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_msgs::msg::GpsWaypoint>()
{
  return mav_msgs::msg::builder::Init_GpsWaypoint_header();
}

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__BUILDER_HPP_
