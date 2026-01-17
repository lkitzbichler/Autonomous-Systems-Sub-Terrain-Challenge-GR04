// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_msgs:msg/GpsWaypoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/gps_waypoint.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__TRAITS_HPP_
#define MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_msgs/msg/detail/gps_waypoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mav_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const GpsWaypoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: latitude
  {
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << ", ";
  }

  // member: longitude
  {
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << ", ";
  }

  // member: altitude
  {
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << ", ";
  }

  // member: heading
  {
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << ", ";
  }

  // member: max_speed
  {
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << ", ";
  }

  // member: max_acc
  {
    out << "max_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acc, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GpsWaypoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.latitude, out);
    out << "\n";
  }

  // member: longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.longitude, out);
    out << "\n";
  }

  // member: altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.altitude, out);
    out << "\n";
  }

  // member: heading
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "heading: ";
    rosidl_generator_traits::value_to_yaml(msg.heading, out);
    out << "\n";
  }

  // member: max_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << "\n";
  }

  // member: max_acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_acc: ";
    rosidl_generator_traits::value_to_yaml(msg.max_acc, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GpsWaypoint & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace mav_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mav_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mav_msgs::msg::GpsWaypoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_msgs::msg::GpsWaypoint & msg)
{
  return mav_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_msgs::msg::GpsWaypoint>()
{
  return "mav_msgs::msg::GpsWaypoint";
}

template<>
inline const char * name<mav_msgs::msg::GpsWaypoint>()
{
  return "mav_msgs/msg/GpsWaypoint";
}

template<>
struct has_fixed_size<mav_msgs::msg::GpsWaypoint>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mav_msgs::msg::GpsWaypoint>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mav_msgs::msg::GpsWaypoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__TRAITS_HPP_
