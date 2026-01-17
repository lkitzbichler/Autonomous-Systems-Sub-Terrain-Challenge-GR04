// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_planning_msgs:msg/Point2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/point2_d.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__TRAITS_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_planning_msgs/msg/detail/point2_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace mav_planning_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Point2D & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Point2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Point2D & msg, bool use_flow_style = false)
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

}  // namespace mav_planning_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mav_planning_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mav_planning_msgs::msg::Point2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_planning_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_planning_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_planning_msgs::msg::Point2D & msg)
{
  return mav_planning_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_planning_msgs::msg::Point2D>()
{
  return "mav_planning_msgs::msg::Point2D";
}

template<>
inline const char * name<mav_planning_msgs::msg::Point2D>()
{
  return "mav_planning_msgs/msg/Point2D";
}

template<>
struct has_fixed_size<mav_planning_msgs::msg::Point2D>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<mav_planning_msgs::msg::Point2D>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<mav_planning_msgs::msg::Point2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__TRAITS_HPP_
