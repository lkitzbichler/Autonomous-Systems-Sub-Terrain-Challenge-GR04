// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_planning_msgs:msg/Polygon2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon2_d.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__TRAITS_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_planning_msgs/msg/detail/polygon2_d__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'points'
#include "mav_planning_msgs/msg/detail/point2_d__traits.hpp"

namespace mav_planning_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Polygon2D & msg,
  std::ostream & out)
{
  out << "{";
  // member: points
  {
    if (msg.points.size() == 0) {
      out << "points: []";
    } else {
      out << "points: [";
      size_t pending_items = msg.points.size();
      for (auto item : msg.points) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Polygon2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: points
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.points.size() == 0) {
      out << "points: []\n";
    } else {
      out << "points:\n";
      for (auto item : msg.points) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Polygon2D & msg, bool use_flow_style = false)
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
  const mav_planning_msgs::msg::Polygon2D & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_planning_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_planning_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_planning_msgs::msg::Polygon2D & msg)
{
  return mav_planning_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_planning_msgs::msg::Polygon2D>()
{
  return "mav_planning_msgs::msg::Polygon2D";
}

template<>
inline const char * name<mav_planning_msgs::msg::Polygon2D>()
{
  return "mav_planning_msgs/msg/Polygon2D";
}

template<>
struct has_fixed_size<mav_planning_msgs::msg::Polygon2D>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mav_planning_msgs::msg::Polygon2D>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mav_planning_msgs::msg::Polygon2D>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__TRAITS_HPP_
