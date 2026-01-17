// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_planning_msgs:msg/PolygonWithHoles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon_with_holes.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__TRAITS_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_planning_msgs/msg/detail/polygon_with_holes__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'hull'
// Member 'holes'
#include "mav_planning_msgs/msg/detail/polygon2_d__traits.hpp"

namespace mav_planning_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PolygonWithHoles & msg,
  std::ostream & out)
{
  out << "{";
  // member: hull
  {
    out << "hull: ";
    to_flow_style_yaml(msg.hull, out);
    out << ", ";
  }

  // member: holes
  {
    if (msg.holes.size() == 0) {
      out << "holes: []";
    } else {
      out << "holes: [";
      size_t pending_items = msg.holes.size();
      for (auto item : msg.holes) {
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
  const PolygonWithHoles & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: hull
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hull:\n";
    to_block_style_yaml(msg.hull, out, indentation + 2);
  }

  // member: holes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.holes.size() == 0) {
      out << "holes: []\n";
    } else {
      out << "holes:\n";
      for (auto item : msg.holes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PolygonWithHoles & msg, bool use_flow_style = false)
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
  const mav_planning_msgs::msg::PolygonWithHoles & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_planning_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_planning_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_planning_msgs::msg::PolygonWithHoles & msg)
{
  return mav_planning_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_planning_msgs::msg::PolygonWithHoles>()
{
  return "mav_planning_msgs::msg::PolygonWithHoles";
}

template<>
inline const char * name<mav_planning_msgs::msg::PolygonWithHoles>()
{
  return "mav_planning_msgs/msg/PolygonWithHoles";
}

template<>
struct has_fixed_size<mav_planning_msgs::msg::PolygonWithHoles>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mav_planning_msgs::msg::PolygonWithHoles>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mav_planning_msgs::msg::PolygonWithHoles>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__TRAITS_HPP_
