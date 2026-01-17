// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_msgs:msg/Actuators.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/actuators.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__ACTUATORS__TRAITS_HPP_
#define MAV_MSGS__MSG__DETAIL__ACTUATORS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_msgs/msg/detail/actuators__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mav_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Actuators & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: angles
  {
    if (msg.angles.size() == 0) {
      out << "angles: []";
    } else {
      out << "angles: [";
      size_t pending_items = msg.angles.size();
      for (auto item : msg.angles) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: angular_velocities
  {
    if (msg.angular_velocities.size() == 0) {
      out << "angular_velocities: []";
    } else {
      out << "angular_velocities: [";
      size_t pending_items = msg.angular_velocities.size();
      for (auto item : msg.angular_velocities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: normalized
  {
    if (msg.normalized.size() == 0) {
      out << "normalized: []";
    } else {
      out << "normalized: [";
      size_t pending_items = msg.normalized.size();
      for (auto item : msg.normalized) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const Actuators & msg,
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

  // member: angles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angles.size() == 0) {
      out << "angles: []\n";
    } else {
      out << "angles:\n";
      for (auto item : msg.angles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: angular_velocities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.angular_velocities.size() == 0) {
      out << "angular_velocities: []\n";
    } else {
      out << "angular_velocities:\n";
      for (auto item : msg.angular_velocities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: normalized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.normalized.size() == 0) {
      out << "normalized: []\n";
    } else {
      out << "normalized:\n";
      for (auto item : msg.normalized) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Actuators & msg, bool use_flow_style = false)
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
  const mav_msgs::msg::Actuators & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_msgs::msg::Actuators & msg)
{
  return mav_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_msgs::msg::Actuators>()
{
  return "mav_msgs::msg::Actuators";
}

template<>
inline const char * name<mav_msgs::msg::Actuators>()
{
  return "mav_msgs/msg/Actuators";
}

template<>
struct has_fixed_size<mav_msgs::msg::Actuators>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mav_msgs::msg::Actuators>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mav_msgs::msg::Actuators>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_MSGS__MSG__DETAIL__ACTUATORS__TRAITS_HPP_
