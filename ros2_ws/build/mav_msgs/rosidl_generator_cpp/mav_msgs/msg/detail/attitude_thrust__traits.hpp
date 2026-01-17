// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_msgs:msg/AttitudeThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/attitude_thrust.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__TRAITS_HPP_
#define MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_msgs/msg/detail/attitude_thrust__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'attitude'
#include "geometry_msgs/msg/detail/quaternion__traits.hpp"
// Member 'thrust'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace mav_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AttitudeThrust & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: attitude
  {
    out << "attitude: ";
    to_flow_style_yaml(msg.attitude, out);
    out << ", ";
  }

  // member: thrust
  {
    out << "thrust: ";
    to_flow_style_yaml(msg.thrust, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AttitudeThrust & msg,
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

  // member: attitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attitude:\n";
    to_block_style_yaml(msg.attitude, out, indentation + 2);
  }

  // member: thrust
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "thrust:\n";
    to_block_style_yaml(msg.thrust, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AttitudeThrust & msg, bool use_flow_style = false)
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
  const mav_msgs::msg::AttitudeThrust & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_msgs::msg::AttitudeThrust & msg)
{
  return mav_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_msgs::msg::AttitudeThrust>()
{
  return "mav_msgs::msg::AttitudeThrust";
}

template<>
inline const char * name<mav_msgs::msg::AttitudeThrust>()
{
  return "mav_msgs/msg/AttitudeThrust";
}

template<>
struct has_fixed_size<mav_msgs::msg::AttitudeThrust>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Quaternion>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mav_msgs::msg::AttitudeThrust>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Quaternion>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mav_msgs::msg::AttitudeThrust>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__TRAITS_HPP_
