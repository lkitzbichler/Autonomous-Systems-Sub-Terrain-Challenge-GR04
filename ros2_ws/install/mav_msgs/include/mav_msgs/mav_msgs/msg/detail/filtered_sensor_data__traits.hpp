// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_msgs:msg/FilteredSensorData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/filtered_sensor_data.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__TRAITS_HPP_
#define MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_msgs/msg/detail/filtered_sensor_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'accelerometer'
// Member 'gyroscope'
// Member 'magnetometer'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace mav_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const FilteredSensorData & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: accelerometer
  {
    out << "accelerometer: ";
    to_flow_style_yaml(msg.accelerometer, out);
    out << ", ";
  }

  // member: gyroscope
  {
    out << "gyroscope: ";
    to_flow_style_yaml(msg.gyroscope, out);
    out << ", ";
  }

  // member: magnetometer
  {
    out << "magnetometer: ";
    to_flow_style_yaml(msg.magnetometer, out);
    out << ", ";
  }

  // member: barometer
  {
    out << "barometer: ";
    rosidl_generator_traits::value_to_yaml(msg.barometer, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const FilteredSensorData & msg,
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

  // member: accelerometer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "accelerometer:\n";
    to_block_style_yaml(msg.accelerometer, out, indentation + 2);
  }

  // member: gyroscope
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gyroscope:\n";
    to_block_style_yaml(msg.gyroscope, out, indentation + 2);
  }

  // member: magnetometer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "magnetometer:\n";
    to_block_style_yaml(msg.magnetometer, out, indentation + 2);
  }

  // member: barometer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "barometer: ";
    rosidl_generator_traits::value_to_yaml(msg.barometer, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const FilteredSensorData & msg, bool use_flow_style = false)
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
  const mav_msgs::msg::FilteredSensorData & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_msgs::msg::FilteredSensorData & msg)
{
  return mav_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_msgs::msg::FilteredSensorData>()
{
  return "mav_msgs::msg::FilteredSensorData";
}

template<>
inline const char * name<mav_msgs::msg::FilteredSensorData>()
{
  return "mav_msgs/msg/FilteredSensorData";
}

template<>
struct has_fixed_size<mav_msgs::msg::FilteredSensorData>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<mav_msgs::msg::FilteredSensorData>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<mav_msgs::msg::FilteredSensorData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__TRAITS_HPP_
