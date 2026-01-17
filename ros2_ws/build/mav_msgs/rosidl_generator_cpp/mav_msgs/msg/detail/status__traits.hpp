// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_msgs:msg/Status.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/status.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
#define MAV_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace mav_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Status & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: vehicle_name
  {
    out << "vehicle_name: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_name, out);
    out << ", ";
  }

  // member: vehicle_type
  {
    out << "vehicle_type: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_type, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: rc_command_mode
  {
    out << "rc_command_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.rc_command_mode, out);
    out << ", ";
  }

  // member: command_interface_enabled
  {
    out << "command_interface_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.command_interface_enabled, out);
    out << ", ";
  }

  // member: flight_time
  {
    out << "flight_time: ";
    rosidl_generator_traits::value_to_yaml(msg.flight_time, out);
    out << ", ";
  }

  // member: system_uptime
  {
    out << "system_uptime: ";
    rosidl_generator_traits::value_to_yaml(msg.system_uptime, out);
    out << ", ";
  }

  // member: cpu_load
  {
    out << "cpu_load: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_load, out);
    out << ", ";
  }

  // member: motor_status
  {
    out << "motor_status: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_status, out);
    out << ", ";
  }

  // member: in_air
  {
    out << "in_air: ";
    rosidl_generator_traits::value_to_yaml(msg.in_air, out);
    out << ", ";
  }

  // member: gps_status
  {
    out << "gps_status: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_status, out);
    out << ", ";
  }

  // member: gps_num_satellites
  {
    out << "gps_num_satellites: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_num_satellites, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Status & msg,
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

  // member: vehicle_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_name: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_name, out);
    out << "\n";
  }

  // member: vehicle_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vehicle_type: ";
    rosidl_generator_traits::value_to_yaml(msg.vehicle_type, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: rc_command_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rc_command_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.rc_command_mode, out);
    out << "\n";
  }

  // member: command_interface_enabled
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "command_interface_enabled: ";
    rosidl_generator_traits::value_to_yaml(msg.command_interface_enabled, out);
    out << "\n";
  }

  // member: flight_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "flight_time: ";
    rosidl_generator_traits::value_to_yaml(msg.flight_time, out);
    out << "\n";
  }

  // member: system_uptime
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_uptime: ";
    rosidl_generator_traits::value_to_yaml(msg.system_uptime, out);
    out << "\n";
  }

  // member: cpu_load
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cpu_load: ";
    rosidl_generator_traits::value_to_yaml(msg.cpu_load, out);
    out << "\n";
  }

  // member: motor_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_status: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_status, out);
    out << "\n";
  }

  // member: in_air
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "in_air: ";
    rosidl_generator_traits::value_to_yaml(msg.in_air, out);
    out << "\n";
  }

  // member: gps_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_status: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_status, out);
    out << "\n";
  }

  // member: gps_num_satellites
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_num_satellites: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_num_satellites, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Status & msg, bool use_flow_style = false)
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
  const mav_msgs::msg::Status & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_msgs::msg::Status & msg)
{
  return mav_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_msgs::msg::Status>()
{
  return "mav_msgs::msg::Status";
}

template<>
inline const char * name<mav_msgs::msg::Status>()
{
  return "mav_msgs/msg/Status";
}

template<>
struct has_fixed_size<mav_msgs::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mav_msgs::msg::Status>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mav_msgs::msg::Status>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_MSGS__MSG__DETAIL__STATUS__TRAITS_HPP_
