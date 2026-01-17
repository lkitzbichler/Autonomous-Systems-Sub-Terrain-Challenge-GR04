// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mav_msgs:msg/Status.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/status.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
#define MAV_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mav_msgs/msg/detail/status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mav_msgs
{

namespace msg
{

namespace builder
{

class Init_Status_gps_num_satellites
{
public:
  explicit Init_Status_gps_num_satellites(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  ::mav_msgs::msg::Status gps_num_satellites(::mav_msgs::msg::Status::_gps_num_satellites_type arg)
  {
    msg_.gps_num_satellites = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_gps_status
{
public:
  explicit Init_Status_gps_status(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_gps_num_satellites gps_status(::mav_msgs::msg::Status::_gps_status_type arg)
  {
    msg_.gps_status = std::move(arg);
    return Init_Status_gps_num_satellites(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_in_air
{
public:
  explicit Init_Status_in_air(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_gps_status in_air(::mav_msgs::msg::Status::_in_air_type arg)
  {
    msg_.in_air = std::move(arg);
    return Init_Status_gps_status(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_motor_status
{
public:
  explicit Init_Status_motor_status(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_in_air motor_status(::mav_msgs::msg::Status::_motor_status_type arg)
  {
    msg_.motor_status = std::move(arg);
    return Init_Status_in_air(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_cpu_load
{
public:
  explicit Init_Status_cpu_load(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_motor_status cpu_load(::mav_msgs::msg::Status::_cpu_load_type arg)
  {
    msg_.cpu_load = std::move(arg);
    return Init_Status_motor_status(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_system_uptime
{
public:
  explicit Init_Status_system_uptime(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_cpu_load system_uptime(::mav_msgs::msg::Status::_system_uptime_type arg)
  {
    msg_.system_uptime = std::move(arg);
    return Init_Status_cpu_load(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_flight_time
{
public:
  explicit Init_Status_flight_time(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_system_uptime flight_time(::mav_msgs::msg::Status::_flight_time_type arg)
  {
    msg_.flight_time = std::move(arg);
    return Init_Status_system_uptime(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_command_interface_enabled
{
public:
  explicit Init_Status_command_interface_enabled(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_flight_time command_interface_enabled(::mav_msgs::msg::Status::_command_interface_enabled_type arg)
  {
    msg_.command_interface_enabled = std::move(arg);
    return Init_Status_flight_time(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_rc_command_mode
{
public:
  explicit Init_Status_rc_command_mode(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_command_interface_enabled rc_command_mode(::mav_msgs::msg::Status::_rc_command_mode_type arg)
  {
    msg_.rc_command_mode = std::move(arg);
    return Init_Status_command_interface_enabled(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_battery_voltage
{
public:
  explicit Init_Status_battery_voltage(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_rc_command_mode battery_voltage(::mav_msgs::msg::Status::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_Status_rc_command_mode(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_vehicle_type
{
public:
  explicit Init_Status_vehicle_type(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_battery_voltage vehicle_type(::mav_msgs::msg::Status::_vehicle_type_type arg)
  {
    msg_.vehicle_type = std::move(arg);
    return Init_Status_battery_voltage(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_vehicle_name
{
public:
  explicit Init_Status_vehicle_name(::mav_msgs::msg::Status & msg)
  : msg_(msg)
  {}
  Init_Status_vehicle_type vehicle_name(::mav_msgs::msg::Status::_vehicle_name_type arg)
  {
    msg_.vehicle_name = std::move(arg);
    return Init_Status_vehicle_type(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

class Init_Status_header
{
public:
  Init_Status_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Status_vehicle_name header(::mav_msgs::msg::Status::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Status_vehicle_name(msg_);
  }

private:
  ::mav_msgs::msg::Status msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mav_msgs::msg::Status>()
{
  return mav_msgs::msg::builder::Init_Status_header();
}

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__STATUS__BUILDER_HPP_
