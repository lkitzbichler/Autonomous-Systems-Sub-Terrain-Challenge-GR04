// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_msgs:msg/Status.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/status.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
#define MAV_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_msgs__msg__Status __attribute__((deprecated))
#else
# define DEPRECATED__mav_msgs__msg__Status __declspec(deprecated)
#endif

namespace mav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Status_
{
  using Type = Status_<ContainerAllocator>;

  explicit Status_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_name = "";
      this->vehicle_type = "";
      this->battery_voltage = 0.0f;
      this->rc_command_mode = "";
      this->command_interface_enabled = false;
      this->flight_time = 0.0f;
      this->system_uptime = 0.0f;
      this->cpu_load = 0.0f;
      this->motor_status = "";
      this->in_air = false;
      this->gps_status = "";
      this->gps_num_satellites = 0l;
    }
  }

  explicit Status_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    vehicle_name(_alloc),
    vehicle_type(_alloc),
    rc_command_mode(_alloc),
    motor_status(_alloc),
    gps_status(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vehicle_name = "";
      this->vehicle_type = "";
      this->battery_voltage = 0.0f;
      this->rc_command_mode = "";
      this->command_interface_enabled = false;
      this->flight_time = 0.0f;
      this->system_uptime = 0.0f;
      this->cpu_load = 0.0f;
      this->motor_status = "";
      this->in_air = false;
      this->gps_status = "";
      this->gps_num_satellites = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _vehicle_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _vehicle_name_type vehicle_name;
  using _vehicle_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _vehicle_type_type vehicle_type;
  using _battery_voltage_type =
    float;
  _battery_voltage_type battery_voltage;
  using _rc_command_mode_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _rc_command_mode_type rc_command_mode;
  using _command_interface_enabled_type =
    bool;
  _command_interface_enabled_type command_interface_enabled;
  using _flight_time_type =
    float;
  _flight_time_type flight_time;
  using _system_uptime_type =
    float;
  _system_uptime_type system_uptime;
  using _cpu_load_type =
    float;
  _cpu_load_type cpu_load;
  using _motor_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _motor_status_type motor_status;
  using _in_air_type =
    bool;
  _in_air_type in_air;
  using _gps_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _gps_status_type gps_status;
  using _gps_num_satellites_type =
    int32_t;
  _gps_num_satellites_type gps_num_satellites;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__vehicle_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->vehicle_name = _arg;
    return *this;
  }
  Type & set__vehicle_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->vehicle_type = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const float & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__rc_command_mode(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->rc_command_mode = _arg;
    return *this;
  }
  Type & set__command_interface_enabled(
    const bool & _arg)
  {
    this->command_interface_enabled = _arg;
    return *this;
  }
  Type & set__flight_time(
    const float & _arg)
  {
    this->flight_time = _arg;
    return *this;
  }
  Type & set__system_uptime(
    const float & _arg)
  {
    this->system_uptime = _arg;
    return *this;
  }
  Type & set__cpu_load(
    const float & _arg)
  {
    this->cpu_load = _arg;
    return *this;
  }
  Type & set__motor_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->motor_status = _arg;
    return *this;
  }
  Type & set__in_air(
    const bool & _arg)
  {
    this->in_air = _arg;
    return *this;
  }
  Type & set__gps_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->gps_status = _arg;
    return *this;
  }
  Type & set__gps_num_satellites(
    const int32_t & _arg)
  {
    this->gps_num_satellites = _arg;
    return *this;
  }

  // constant declarations
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> RC_COMMAND_ATTITUDE;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> RC_COMMAND_ATTITUDE_HEIGHT;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> RC_COMMAND_POSITION;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MOTOR_STATUS_RUNNING;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MOTOR_STATUS_STOPPED;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MOTOR_STATUS_STARTING;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> MOTOR_STATUS_STOPPING;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> GPS_STATUS_LOCK;
  static const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> GPS_STATUS_NO_LOCK;

  // pointer types
  using RawPtr =
    mav_msgs::msg::Status_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_msgs::msg::Status_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_msgs::msg::Status_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_msgs::msg::Status_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::Status_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::Status_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::Status_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::Status_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_msgs::msg::Status_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_msgs::msg::Status_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_msgs__msg__Status
    std::shared_ptr<mav_msgs::msg::Status_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_msgs__msg__Status
    std::shared_ptr<mav_msgs::msg::Status_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Status_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->vehicle_name != other.vehicle_name) {
      return false;
    }
    if (this->vehicle_type != other.vehicle_type) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->rc_command_mode != other.rc_command_mode) {
      return false;
    }
    if (this->command_interface_enabled != other.command_interface_enabled) {
      return false;
    }
    if (this->flight_time != other.flight_time) {
      return false;
    }
    if (this->system_uptime != other.system_uptime) {
      return false;
    }
    if (this->cpu_load != other.cpu_load) {
      return false;
    }
    if (this->motor_status != other.motor_status) {
      return false;
    }
    if (this->in_air != other.in_air) {
      return false;
    }
    if (this->gps_status != other.gps_status) {
      return false;
    }
    if (this->gps_num_satellites != other.gps_num_satellites) {
      return false;
    }
    return true;
  }
  bool operator!=(const Status_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Status_

// alias to use template instance with default allocator
using Status =
  mav_msgs::msg::Status_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
Status_<ContainerAllocator>::RC_COMMAND_ATTITUDE = "attitude_thrust";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
Status_<ContainerAllocator>::RC_COMMAND_ATTITUDE_HEIGHT = "attitude_height";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
Status_<ContainerAllocator>::RC_COMMAND_POSITION = "position";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
Status_<ContainerAllocator>::MOTOR_STATUS_RUNNING = "running";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
Status_<ContainerAllocator>::MOTOR_STATUS_STOPPED = "stopped";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
Status_<ContainerAllocator>::MOTOR_STATUS_STARTING = "starting";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
Status_<ContainerAllocator>::MOTOR_STATUS_STOPPING = "stopping";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
Status_<ContainerAllocator>::GPS_STATUS_LOCK = "lock";
template<typename ContainerAllocator>
const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>
Status_<ContainerAllocator>::GPS_STATUS_NO_LOCK = "no_lock";

}  // namespace msg

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__STATUS__STRUCT_HPP_
