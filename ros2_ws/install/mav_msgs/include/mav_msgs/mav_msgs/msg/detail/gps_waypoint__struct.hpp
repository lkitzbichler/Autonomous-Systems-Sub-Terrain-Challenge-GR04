// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_msgs:msg/GpsWaypoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/gps_waypoint.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__STRUCT_HPP_
#define MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__STRUCT_HPP_

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
# define DEPRECATED__mav_msgs__msg__GpsWaypoint __attribute__((deprecated))
#else
# define DEPRECATED__mav_msgs__msg__GpsWaypoint __declspec(deprecated)
#endif

namespace mav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GpsWaypoint_
{
  using Type = GpsWaypoint_<ContainerAllocator>;

  explicit GpsWaypoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->heading = 0.0;
      this->max_speed = 0.0;
      this->max_acc = 0.0;
    }
  }

  explicit GpsWaypoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->latitude = 0.0;
      this->longitude = 0.0;
      this->altitude = 0.0;
      this->heading = 0.0;
      this->max_speed = 0.0;
      this->max_acc = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _latitude_type =
    double;
  _latitude_type latitude;
  using _longitude_type =
    double;
  _longitude_type longitude;
  using _altitude_type =
    double;
  _altitude_type altitude;
  using _heading_type =
    double;
  _heading_type heading;
  using _max_speed_type =
    double;
  _max_speed_type max_speed;
  using _max_acc_type =
    double;
  _max_acc_type max_acc;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__latitude(
    const double & _arg)
  {
    this->latitude = _arg;
    return *this;
  }
  Type & set__longitude(
    const double & _arg)
  {
    this->longitude = _arg;
    return *this;
  }
  Type & set__altitude(
    const double & _arg)
  {
    this->altitude = _arg;
    return *this;
  }
  Type & set__heading(
    const double & _arg)
  {
    this->heading = _arg;
    return *this;
  }
  Type & set__max_speed(
    const double & _arg)
  {
    this->max_speed = _arg;
    return *this;
  }
  Type & set__max_acc(
    const double & _arg)
  {
    this->max_acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_msgs::msg::GpsWaypoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_msgs::msg::GpsWaypoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_msgs::msg::GpsWaypoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_msgs::msg::GpsWaypoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::GpsWaypoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::GpsWaypoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::GpsWaypoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::GpsWaypoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_msgs::msg::GpsWaypoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_msgs::msg::GpsWaypoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_msgs__msg__GpsWaypoint
    std::shared_ptr<mav_msgs::msg::GpsWaypoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_msgs__msg__GpsWaypoint
    std::shared_ptr<mav_msgs::msg::GpsWaypoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GpsWaypoint_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->latitude != other.latitude) {
      return false;
    }
    if (this->longitude != other.longitude) {
      return false;
    }
    if (this->altitude != other.altitude) {
      return false;
    }
    if (this->heading != other.heading) {
      return false;
    }
    if (this->max_speed != other.max_speed) {
      return false;
    }
    if (this->max_acc != other.max_acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const GpsWaypoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GpsWaypoint_

// alias to use template instance with default allocator
using GpsWaypoint =
  mav_msgs::msg::GpsWaypoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__STRUCT_HPP_
