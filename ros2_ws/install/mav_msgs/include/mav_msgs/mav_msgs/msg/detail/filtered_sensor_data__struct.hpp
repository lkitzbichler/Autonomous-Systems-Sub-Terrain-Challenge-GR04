// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_msgs:msg/FilteredSensorData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/filtered_sensor_data.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__STRUCT_HPP_
#define MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__STRUCT_HPP_

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
// Member 'accelerometer'
// Member 'gyroscope'
// Member 'magnetometer'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_msgs__msg__FilteredSensorData __attribute__((deprecated))
#else
# define DEPRECATED__mav_msgs__msg__FilteredSensorData __declspec(deprecated)
#endif

namespace mav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FilteredSensorData_
{
  using Type = FilteredSensorData_<ContainerAllocator>;

  explicit FilteredSensorData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    accelerometer(_init),
    gyroscope(_init),
    magnetometer(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->barometer = 0.0;
    }
  }

  explicit FilteredSensorData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    accelerometer(_alloc, _init),
    gyroscope(_alloc, _init),
    magnetometer(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->barometer = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _accelerometer_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _accelerometer_type accelerometer;
  using _gyroscope_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _gyroscope_type gyroscope;
  using _magnetometer_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _magnetometer_type magnetometer;
  using _barometer_type =
    double;
  _barometer_type barometer;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__accelerometer(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->accelerometer = _arg;
    return *this;
  }
  Type & set__gyroscope(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->gyroscope = _arg;
    return *this;
  }
  Type & set__magnetometer(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->magnetometer = _arg;
    return *this;
  }
  Type & set__barometer(
    const double & _arg)
  {
    this->barometer = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_msgs::msg::FilteredSensorData_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_msgs::msg::FilteredSensorData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_msgs::msg::FilteredSensorData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_msgs::msg::FilteredSensorData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::FilteredSensorData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::FilteredSensorData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::FilteredSensorData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::FilteredSensorData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_msgs::msg::FilteredSensorData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_msgs::msg::FilteredSensorData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_msgs__msg__FilteredSensorData
    std::shared_ptr<mav_msgs::msg::FilteredSensorData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_msgs__msg__FilteredSensorData
    std::shared_ptr<mav_msgs::msg::FilteredSensorData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FilteredSensorData_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->accelerometer != other.accelerometer) {
      return false;
    }
    if (this->gyroscope != other.gyroscope) {
      return false;
    }
    if (this->magnetometer != other.magnetometer) {
      return false;
    }
    if (this->barometer != other.barometer) {
      return false;
    }
    return true;
  }
  bool operator!=(const FilteredSensorData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FilteredSensorData_

// alias to use template instance with default allocator
using FilteredSensorData =
  mav_msgs::msg::FilteredSensorData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__STRUCT_HPP_
