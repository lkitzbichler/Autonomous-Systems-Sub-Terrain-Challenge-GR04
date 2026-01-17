// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_msgs:msg/AttitudeThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/attitude_thrust.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__STRUCT_HPP_
#define MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__STRUCT_HPP_

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
// Member 'attitude'
#include "geometry_msgs/msg/detail/quaternion__struct.hpp"
// Member 'thrust'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_msgs__msg__AttitudeThrust __attribute__((deprecated))
#else
# define DEPRECATED__mav_msgs__msg__AttitudeThrust __declspec(deprecated)
#endif

namespace mav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AttitudeThrust_
{
  using Type = AttitudeThrust_<ContainerAllocator>;

  explicit AttitudeThrust_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    attitude(_init),
    thrust(_init)
  {
    (void)_init;
  }

  explicit AttitudeThrust_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    attitude(_alloc, _init),
    thrust(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _attitude_type =
    geometry_msgs::msg::Quaternion_<ContainerAllocator>;
  _attitude_type attitude;
  using _thrust_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _thrust_type thrust;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__attitude(
    const geometry_msgs::msg::Quaternion_<ContainerAllocator> & _arg)
  {
    this->attitude = _arg;
    return *this;
  }
  Type & set__thrust(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->thrust = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_msgs::msg::AttitudeThrust_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_msgs::msg::AttitudeThrust_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_msgs::msg::AttitudeThrust_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_msgs::msg::AttitudeThrust_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::AttitudeThrust_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::AttitudeThrust_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::AttitudeThrust_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::AttitudeThrust_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_msgs::msg::AttitudeThrust_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_msgs::msg::AttitudeThrust_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_msgs__msg__AttitudeThrust
    std::shared_ptr<mav_msgs::msg::AttitudeThrust_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_msgs__msg__AttitudeThrust
    std::shared_ptr<mav_msgs::msg::AttitudeThrust_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AttitudeThrust_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->attitude != other.attitude) {
      return false;
    }
    if (this->thrust != other.thrust) {
      return false;
    }
    return true;
  }
  bool operator!=(const AttitudeThrust_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AttitudeThrust_

// alias to use template instance with default allocator
using AttitudeThrust =
  mav_msgs::msg::AttitudeThrust_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__STRUCT_HPP_
