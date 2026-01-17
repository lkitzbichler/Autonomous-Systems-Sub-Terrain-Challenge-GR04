// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_msgs:msg/Actuators.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/actuators.hpp"


#ifndef MAV_MSGS__MSG__DETAIL__ACTUATORS__STRUCT_HPP_
#define MAV_MSGS__MSG__DETAIL__ACTUATORS__STRUCT_HPP_

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
# define DEPRECATED__mav_msgs__msg__Actuators __attribute__((deprecated))
#else
# define DEPRECATED__mav_msgs__msg__Actuators __declspec(deprecated)
#endif

namespace mav_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Actuators_
{
  using Type = Actuators_<ContainerAllocator>;

  explicit Actuators_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Actuators_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _angles_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _angles_type angles;
  using _angular_velocities_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _angular_velocities_type angular_velocities;
  using _normalized_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _normalized_type normalized;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__angles(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->angles = _arg;
    return *this;
  }
  Type & set__angular_velocities(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->angular_velocities = _arg;
    return *this;
  }
  Type & set__normalized(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->normalized = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_msgs::msg::Actuators_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_msgs::msg::Actuators_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_msgs::msg::Actuators_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_msgs::msg::Actuators_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::Actuators_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::Actuators_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_msgs::msg::Actuators_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_msgs::msg::Actuators_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_msgs::msg::Actuators_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_msgs::msg::Actuators_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_msgs__msg__Actuators
    std::shared_ptr<mav_msgs::msg::Actuators_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_msgs__msg__Actuators
    std::shared_ptr<mav_msgs::msg::Actuators_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Actuators_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->angles != other.angles) {
      return false;
    }
    if (this->angular_velocities != other.angular_velocities) {
      return false;
    }
    if (this->normalized != other.normalized) {
      return false;
    }
    return true;
  }
  bool operator!=(const Actuators_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Actuators_

// alias to use template instance with default allocator
using Actuators =
  mav_msgs::msg::Actuators_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mav_msgs

#endif  // MAV_MSGS__MSG__DETAIL__ACTUATORS__STRUCT_HPP_
