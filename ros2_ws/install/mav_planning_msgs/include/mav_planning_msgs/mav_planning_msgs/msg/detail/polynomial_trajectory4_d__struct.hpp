// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_planning_msgs:msg/PolynomialTrajectory4D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polynomial_trajectory4_d.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY4_D__STRUCT_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY4_D__STRUCT_HPP_

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
// Member 'segments'
#include "mav_planning_msgs/msg/detail/polynomial_segment4_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__msg__PolynomialTrajectory4D __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__msg__PolynomialTrajectory4D __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PolynomialTrajectory4D_
{
  using Type = PolynomialTrajectory4D_<ContainerAllocator>;

  explicit PolynomialTrajectory4D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit PolynomialTrajectory4D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _segments_type =
    std::vector<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>>>;
  _segments_type segments;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__segments(
    const std::vector<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>>> & _arg)
  {
    this->segments = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__msg__PolynomialTrajectory4D
    std::shared_ptr<mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__msg__PolynomialTrajectory4D
    std::shared_ptr<mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PolynomialTrajectory4D_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->segments != other.segments) {
      return false;
    }
    return true;
  }
  bool operator!=(const PolynomialTrajectory4D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PolynomialTrajectory4D_

// alias to use template instance with default allocator
using PolynomialTrajectory4D =
  mav_planning_msgs::msg::PolynomialTrajectory4D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY4_D__STRUCT_HPP_
