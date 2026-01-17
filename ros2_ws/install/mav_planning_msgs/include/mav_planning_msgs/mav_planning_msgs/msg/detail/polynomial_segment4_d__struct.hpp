// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_planning_msgs:msg/PolynomialSegment4D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polynomial_segment4_d.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT4_D__STRUCT_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT4_D__STRUCT_HPP_

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
// Member 'segment_time'
#include "builtin_interfaces/msg/detail/duration__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__msg__PolynomialSegment4D __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__msg__PolynomialSegment4D __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PolynomialSegment4D_
{
  using Type = PolynomialSegment4D_<ContainerAllocator>;

  explicit PolynomialSegment4D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    segment_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_coeffs = 0l;
    }
  }

  explicit PolynomialSegment4D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    segment_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_coeffs = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _num_coeffs_type =
    int32_t;
  _num_coeffs_type num_coeffs;
  using _segment_time_type =
    builtin_interfaces::msg::Duration_<ContainerAllocator>;
  _segment_time_type segment_time;
  using _x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x_type x;
  using _y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_type y;
  using _z_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _z_type z;
  using _yaw_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _yaw_type yaw;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__num_coeffs(
    const int32_t & _arg)
  {
    this->num_coeffs = _arg;
    return *this;
  }
  Type & set__segment_time(
    const builtin_interfaces::msg::Duration_<ContainerAllocator> & _arg)
  {
    this->segment_time = _arg;
    return *this;
  }
  Type & set__x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__yaw(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__msg__PolynomialSegment4D
    std::shared_ptr<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__msg__PolynomialSegment4D
    std::shared_ptr<mav_planning_msgs::msg::PolynomialSegment4D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PolynomialSegment4D_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->num_coeffs != other.num_coeffs) {
      return false;
    }
    if (this->segment_time != other.segment_time) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const PolynomialSegment4D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PolynomialSegment4D_

// alias to use template instance with default allocator
using PolynomialSegment4D =
  mav_planning_msgs::msg::PolynomialSegment4D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT4_D__STRUCT_HPP_
