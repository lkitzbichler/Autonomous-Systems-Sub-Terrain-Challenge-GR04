// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_planning_msgs:msg/Polygon2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon2_d.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__STRUCT_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'points'
#include "mav_planning_msgs/msg/detail/point2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__msg__Polygon2D __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__msg__Polygon2D __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Polygon2D_
{
  using Type = Polygon2D_<ContainerAllocator>;

  explicit Polygon2D_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Polygon2D_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _points_type =
    std::vector<mav_planning_msgs::msg::Point2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::msg::Point2D_<ContainerAllocator>>>;
  _points_type points;

  // setters for named parameter idiom
  Type & set__points(
    const std::vector<mav_planning_msgs::msg::Point2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::msg::Point2D_<ContainerAllocator>>> & _arg)
  {
    this->points = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::msg::Polygon2D_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::msg::Polygon2D_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__msg__Polygon2D
    std::shared_ptr<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__msg__Polygon2D
    std::shared_ptr<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Polygon2D_ & other) const
  {
    if (this->points != other.points) {
      return false;
    }
    return true;
  }
  bool operator!=(const Polygon2D_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Polygon2D_

// alias to use template instance with default allocator
using Polygon2D =
  mav_planning_msgs::msg::Polygon2D_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__STRUCT_HPP_
