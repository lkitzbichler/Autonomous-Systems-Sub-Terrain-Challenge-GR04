// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_planning_msgs:msg/PolygonWithHoles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon_with_holes.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__STRUCT_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'hull'
// Member 'holes'
#include "mav_planning_msgs/msg/detail/polygon2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__msg__PolygonWithHoles __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__msg__PolygonWithHoles __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PolygonWithHoles_
{
  using Type = PolygonWithHoles_<ContainerAllocator>;

  explicit PolygonWithHoles_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : hull(_init)
  {
    (void)_init;
  }

  explicit PolygonWithHoles_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : hull(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _hull_type =
    mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>;
  _hull_type hull;
  using _holes_type =
    std::vector<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>>>;
  _holes_type holes;

  // setters for named parameter idiom
  Type & set__hull(
    const mav_planning_msgs::msg::Polygon2D_<ContainerAllocator> & _arg)
  {
    this->hull = _arg;
    return *this;
  }
  Type & set__holes(
    const std::vector<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::msg::Polygon2D_<ContainerAllocator>>> & _arg)
  {
    this->holes = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__msg__PolygonWithHoles
    std::shared_ptr<mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__msg__PolygonWithHoles
    std::shared_ptr<mav_planning_msgs::msg::PolygonWithHoles_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PolygonWithHoles_ & other) const
  {
    if (this->hull != other.hull) {
      return false;
    }
    if (this->holes != other.holes) {
      return false;
    }
    return true;
  }
  bool operator!=(const PolygonWithHoles_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PolygonWithHoles_

// alias to use template instance with default allocator
using PolygonWithHoles =
  mav_planning_msgs::msg::PolygonWithHoles_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__STRUCT_HPP_
