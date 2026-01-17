// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_planning_msgs:srv/PlannerService.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/srv/planner_service.hpp"


#ifndef MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__STRUCT_HPP_
#define MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'start_pose'
// Member 'goal_pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"
// Member 'start_velocity'
// Member 'goal_velocity'
// Member 'bounding_box'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__srv__PlannerService_Request __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__srv__PlannerService_Request __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlannerService_Request_
{
  using Type = PlannerService_Request_<ContainerAllocator>;

  explicit PlannerService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_pose(_init),
    start_velocity(_init),
    goal_pose(_init),
    goal_velocity(_init),
    bounding_box(_init)
  {
    (void)_init;
  }

  explicit PlannerService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : start_pose(_alloc, _init),
    start_velocity(_alloc, _init),
    goal_pose(_alloc, _init),
    goal_velocity(_alloc, _init),
    bounding_box(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _start_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _start_pose_type start_pose;
  using _start_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _start_velocity_type start_velocity;
  using _goal_pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _goal_pose_type goal_pose;
  using _goal_velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _goal_velocity_type goal_velocity;
  using _bounding_box_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _bounding_box_type bounding_box;

  // setters for named parameter idiom
  Type & set__start_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->start_pose = _arg;
    return *this;
  }
  Type & set__start_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->start_velocity = _arg;
    return *this;
  }
  Type & set__goal_pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->goal_pose = _arg;
    return *this;
  }
  Type & set__goal_velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->goal_velocity = _arg;
    return *this;
  }
  Type & set__bounding_box(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->bounding_box = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__srv__PlannerService_Request
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__srv__PlannerService_Request
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlannerService_Request_ & other) const
  {
    if (this->start_pose != other.start_pose) {
      return false;
    }
    if (this->start_velocity != other.start_velocity) {
      return false;
    }
    if (this->goal_pose != other.goal_pose) {
      return false;
    }
    if (this->goal_velocity != other.goal_velocity) {
      return false;
    }
    if (this->bounding_box != other.bounding_box) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlannerService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlannerService_Request_

// alias to use template instance with default allocator
using PlannerService_Request =
  mav_planning_msgs::srv::PlannerService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mav_planning_msgs


// Include directives for member types
// Member 'polynomial_plan'
#include "mav_planning_msgs/msg/detail/polynomial_trajectory__struct.hpp"
// Member 'polynomial_plan_4d'
#include "mav_planning_msgs/msg/detail/polynomial_trajectory4_d__struct.hpp"
// Member 'sampled_plan'
#include "trajectory_msgs/msg/detail/multi_dof_joint_trajectory__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__srv__PlannerService_Response __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__srv__PlannerService_Response __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlannerService_Response_
{
  using Type = PlannerService_Response_<ContainerAllocator>;

  explicit PlannerService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : polynomial_plan(_init),
    polynomial_plan_4d(_init),
    sampled_plan(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit PlannerService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : polynomial_plan(_alloc, _init),
    polynomial_plan_4d(_alloc, _init),
    sampled_plan(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _polynomial_plan_type =
    mav_planning_msgs::msg::PolynomialTrajectory_<ContainerAllocator>;
  _polynomial_plan_type polynomial_plan;
  using _polynomial_plan_4d_type =
    mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator>;
  _polynomial_plan_4d_type polynomial_plan_4d;
  using _sampled_plan_type =
    trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator>;
  _sampled_plan_type sampled_plan;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__polynomial_plan(
    const mav_planning_msgs::msg::PolynomialTrajectory_<ContainerAllocator> & _arg)
  {
    this->polynomial_plan = _arg;
    return *this;
  }
  Type & set__polynomial_plan_4d(
    const mav_planning_msgs::msg::PolynomialTrajectory4D_<ContainerAllocator> & _arg)
  {
    this->polynomial_plan_4d = _arg;
    return *this;
  }
  Type & set__sampled_plan(
    const trajectory_msgs::msg::MultiDOFJointTrajectory_<ContainerAllocator> & _arg)
  {
    this->sampled_plan = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__srv__PlannerService_Response
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__srv__PlannerService_Response
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlannerService_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->polynomial_plan != other.polynomial_plan) {
      return false;
    }
    if (this->polynomial_plan_4d != other.polynomial_plan_4d) {
      return false;
    }
    if (this->sampled_plan != other.sampled_plan) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlannerService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlannerService_Response_

// alias to use template instance with default allocator
using PlannerService_Response =
  mav_planning_msgs::srv::PlannerService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mav_planning_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__srv__PlannerService_Event __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__srv__PlannerService_Event __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PlannerService_Event_
{
  using Type = PlannerService_Event_<ContainerAllocator>;

  explicit PlannerService_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit PlannerService_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::srv::PlannerService_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::srv::PlannerService_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__srv__PlannerService_Event
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__srv__PlannerService_Event
    std::shared_ptr<mav_planning_msgs::srv::PlannerService_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlannerService_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlannerService_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlannerService_Event_

// alias to use template instance with default allocator
using PlannerService_Event =
  mav_planning_msgs::srv::PlannerService_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mav_planning_msgs

namespace mav_planning_msgs
{

namespace srv
{

struct PlannerService
{
  using Request = mav_planning_msgs::srv::PlannerService_Request;
  using Response = mav_planning_msgs::srv::PlannerService_Response;
  using Event = mav_planning_msgs::srv::PlannerService_Event;
};

}  // namespace srv

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__SRV__DETAIL__PLANNER_SERVICE__STRUCT_HPP_
