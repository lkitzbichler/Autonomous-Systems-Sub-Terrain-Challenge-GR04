// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from mav_planning_msgs:srv/ChangeNameService.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/srv/change_name_service.hpp"


#ifndef MAV_PLANNING_MSGS__SRV__DETAIL__CHANGE_NAME_SERVICE__STRUCT_HPP_
#define MAV_PLANNING_MSGS__SRV__DETAIL__CHANGE_NAME_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Request __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Request __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ChangeNameService_Request_
{
  using Type = ChangeNameService_Request_<ContainerAllocator>;

  explicit ChangeNameService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  explicit ChangeNameService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Request
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Request
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChangeNameService_Request_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChangeNameService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChangeNameService_Request_

// alias to use template instance with default allocator
using ChangeNameService_Request =
  mav_planning_msgs::srv::ChangeNameService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mav_planning_msgs


#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Response __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Response __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ChangeNameService_Response_
{
  using Type = ChangeNameService_Response_<ContainerAllocator>;

  explicit ChangeNameService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ChangeNameService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Response
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Response
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChangeNameService_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChangeNameService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChangeNameService_Response_

// alias to use template instance with default allocator
using ChangeNameService_Response =
  mav_planning_msgs::srv::ChangeNameService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mav_planning_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Event __attribute__((deprecated))
#else
# define DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Event __declspec(deprecated)
#endif

namespace mav_planning_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ChangeNameService_Event_
{
  using Type = ChangeNameService_Event_<ContainerAllocator>;

  explicit ChangeNameService_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit ChangeNameService_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::srv::ChangeNameService_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<mav_planning_msgs::srv::ChangeNameService_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Event
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__mav_planning_msgs__srv__ChangeNameService_Event
    std::shared_ptr<mav_planning_msgs::srv::ChangeNameService_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChangeNameService_Event_ & other) const
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
  bool operator!=(const ChangeNameService_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChangeNameService_Event_

// alias to use template instance with default allocator
using ChangeNameService_Event =
  mav_planning_msgs::srv::ChangeNameService_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace mav_planning_msgs

namespace mav_planning_msgs
{

namespace srv
{

struct ChangeNameService
{
  using Request = mav_planning_msgs::srv::ChangeNameService_Request;
  using Response = mav_planning_msgs::srv::ChangeNameService_Response;
  using Event = mav_planning_msgs::srv::ChangeNameService_Event;
};

}  // namespace srv

}  // namespace mav_planning_msgs

#endif  // MAV_PLANNING_MSGS__SRV__DETAIL__CHANGE_NAME_SERVICE__STRUCT_HPP_
