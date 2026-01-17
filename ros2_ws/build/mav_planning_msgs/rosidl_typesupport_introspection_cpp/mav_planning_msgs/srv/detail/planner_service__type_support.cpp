// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mav_planning_msgs:srv/PlannerService.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mav_planning_msgs/srv/detail/planner_service__functions.h"
#include "mav_planning_msgs/srv/detail/planner_service__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mav_planning_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void PlannerService_Request_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mav_planning_msgs::srv::PlannerService_Request(_init);
}

void PlannerService_Request_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mav_planning_msgs::srv::PlannerService_Request *>(message_memory);
  typed_message->~PlannerService_Request();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PlannerService_Request_message_member_array[5] = {
  {
    "start_pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Request, start_pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "start_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Request, start_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "goal_pose",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::PoseStamped>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Request, goal_pose),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "goal_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Request, goal_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "bounding_box",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Vector3>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Request, bounding_box),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PlannerService_Request_message_members = {
  "mav_planning_msgs::srv",  // message namespace
  "PlannerService_Request",  // message name
  5,  // number of fields
  sizeof(mav_planning_msgs::srv::PlannerService_Request),
  false,  // has_any_key_member_
  PlannerService_Request_message_member_array,  // message members
  PlannerService_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  PlannerService_Request_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PlannerService_Request_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PlannerService_Request_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mav_planning_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mav_planning_msgs::srv::PlannerService_Request>()
{
  return &::mav_planning_msgs::srv::rosidl_typesupport_introspection_cpp::PlannerService_Request_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mav_planning_msgs, srv, PlannerService_Request)() {
  return &::mav_planning_msgs::srv::rosidl_typesupport_introspection_cpp::PlannerService_Request_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__functions.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mav_planning_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void PlannerService_Response_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mav_planning_msgs::srv::PlannerService_Response(_init);
}

void PlannerService_Response_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mav_planning_msgs::srv::PlannerService_Response *>(message_memory);
  typed_message->~PlannerService_Response();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PlannerService_Response_message_member_array[4] = {
  {
    "success",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Response, success),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "polynomial_plan",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mav_planning_msgs::msg::PolynomialTrajectory>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Response, polynomial_plan),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "polynomial_plan_4d",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mav_planning_msgs::msg::PolynomialTrajectory4D>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Response, polynomial_plan_4d),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "sampled_plan",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<trajectory_msgs::msg::MultiDOFJointTrajectory>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Response, sampled_plan),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PlannerService_Response_message_members = {
  "mav_planning_msgs::srv",  // message namespace
  "PlannerService_Response",  // message name
  4,  // number of fields
  sizeof(mav_planning_msgs::srv::PlannerService_Response),
  false,  // has_any_key_member_
  PlannerService_Response_message_member_array,  // message members
  PlannerService_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  PlannerService_Response_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PlannerService_Response_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PlannerService_Response_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mav_planning_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mav_planning_msgs::srv::PlannerService_Response>()
{
  return &::mav_planning_msgs::srv::rosidl_typesupport_introspection_cpp::PlannerService_Response_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mav_planning_msgs, srv, PlannerService_Response)() {
  return &::mav_planning_msgs::srv::rosidl_typesupport_introspection_cpp::PlannerService_Response_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "array"
// already included above
// #include "cstddef"
// already included above
// #include "string"
// already included above
// #include "vector"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__functions.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/field_types.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mav_planning_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

void PlannerService_Event_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mav_planning_msgs::srv::PlannerService_Event(_init);
}

void PlannerService_Event_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mav_planning_msgs::srv::PlannerService_Event *>(message_memory);
  typed_message->~PlannerService_Event();
}

size_t size_function__PlannerService_Event__request(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mav_planning_msgs::srv::PlannerService_Request> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PlannerService_Event__request(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mav_planning_msgs::srv::PlannerService_Request> *>(untyped_member);
  return &member[index];
}

void * get_function__PlannerService_Event__request(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mav_planning_msgs::srv::PlannerService_Request> *>(untyped_member);
  return &member[index];
}

void fetch_function__PlannerService_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mav_planning_msgs::srv::PlannerService_Request *>(
    get_const_function__PlannerService_Event__request(untyped_member, index));
  auto & value = *reinterpret_cast<mav_planning_msgs::srv::PlannerService_Request *>(untyped_value);
  value = item;
}

void assign_function__PlannerService_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mav_planning_msgs::srv::PlannerService_Request *>(
    get_function__PlannerService_Event__request(untyped_member, index));
  const auto & value = *reinterpret_cast<const mav_planning_msgs::srv::PlannerService_Request *>(untyped_value);
  item = value;
}

void resize_function__PlannerService_Event__request(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mav_planning_msgs::srv::PlannerService_Request> *>(untyped_member);
  member->resize(size);
}

size_t size_function__PlannerService_Event__response(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<mav_planning_msgs::srv::PlannerService_Response> *>(untyped_member);
  return member->size();
}

const void * get_const_function__PlannerService_Event__response(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<mav_planning_msgs::srv::PlannerService_Response> *>(untyped_member);
  return &member[index];
}

void * get_function__PlannerService_Event__response(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<mav_planning_msgs::srv::PlannerService_Response> *>(untyped_member);
  return &member[index];
}

void fetch_function__PlannerService_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const mav_planning_msgs::srv::PlannerService_Response *>(
    get_const_function__PlannerService_Event__response(untyped_member, index));
  auto & value = *reinterpret_cast<mav_planning_msgs::srv::PlannerService_Response *>(untyped_value);
  value = item;
}

void assign_function__PlannerService_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<mav_planning_msgs::srv::PlannerService_Response *>(
    get_function__PlannerService_Event__response(untyped_member, index));
  const auto & value = *reinterpret_cast<const mav_planning_msgs::srv::PlannerService_Response *>(untyped_value);
  item = value;
}

void resize_function__PlannerService_Event__response(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<mav_planning_msgs::srv::PlannerService_Response> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PlannerService_Event_message_member_array[3] = {
  {
    "info",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<service_msgs::msg::ServiceEventInfo>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Event, info),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "request",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mav_planning_msgs::srv::PlannerService_Request>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Event, request),  // bytes offset in struct
    nullptr,  // default value
    size_function__PlannerService_Event__request,  // size() function pointer
    get_const_function__PlannerService_Event__request,  // get_const(index) function pointer
    get_function__PlannerService_Event__request,  // get(index) function pointer
    fetch_function__PlannerService_Event__request,  // fetch(index, &value) function pointer
    assign_function__PlannerService_Event__request,  // assign(index, value) function pointer
    resize_function__PlannerService_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mav_planning_msgs::srv::PlannerService_Response>(),  // members of sub message
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(mav_planning_msgs::srv::PlannerService_Event, response),  // bytes offset in struct
    nullptr,  // default value
    size_function__PlannerService_Event__response,  // size() function pointer
    get_const_function__PlannerService_Event__response,  // get_const(index) function pointer
    get_function__PlannerService_Event__response,  // get(index) function pointer
    fetch_function__PlannerService_Event__response,  // fetch(index, &value) function pointer
    assign_function__PlannerService_Event__response,  // assign(index, value) function pointer
    resize_function__PlannerService_Event__response  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PlannerService_Event_message_members = {
  "mav_planning_msgs::srv",  // message namespace
  "PlannerService_Event",  // message name
  3,  // number of fields
  sizeof(mav_planning_msgs::srv::PlannerService_Event),
  false,  // has_any_key_member_
  PlannerService_Event_message_member_array,  // message members
  PlannerService_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  PlannerService_Event_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PlannerService_Event_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PlannerService_Event_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mav_planning_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mav_planning_msgs::srv::PlannerService_Event>()
{
  return &::mav_planning_msgs::srv::rosidl_typesupport_introspection_cpp::PlannerService_Event_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mav_planning_msgs, srv, PlannerService_Event)() {
  return &::mav_planning_msgs::srv::rosidl_typesupport_introspection_cpp::PlannerService_Event_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_interface/macros.h"
// already included above
// #include "rosidl_typesupport_introspection_cpp/visibility_control.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__functions.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__struct.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

namespace mav_planning_msgs
{

namespace srv
{

namespace rosidl_typesupport_introspection_cpp
{

// this is intentionally not const to allow initialization later to prevent an initialization race
static ::rosidl_typesupport_introspection_cpp::ServiceMembers PlannerService_service_members = {
  "mav_planning_msgs::srv",  // service namespace
  "PlannerService",  // service name
  // the following fields are initialized below on first access
  // see get_service_type_support_handle<mav_planning_msgs::srv::PlannerService>()
  nullptr,  // request message
  nullptr,  // response message
  nullptr,  // event message
};

static const rosidl_service_type_support_t PlannerService_service_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PlannerService_service_members,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mav_planning_msgs::srv::PlannerService_Request>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mav_planning_msgs::srv::PlannerService_Response>(),
  ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mav_planning_msgs::srv::PlannerService_Event>(),
  &::rosidl_typesupport_cpp::service_create_event_message<mav_planning_msgs::srv::PlannerService>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<mav_planning_msgs::srv::PlannerService>,
  &mav_planning_msgs__srv__PlannerService__get_type_hash,
  &mav_planning_msgs__srv__PlannerService__get_type_description,
  &mav_planning_msgs__srv__PlannerService__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace srv

}  // namespace mav_planning_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<mav_planning_msgs::srv::PlannerService>()
{
  // get a handle to the value to be returned
  auto service_type_support =
    &::mav_planning_msgs::srv::rosidl_typesupport_introspection_cpp::PlannerService_service_type_support_handle;
  // get a non-const and properly typed version of the data void *
  auto service_members = const_cast<::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      service_type_support->data));
  // make sure all of the service_members are initialized
  // if they are not, initialize them
  if (
    service_members->request_members_ == nullptr ||
    service_members->response_members_ == nullptr ||
    service_members->event_members_ == nullptr)
  {
    // initialize the request_members_ with the static function from the external library
    service_members->request_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::mav_planning_msgs::srv::PlannerService_Request
      >()->data
      );
    // initialize the response_members_ with the static function from the external library
    service_members->response_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::mav_planning_msgs::srv::PlannerService_Response
      >()->data
      );
    // initialize the event_members_ with the static function from the external library
    service_members->event_members_ = static_cast<
      const ::rosidl_typesupport_introspection_cpp::MessageMembers *
      >(
      ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<
        ::mav_planning_msgs::srv::PlannerService_Event
      >()->data
      );
  }
  // finally return the properly initialized service_type_support handle
  return service_type_support;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mav_planning_msgs, srv, PlannerService)() {
  return ::rosidl_typesupport_introspection_cpp::get_service_type_support_handle<mav_planning_msgs::srv::PlannerService>();
}

#ifdef __cplusplus
}
#endif
