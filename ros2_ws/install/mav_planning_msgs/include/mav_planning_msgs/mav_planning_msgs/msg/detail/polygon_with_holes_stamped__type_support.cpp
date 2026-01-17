// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from mav_planning_msgs:msg/PolygonWithHolesStamped.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "mav_planning_msgs/msg/detail/polygon_with_holes_stamped__functions.h"
#include "mav_planning_msgs/msg/detail/polygon_with_holes_stamped__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace mav_planning_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void PolygonWithHolesStamped_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) mav_planning_msgs::msg::PolygonWithHolesStamped(_init);
}

void PolygonWithHolesStamped_fini_function(void * message_memory)
{
  auto typed_message = static_cast<mav_planning_msgs::msg::PolygonWithHolesStamped *>(message_memory);
  typed_message->~PolygonWithHolesStamped();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember PolygonWithHolesStamped_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolygonWithHolesStamped, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "altitude",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolygonWithHolesStamped, altitude),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "polygon",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<mav_planning_msgs::msg::PolygonWithHoles>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs::msg::PolygonWithHolesStamped, polygon),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers PolygonWithHolesStamped_message_members = {
  "mav_planning_msgs::msg",  // message namespace
  "PolygonWithHolesStamped",  // message name
  3,  // number of fields
  sizeof(mav_planning_msgs::msg::PolygonWithHolesStamped),
  false,  // has_any_key_member_
  PolygonWithHolesStamped_message_member_array,  // message members
  PolygonWithHolesStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  PolygonWithHolesStamped_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t PolygonWithHolesStamped_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &PolygonWithHolesStamped_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_hash,
  &mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_description,
  &mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace mav_planning_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<mav_planning_msgs::msg::PolygonWithHolesStamped>()
{
  return &::mav_planning_msgs::msg::rosidl_typesupport_introspection_cpp::PolygonWithHolesStamped_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, mav_planning_msgs, msg, PolygonWithHolesStamped)() {
  return &::mav_planning_msgs::msg::rosidl_typesupport_introspection_cpp::PolygonWithHolesStamped_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
