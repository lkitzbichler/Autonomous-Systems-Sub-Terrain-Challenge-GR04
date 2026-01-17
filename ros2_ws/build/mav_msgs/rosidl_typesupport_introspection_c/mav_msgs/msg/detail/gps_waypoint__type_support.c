// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mav_msgs:msg/GpsWaypoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mav_msgs/msg/detail/gps_waypoint__rosidl_typesupport_introspection_c.h"
#include "mav_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mav_msgs/msg/detail/gps_waypoint__functions.h"
#include "mav_msgs/msg/detail/gps_waypoint__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mav_msgs__msg__GpsWaypoint__init(message_memory);
}

void mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_fini_function(void * message_memory)
{
  mav_msgs__msg__GpsWaypoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__GpsWaypoint, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__GpsWaypoint, latitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__GpsWaypoint, longitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__GpsWaypoint, altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "heading",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__GpsWaypoint, heading),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__GpsWaypoint, max_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__GpsWaypoint, max_acc),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_message_members = {
  "mav_msgs__msg",  // message namespace
  "GpsWaypoint",  // message name
  7,  // number of fields
  sizeof(mav_msgs__msg__GpsWaypoint),
  false,  // has_any_key_member_
  mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_message_member_array,  // message members
  mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_init_function,  // function to initialize message memory (memory has to be allocated)
  mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_message_type_support_handle = {
  0,
  &mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_message_members,
  get_message_typesupport_handle_function,
  &mav_msgs__msg__GpsWaypoint__get_type_hash,
  &mav_msgs__msg__GpsWaypoint__get_type_description,
  &mav_msgs__msg__GpsWaypoint__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mav_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_msgs, msg, GpsWaypoint)() {
  mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_message_type_support_handle.typesupport_identifier) {
    mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mav_msgs__msg__GpsWaypoint__rosidl_typesupport_introspection_c__GpsWaypoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
