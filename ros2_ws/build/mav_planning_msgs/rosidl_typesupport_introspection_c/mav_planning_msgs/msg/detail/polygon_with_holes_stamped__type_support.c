// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mav_planning_msgs:msg/PolygonWithHolesStamped.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mav_planning_msgs/msg/detail/polygon_with_holes_stamped__rosidl_typesupport_introspection_c.h"
#include "mav_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mav_planning_msgs/msg/detail/polygon_with_holes_stamped__functions.h"
#include "mav_planning_msgs/msg/detail/polygon_with_holes_stamped__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `polygon`
#include "mav_planning_msgs/msg/polygon_with_holes.h"
// Member `polygon`
#include "mav_planning_msgs/msg/detail/polygon_with_holes__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mav_planning_msgs__msg__PolygonWithHolesStamped__init(message_memory);
}

void mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_fini_function(void * message_memory)
{
  mav_planning_msgs__msg__PolygonWithHolesStamped__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolygonWithHolesStamped, header),  // bytes offset in struct
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
    offsetof(mav_planning_msgs__msg__PolygonWithHolesStamped, altitude),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "polygon",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolygonWithHolesStamped, polygon),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_members = {
  "mav_planning_msgs__msg",  // message namespace
  "PolygonWithHolesStamped",  // message name
  3,  // number of fields
  sizeof(mav_planning_msgs__msg__PolygonWithHolesStamped),
  false,  // has_any_key_member_
  mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_member_array,  // message members
  mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_init_function,  // function to initialize message memory (memory has to be allocated)
  mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_type_support_handle = {
  0,
  &mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_hash,
  &mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_description,
  &mav_planning_msgs__msg__PolygonWithHolesStamped__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mav_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, msg, PolygonWithHolesStamped)() {
  mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, msg, PolygonWithHoles)();
  if (!mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_type_support_handle.typesupport_identifier) {
    mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mav_planning_msgs__msg__PolygonWithHolesStamped__rosidl_typesupport_introspection_c__PolygonWithHolesStamped_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
