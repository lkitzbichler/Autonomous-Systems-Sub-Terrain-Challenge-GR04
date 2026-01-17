// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mav_planning_msgs:msg/PolygonWithHoles.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mav_planning_msgs/msg/detail/polygon_with_holes__rosidl_typesupport_introspection_c.h"
#include "mav_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mav_planning_msgs/msg/detail/polygon_with_holes__functions.h"
#include "mav_planning_msgs/msg/detail/polygon_with_holes__struct.h"


// Include directives for member types
// Member `hull`
// Member `holes`
#include "mav_planning_msgs/msg/polygon2_d.h"
// Member `hull`
// Member `holes`
#include "mav_planning_msgs/msg/detail/polygon2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mav_planning_msgs__msg__PolygonWithHoles__init(message_memory);
}

void mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_fini_function(void * message_memory)
{
  mav_planning_msgs__msg__PolygonWithHoles__fini(message_memory);
}

size_t mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__size_function__PolygonWithHoles__holes(
  const void * untyped_member)
{
  const mav_planning_msgs__msg__Polygon2D__Sequence * member =
    (const mav_planning_msgs__msg__Polygon2D__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__get_const_function__PolygonWithHoles__holes(
  const void * untyped_member, size_t index)
{
  const mav_planning_msgs__msg__Polygon2D__Sequence * member =
    (const mav_planning_msgs__msg__Polygon2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__get_function__PolygonWithHoles__holes(
  void * untyped_member, size_t index)
{
  mav_planning_msgs__msg__Polygon2D__Sequence * member =
    (mav_planning_msgs__msg__Polygon2D__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__fetch_function__PolygonWithHoles__holes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const mav_planning_msgs__msg__Polygon2D * item =
    ((const mav_planning_msgs__msg__Polygon2D *)
    mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__get_const_function__PolygonWithHoles__holes(untyped_member, index));
  mav_planning_msgs__msg__Polygon2D * value =
    (mav_planning_msgs__msg__Polygon2D *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__assign_function__PolygonWithHoles__holes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  mav_planning_msgs__msg__Polygon2D * item =
    ((mav_planning_msgs__msg__Polygon2D *)
    mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__get_function__PolygonWithHoles__holes(untyped_member, index));
  const mav_planning_msgs__msg__Polygon2D * value =
    (const mav_planning_msgs__msg__Polygon2D *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__resize_function__PolygonWithHoles__holes(
  void * untyped_member, size_t size)
{
  mav_planning_msgs__msg__Polygon2D__Sequence * member =
    (mav_planning_msgs__msg__Polygon2D__Sequence *)(untyped_member);
  mav_planning_msgs__msg__Polygon2D__Sequence__fini(member);
  return mav_planning_msgs__msg__Polygon2D__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_member_array[2] = {
  {
    "hull",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolygonWithHoles, hull),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "holes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolygonWithHoles, holes),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__size_function__PolygonWithHoles__holes,  // size() function pointer
    mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__get_const_function__PolygonWithHoles__holes,  // get_const(index) function pointer
    mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__get_function__PolygonWithHoles__holes,  // get(index) function pointer
    mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__fetch_function__PolygonWithHoles__holes,  // fetch(index, &value) function pointer
    mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__assign_function__PolygonWithHoles__holes,  // assign(index, value) function pointer
    mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__resize_function__PolygonWithHoles__holes  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_members = {
  "mav_planning_msgs__msg",  // message namespace
  "PolygonWithHoles",  // message name
  2,  // number of fields
  sizeof(mav_planning_msgs__msg__PolygonWithHoles),
  false,  // has_any_key_member_
  mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_member_array,  // message members
  mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_init_function,  // function to initialize message memory (memory has to be allocated)
  mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_type_support_handle = {
  0,
  &mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__msg__PolygonWithHoles__get_type_hash,
  &mav_planning_msgs__msg__PolygonWithHoles__get_type_description,
  &mav_planning_msgs__msg__PolygonWithHoles__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mav_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, msg, PolygonWithHoles)() {
  mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, msg, Polygon2D)();
  mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, msg, Polygon2D)();
  if (!mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_type_support_handle.typesupport_identifier) {
    mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mav_planning_msgs__msg__PolygonWithHoles__rosidl_typesupport_introspection_c__PolygonWithHoles_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
