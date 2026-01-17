// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mav_msgs:msg/Actuators.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mav_msgs/msg/detail/actuators__rosidl_typesupport_introspection_c.h"
#include "mav_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mav_msgs/msg/detail/actuators__functions.h"
#include "mav_msgs/msg/detail/actuators__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `angles`
// Member `angular_velocities`
// Member `normalized`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mav_msgs__msg__Actuators__init(message_memory);
}

void mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_fini_function(void * message_memory)
{
  mav_msgs__msg__Actuators__fini(message_memory);
}

size_t mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__size_function__Actuators__angles(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_const_function__Actuators__angles(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_function__Actuators__angles(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__fetch_function__Actuators__angles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_const_function__Actuators__angles(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__assign_function__Actuators__angles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_function__Actuators__angles(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__resize_function__Actuators__angles(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__size_function__Actuators__angular_velocities(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_const_function__Actuators__angular_velocities(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_function__Actuators__angular_velocities(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__fetch_function__Actuators__angular_velocities(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_const_function__Actuators__angular_velocities(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__assign_function__Actuators__angular_velocities(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_function__Actuators__angular_velocities(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__resize_function__Actuators__angular_velocities(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__size_function__Actuators__normalized(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_const_function__Actuators__normalized(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_function__Actuators__normalized(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__fetch_function__Actuators__normalized(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_const_function__Actuators__normalized(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__assign_function__Actuators__normalized(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_function__Actuators__normalized(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__resize_function__Actuators__normalized(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__Actuators, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__Actuators, angles),  // bytes offset in struct
    NULL,  // default value
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__size_function__Actuators__angles,  // size() function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_const_function__Actuators__angles,  // get_const(index) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_function__Actuators__angles,  // get(index) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__fetch_function__Actuators__angles,  // fetch(index, &value) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__assign_function__Actuators__angles,  // assign(index, value) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__resize_function__Actuators__angles  // resize(index) function pointer
  },
  {
    "angular_velocities",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__Actuators, angular_velocities),  // bytes offset in struct
    NULL,  // default value
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__size_function__Actuators__angular_velocities,  // size() function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_const_function__Actuators__angular_velocities,  // get_const(index) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_function__Actuators__angular_velocities,  // get(index) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__fetch_function__Actuators__angular_velocities,  // fetch(index, &value) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__assign_function__Actuators__angular_velocities,  // assign(index, value) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__resize_function__Actuators__angular_velocities  // resize(index) function pointer
  },
  {
    "normalized",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_msgs__msg__Actuators, normalized),  // bytes offset in struct
    NULL,  // default value
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__size_function__Actuators__normalized,  // size() function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_const_function__Actuators__normalized,  // get_const(index) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__get_function__Actuators__normalized,  // get(index) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__fetch_function__Actuators__normalized,  // fetch(index, &value) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__assign_function__Actuators__normalized,  // assign(index, value) function pointer
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__resize_function__Actuators__normalized  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_message_members = {
  "mav_msgs__msg",  // message namespace
  "Actuators",  // message name
  4,  // number of fields
  sizeof(mav_msgs__msg__Actuators),
  false,  // has_any_key_member_
  mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_message_member_array,  // message members
  mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_init_function,  // function to initialize message memory (memory has to be allocated)
  mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_message_type_support_handle = {
  0,
  &mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_message_members,
  get_message_typesupport_handle_function,
  &mav_msgs__msg__Actuators__get_type_hash,
  &mav_msgs__msg__Actuators__get_type_description,
  &mav_msgs__msg__Actuators__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mav_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_msgs, msg, Actuators)() {
  mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_message_type_support_handle.typesupport_identifier) {
    mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mav_msgs__msg__Actuators__rosidl_typesupport_introspection_c__Actuators_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
