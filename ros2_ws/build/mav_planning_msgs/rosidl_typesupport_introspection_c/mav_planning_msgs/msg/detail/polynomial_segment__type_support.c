// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from mav_planning_msgs:msg/PolynomialSegment.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "mav_planning_msgs/msg/detail/polynomial_segment__rosidl_typesupport_introspection_c.h"
#include "mav_planning_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "mav_planning_msgs/msg/detail/polynomial_segment__functions.h"
#include "mav_planning_msgs/msg/detail/polynomial_segment__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `segment_time`
#include "builtin_interfaces/msg/duration.h"
// Member `segment_time`
#include "builtin_interfaces/msg/detail/duration__rosidl_typesupport_introspection_c.h"
// Member `x`
// Member `y`
// Member `z`
// Member `rx`
// Member `ry`
// Member `rz`
// Member `yaw`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  mav_planning_msgs__msg__PolynomialSegment__init(message_memory);
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_fini_function(void * message_memory)
{
  mav_planning_msgs__msg__PolynomialSegment__fini(message_memory);
}

size_t mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__x(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__x(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__x(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__y(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__y(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__y(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__z(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__z(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__z(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__z(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__z(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__z(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__rx(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__rx(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__rx(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__rx(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__rx(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__rx(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__rx(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__rx(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__ry(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__ry(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__ry(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__ry(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__ry(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__ry(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__ry(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__ry(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__rz(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__rz(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__rz(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__rz(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__rz(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__rz(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__rz(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__rz(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__yaw(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__yaw(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__yaw(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__yaw(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__yaw(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__yaw(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__yaw(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__yaw(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_member_array[10] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_coeffs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, num_coeffs),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "segment_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, segment_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, x),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__x,  // size() function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__x,  // get_const(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__x,  // get(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__x,  // fetch(index, &value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__x,  // assign(index, value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__x  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, y),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__y,  // size() function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__y,  // get_const(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__y,  // get(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__y,  // fetch(index, &value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__y,  // assign(index, value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__y  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, z),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__z,  // size() function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__z,  // get_const(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__z,  // get(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__z,  // fetch(index, &value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__z,  // assign(index, value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__z  // resize(index) function pointer
  },
  {
    "rx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, rx),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__rx,  // size() function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__rx,  // get_const(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__rx,  // get(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__rx,  // fetch(index, &value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__rx,  // assign(index, value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__rx  // resize(index) function pointer
  },
  {
    "ry",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, ry),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__ry,  // size() function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__ry,  // get_const(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__ry,  // get(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__ry,  // fetch(index, &value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__ry,  // assign(index, value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__ry  // resize(index) function pointer
  },
  {
    "rz",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, rz),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__rz,  // size() function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__rz,  // get_const(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__rz,  // get(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__rz,  // fetch(index, &value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__rz,  // assign(index, value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__rz  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(mav_planning_msgs__msg__PolynomialSegment, yaw),  // bytes offset in struct
    NULL,  // default value
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__size_function__PolynomialSegment__yaw,  // size() function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_const_function__PolynomialSegment__yaw,  // get_const(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__get_function__PolynomialSegment__yaw,  // get(index) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__fetch_function__PolynomialSegment__yaw,  // fetch(index, &value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__assign_function__PolynomialSegment__yaw,  // assign(index, value) function pointer
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__resize_function__PolynomialSegment__yaw  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_members = {
  "mav_planning_msgs__msg",  // message namespace
  "PolynomialSegment",  // message name
  10,  // number of fields
  sizeof(mav_planning_msgs__msg__PolynomialSegment),
  false,  // has_any_key_member_
  mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_member_array,  // message members
  mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_init_function,  // function to initialize message memory (memory has to be allocated)
  mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_type_support_handle = {
  0,
  &mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_members,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__msg__PolynomialSegment__get_type_hash,
  &mav_planning_msgs__msg__PolynomialSegment__get_type_description,
  &mav_planning_msgs__msg__PolynomialSegment__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_mav_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, mav_planning_msgs, msg, PolynomialSegment)() {
  mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, builtin_interfaces, msg, Duration)();
  if (!mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_type_support_handle.typesupport_identifier) {
    mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &mav_planning_msgs__msg__PolynomialSegment__rosidl_typesupport_introspection_c__PolynomialSegment_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
