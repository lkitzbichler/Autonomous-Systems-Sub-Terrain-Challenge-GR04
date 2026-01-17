// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_planning_msgs:msg/PolynomialSegment4D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polynomial_segment4_d.h"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT4_D__STRUCT_H_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT4_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'segment_time'
#include "builtin_interfaces/msg/detail/duration__struct.h"
// Member 'x'
// Member 'y'
// Member 'z'
// Member 'yaw'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/PolynomialSegment4D in the package mav_planning_msgs.
typedef struct mav_planning_msgs__msg__PolynomialSegment4D
{
  std_msgs__msg__Header header;
  /// order of the polynomial + 1, should match size of x[]
  int32_t num_coeffs;
  /// duration of the segment
  builtin_interfaces__msg__Duration segment_time;
  /// coefficients for the x-axis, INCREASING order
  rosidl_runtime_c__double__Sequence x;
  /// coefficients for the y-axis, INCREASING order
  rosidl_runtime_c__double__Sequence y;
  /// coefficients for the z-axis, INCREASING order
  rosidl_runtime_c__double__Sequence z;
  /// coefficients for the yaw, INCREASING order
  rosidl_runtime_c__double__Sequence yaw;
} mav_planning_msgs__msg__PolynomialSegment4D;

// Struct for a sequence of mav_planning_msgs__msg__PolynomialSegment4D.
typedef struct mav_planning_msgs__msg__PolynomialSegment4D__Sequence
{
  mav_planning_msgs__msg__PolynomialSegment4D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__msg__PolynomialSegment4D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT4_D__STRUCT_H_
