// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_planning_msgs:msg/PolynomialTrajectory.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polynomial_trajectory.h"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__STRUCT_H_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__STRUCT_H_

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
// Member 'segments'
#include "mav_planning_msgs/msg/detail/polynomial_segment__struct.h"

/// Struct defined in msg/PolynomialTrajectory in the package mav_planning_msgs.
typedef struct mav_planning_msgs__msg__PolynomialTrajectory
{
  std_msgs__msg__Header header;
  mav_planning_msgs__msg__PolynomialSegment__Sequence segments;
} mav_planning_msgs__msg__PolynomialTrajectory;

// Struct for a sequence of mav_planning_msgs__msg__PolynomialTrajectory.
typedef struct mav_planning_msgs__msg__PolynomialTrajectory__Sequence
{
  mav_planning_msgs__msg__PolynomialTrajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__msg__PolynomialTrajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__STRUCT_H_
