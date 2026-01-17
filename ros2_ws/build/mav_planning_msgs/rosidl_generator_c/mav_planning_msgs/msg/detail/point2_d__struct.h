// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_planning_msgs:msg/Point2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/point2_d.h"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__STRUCT_H_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/Point2D in the package mav_planning_msgs.
/**
  * This contains the position of a 2D point.
 */
typedef struct mav_planning_msgs__msg__Point2D
{
  double x;
  double y;
} mav_planning_msgs__msg__Point2D;

// Struct for a sequence of mav_planning_msgs__msg__Point2D.
typedef struct mav_planning_msgs__msg__Point2D__Sequence
{
  mav_planning_msgs__msg__Point2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__msg__Point2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__STRUCT_H_
