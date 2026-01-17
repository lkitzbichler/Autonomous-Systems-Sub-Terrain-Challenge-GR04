// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_planning_msgs:msg/Polygon2D.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon2_d.h"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__STRUCT_H_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'points'
#include "mav_planning_msgs/msg/detail/point2_d__struct.h"

/// Struct defined in msg/Polygon2D in the package mav_planning_msgs.
/**
  * A specification of a 2D polygon where the first and last points are assumed to be connected.
 */
typedef struct mav_planning_msgs__msg__Polygon2D
{
  mav_planning_msgs__msg__Point2D__Sequence points;
} mav_planning_msgs__msg__Polygon2D;

// Struct for a sequence of mav_planning_msgs__msg__Polygon2D.
typedef struct mav_planning_msgs__msg__Polygon2D__Sequence
{
  mav_planning_msgs__msg__Polygon2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__msg__Polygon2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON2_D__STRUCT_H_
