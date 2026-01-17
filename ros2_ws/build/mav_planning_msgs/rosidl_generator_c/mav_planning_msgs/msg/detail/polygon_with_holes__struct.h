// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_planning_msgs:msg/PolygonWithHoles.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon_with_holes.h"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__STRUCT_H_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'hull'
// Member 'holes'
#include "mav_planning_msgs/msg/detail/polygon2_d__struct.h"

/// Struct defined in msg/PolygonWithHoles in the package mav_planning_msgs.
/**
  * A message to define a 2D polygon with holes.
 */
typedef struct mav_planning_msgs__msg__PolygonWithHoles
{
  mav_planning_msgs__msg__Polygon2D hull;
  mav_planning_msgs__msg__Polygon2D__Sequence holes;
} mav_planning_msgs__msg__PolygonWithHoles;

// Struct for a sequence of mav_planning_msgs__msg__PolygonWithHoles.
typedef struct mav_planning_msgs__msg__PolygonWithHoles__Sequence
{
  mav_planning_msgs__msg__PolygonWithHoles * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__msg__PolygonWithHoles__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES__STRUCT_H_
