// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_planning_msgs:msg/PolygonWithHolesStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polygon_with_holes_stamped.h"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES_STAMPED__STRUCT_H_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES_STAMPED__STRUCT_H_

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
// Member 'polygon'
#include "mav_planning_msgs/msg/detail/polygon_with_holes__struct.h"

/// Struct defined in msg/PolygonWithHolesStamped in the package mav_planning_msgs.
/**
  * A message to define a 2D polygon with holes, stamp, and altitude above ground.
 */
typedef struct mav_planning_msgs__msg__PolygonWithHolesStamped
{
  std_msgs__msg__Header header;
  double altitude;
  mav_planning_msgs__msg__PolygonWithHoles polygon;
} mav_planning_msgs__msg__PolygonWithHolesStamped;

// Struct for a sequence of mav_planning_msgs__msg__PolygonWithHolesStamped.
typedef struct mav_planning_msgs__msg__PolygonWithHolesStamped__Sequence
{
  mav_planning_msgs__msg__PolygonWithHolesStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__msg__PolygonWithHolesStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYGON_WITH_HOLES_STAMPED__STRUCT_H_
