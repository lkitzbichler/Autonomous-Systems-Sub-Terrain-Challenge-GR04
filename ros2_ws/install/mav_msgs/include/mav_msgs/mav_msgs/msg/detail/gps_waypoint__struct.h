// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_msgs:msg/GpsWaypoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/gps_waypoint.h"


#ifndef MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__STRUCT_H_
#define MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__STRUCT_H_

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

/// Struct defined in msg/GpsWaypoint in the package mav_msgs.
typedef struct mav_msgs__msg__GpsWaypoint
{
  std_msgs__msg__Header header;
  /// latitude in degree
  double latitude;
  /// longitude in degree
  double longitude;
  /// above start-up point
  double altitude;
  /// GPS heading
  double heading;
  /// maximum approach speed
  double max_speed;
  /// maximum approach accelerations
  double max_acc;
} mav_msgs__msg__GpsWaypoint;

// Struct for a sequence of mav_msgs__msg__GpsWaypoint.
typedef struct mav_msgs__msg__GpsWaypoint__Sequence
{
  mav_msgs__msg__GpsWaypoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_msgs__msg__GpsWaypoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__GPS_WAYPOINT__STRUCT_H_
