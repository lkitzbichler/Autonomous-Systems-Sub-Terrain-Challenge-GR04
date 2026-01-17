// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_msgs:msg/AttitudeThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/attitude_thrust.h"


#ifndef MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__STRUCT_H_
#define MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__STRUCT_H_

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
// Member 'attitude'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'thrust'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/AttitudeThrust in the package mav_msgs.
typedef struct mav_msgs__msg__AttitudeThrust
{
  std_msgs__msg__Header header;
  /// Attitude expressed in the header/frame_id frame.
  geometry_msgs__msg__Quaternion attitude;
  /// Thrust expressed in the body frame.
  /// For a fixed-wing, usually the x-component is used.
  /// For a multi-rotor, usually the z-component is used.
  /// Set all un-used components to 0.
  geometry_msgs__msg__Vector3 thrust;
} mav_msgs__msg__AttitudeThrust;

// Struct for a sequence of mav_msgs__msg__AttitudeThrust.
typedef struct mav_msgs__msg__AttitudeThrust__Sequence
{
  mav_msgs__msg__AttitudeThrust * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_msgs__msg__AttitudeThrust__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__STRUCT_H_
