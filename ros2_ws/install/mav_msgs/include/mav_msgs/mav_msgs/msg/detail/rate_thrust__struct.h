// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_msgs:msg/RateThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/rate_thrust.h"


#ifndef MAV_MSGS__MSG__DETAIL__RATE_THRUST__STRUCT_H_
#define MAV_MSGS__MSG__DETAIL__RATE_THRUST__STRUCT_H_

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
// Member 'angular_rates'
// Member 'thrust'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/RateThrust in the package mav_msgs.
typedef struct mav_msgs__msg__RateThrust
{
  std_msgs__msg__Header header;
  /// We use the coordinate frames with the following convention:
  ///   x: forward
  ///   y: left
  ///   z: up
  /// Roll-, pitch-, yaw-rate around body axes
  geometry_msgs__msg__Vector3 angular_rates;
  /// Thrust expressed in the body frame.
  /// For a fixed-wing, usually the x-component is used.
  /// For a multi-rotor, usually the z-component is used.
  /// Set all un-used components to 0.
  geometry_msgs__msg__Vector3 thrust;
} mav_msgs__msg__RateThrust;

// Struct for a sequence of mav_msgs__msg__RateThrust.
typedef struct mav_msgs__msg__RateThrust__Sequence
{
  mav_msgs__msg__RateThrust * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_msgs__msg__RateThrust__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__RATE_THRUST__STRUCT_H_
