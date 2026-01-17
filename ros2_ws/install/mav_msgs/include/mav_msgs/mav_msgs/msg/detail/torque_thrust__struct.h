// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_msgs:msg/TorqueThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/torque_thrust.h"


#ifndef MAV_MSGS__MSG__DETAIL__TORQUE_THRUST__STRUCT_H_
#define MAV_MSGS__MSG__DETAIL__TORQUE_THRUST__STRUCT_H_

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
// Member 'torque'
// Member 'thrust'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/TorqueThrust in the package mav_msgs.
typedef struct mav_msgs__msg__TorqueThrust
{
  std_msgs__msg__Header header;
  /// We use the coordinate frames with the following convention:
  ///   x: forward
  ///   y: left
  ///   z: up
  /// Torque expressed in the body frame.
  geometry_msgs__msg__Vector3 torque;
  /// Thrust expressed in the body frame.
  /// For a fixed-wing, usually the x-component is used.
  /// For a multi-rotor, usually the z-component is used.
  /// Set all un-used components to 0.
  geometry_msgs__msg__Vector3 thrust;
} mav_msgs__msg__TorqueThrust;

// Struct for a sequence of mav_msgs__msg__TorqueThrust.
typedef struct mav_msgs__msg__TorqueThrust__Sequence
{
  mav_msgs__msg__TorqueThrust * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_msgs__msg__TorqueThrust__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__TORQUE_THRUST__STRUCT_H_
