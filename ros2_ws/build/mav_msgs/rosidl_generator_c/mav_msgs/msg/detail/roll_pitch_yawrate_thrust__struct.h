// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_msgs:msg/RollPitchYawrateThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/roll_pitch_yawrate_thrust.h"


#ifndef MAV_MSGS__MSG__DETAIL__ROLL_PITCH_YAWRATE_THRUST__STRUCT_H_
#define MAV_MSGS__MSG__DETAIL__ROLL_PITCH_YAWRATE_THRUST__STRUCT_H_

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
// Member 'thrust'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/RollPitchYawrateThrust in the package mav_msgs.
typedef struct mav_msgs__msg__RollPitchYawrateThrust
{
  std_msgs__msg__Header header;
  /// We use the coordinate frames with the following convention:
  ///   x: forward
  ///   y: left
  ///   z: up
  /// rotation convention (z-y'-x''):
  /// yaw rotates around fixed frame's z axis
  /// pitch rotates around new y-axis (y')
  /// roll rotates around new x-axis (x'')
  /// This is a convenience-message to support that low-level (microcontroller-based) state
  /// estimators may not have knowledge about the absolute yaw.
  /// Roll- and pitch-angle should be specified in the header/frame_id frame
  /// Roll angle
  double roll;
  /// Pitch angle
  double pitch;
  /// Yaw rate around z-axis
  double yaw_rate;
  /// Thrust expressed in the body frame.
  /// For a fixed-wing, usually the x-component is used.
  /// For a multi-rotor, usually the z-component is used.
  /// Set all un-used components to 0.
  geometry_msgs__msg__Vector3 thrust;
} mav_msgs__msg__RollPitchYawrateThrust;

// Struct for a sequence of mav_msgs__msg__RollPitchYawrateThrust.
typedef struct mav_msgs__msg__RollPitchYawrateThrust__Sequence
{
  mav_msgs__msg__RollPitchYawrateThrust * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_msgs__msg__RollPitchYawrateThrust__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__ROLL_PITCH_YAWRATE_THRUST__STRUCT_H_
