// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_msgs:msg/Actuators.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/actuators.h"


#ifndef MAV_MSGS__MSG__DETAIL__ACTUATORS__STRUCT_H_
#define MAV_MSGS__MSG__DETAIL__ACTUATORS__STRUCT_H_

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
// Member 'angles'
// Member 'angular_velocities'
// Member 'normalized'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Actuators in the package mav_msgs.
typedef struct mav_msgs__msg__Actuators
{
  std_msgs__msg__Header header;
  /// This message defines lowest level commands to be sent to the actuator(s).
  /// Angle of the actuator in.
  /// E.g. servo angle of a control surface(not angle of the surface!), orientation-angle of a thruster.
  rosidl_runtime_c__double__Sequence angles;
  /// Angular velocities of the actuator in.
  /// E.g. "rpm" of rotors, propellers, thrusters
  rosidl_runtime_c__double__Sequence angular_velocities;
  /// Everything that does not fit the above, normalized between.
  rosidl_runtime_c__double__Sequence normalized;
} mav_msgs__msg__Actuators;

// Struct for a sequence of mav_msgs__msg__Actuators.
typedef struct mav_msgs__msg__Actuators__Sequence
{
  mav_msgs__msg__Actuators * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_msgs__msg__Actuators__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__ACTUATORS__STRUCT_H_
