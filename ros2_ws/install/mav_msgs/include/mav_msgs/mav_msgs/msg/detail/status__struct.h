// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_msgs:msg/Status.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/status.h"


#ifndef MAV_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
#define MAV_MSGS__MSG__DETAIL__STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Constant 'RC_COMMAND_ATTITUDE'.
static const char * const mav_msgs__msg__Status__RC_COMMAND_ATTITUDE = "attitude_thrust";

/// Constant 'RC_COMMAND_ATTITUDE_HEIGHT'.
static const char * const mav_msgs__msg__Status__RC_COMMAND_ATTITUDE_HEIGHT = "attitude_height";

/// Constant 'RC_COMMAND_POSITION'.
static const char * const mav_msgs__msg__Status__RC_COMMAND_POSITION = "position";

/// Constant 'MOTOR_STATUS_RUNNING'.
static const char * const mav_msgs__msg__Status__MOTOR_STATUS_RUNNING = "running";

/// Constant 'MOTOR_STATUS_STOPPED'.
static const char * const mav_msgs__msg__Status__MOTOR_STATUS_STOPPED = "stopped";

/// Constant 'MOTOR_STATUS_STARTING'.
static const char * const mav_msgs__msg__Status__MOTOR_STATUS_STARTING = "starting";

/// Constant 'MOTOR_STATUS_STOPPING'.
static const char * const mav_msgs__msg__Status__MOTOR_STATUS_STOPPING = "stopping";

/// Constant 'GPS_STATUS_LOCK'.
static const char * const mav_msgs__msg__Status__GPS_STATUS_LOCK = "lock";

/// Constant 'GPS_STATUS_NO_LOCK'.
static const char * const mav_msgs__msg__Status__GPS_STATUS_NO_LOCK = "no_lock";

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'vehicle_name'
// Member 'vehicle_type'
// Member 'rc_command_mode'
// Member 'motor_status'
// Member 'gps_status'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Status in the package mav_msgs.
typedef struct mav_msgs__msg__Status
{
  std_msgs__msg__Header header;
  /// If values are not known / available, set to -1 or empty string.
  rosidl_runtime_c__String vehicle_name;
  /// E.g. firefly, pelican ...
  rosidl_runtime_c__String vehicle_type;
  /// Battery voltage in V.
  float battery_voltage;
  /// Command mode set on the 3 position switch on the rc.
  rosidl_runtime_c__String rc_command_mode;
  /// Reports whether the serial command interface is enabled.
  bool command_interface_enabled;
  /// Flight time in s.
  float flight_time;
  /// MAV uptime in s.
  float system_uptime;
  /// MAV CPU load: 0.0 ... 1.0
  float cpu_load;
  /// Current motor status: running, stopped, starting, stopping.
  rosidl_runtime_c__String motor_status;
  /// True if vehicle is actually in air, false otherwise
  bool in_air;
  /// GPS status: lock, no_lock
  rosidl_runtime_c__String gps_status;
  /// Number of visible satellites
  int32_t gps_num_satellites;
} mav_msgs__msg__Status;

// Struct for a sequence of mav_msgs__msg__Status.
typedef struct mav_msgs__msg__Status__Sequence
{
  mav_msgs__msg__Status * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_msgs__msg__Status__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__STATUS__STRUCT_H_
