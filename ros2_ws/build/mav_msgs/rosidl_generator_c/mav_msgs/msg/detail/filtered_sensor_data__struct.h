// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_msgs:msg/FilteredSensorData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/filtered_sensor_data.h"


#ifndef MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__STRUCT_H_
#define MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__STRUCT_H_

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
// Member 'accelerometer'
// Member 'gyroscope'
// Member 'magnetometer'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/FilteredSensorData in the package mav_msgs.
typedef struct mav_msgs__msg__FilteredSensorData
{
  std_msgs__msg__Header header;
  /// acceleration in vehicle frame
  geometry_msgs__msg__Vector3 accelerometer;
  /// rotational velocity in vehicle frame
  geometry_msgs__msg__Vector3 gyroscope;
  /// Magnetometer measurements in vehicle frame
  geometry_msgs__msg__Vector3 magnetometer;
  /// Height from barometer relative to start-up point
  double barometer;
} mav_msgs__msg__FilteredSensorData;

// Struct for a sequence of mav_msgs__msg__FilteredSensorData.
typedef struct mav_msgs__msg__FilteredSensorData__Sequence
{
  mav_msgs__msg__FilteredSensorData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_msgs__msg__FilteredSensorData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__STRUCT_H_
