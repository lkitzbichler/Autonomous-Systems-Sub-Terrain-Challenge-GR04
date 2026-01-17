// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from mav_planning_msgs:msg/PointCloudWithPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/point_cloud_with_pose.h"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__STRUCT_H_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__STRUCT_H_

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
// Member 'sensor_pose'
#include "geometry_msgs/msg/detail/transform_stamped__struct.h"
// Member 'cloud_in_sensor_frame'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

/// Struct defined in msg/PointCloudWithPose in the package mav_planning_msgs.
typedef struct mav_planning_msgs__msg__PointCloudWithPose
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__TransformStamped sensor_pose;
  sensor_msgs__msg__PointCloud2 cloud_in_sensor_frame;
} mav_planning_msgs__msg__PointCloudWithPose;

// Struct for a sequence of mav_planning_msgs__msg__PointCloudWithPose.
typedef struct mav_planning_msgs__msg__PointCloudWithPose__Sequence
{
  mav_planning_msgs__msg__PointCloudWithPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} mav_planning_msgs__msg__PointCloudWithPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__STRUCT_H_
