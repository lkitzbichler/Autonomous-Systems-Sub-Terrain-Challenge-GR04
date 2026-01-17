// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from mav_planning_msgs:msg/Point2D.idl
// generated code does not contain a copyright notice
#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mav_planning_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mav_planning_msgs/msg/detail/point2_d__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_mav_planning_msgs__msg__Point2D(
  const mav_planning_msgs__msg__Point2D * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_deserialize_mav_planning_msgs__msg__Point2D(
  eprosima::fastcdr::Cdr &,
  mav_planning_msgs__msg__Point2D * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_mav_planning_msgs__msg__Point2D(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_mav_planning_msgs__msg__Point2D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_key_mav_planning_msgs__msg__Point2D(
  const mav_planning_msgs__msg__Point2D * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_key_mav_planning_msgs__msg__Point2D(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_key_mav_planning_msgs__msg__Point2D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, msg, Point2D)();

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POINT2_D__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
