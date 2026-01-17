// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from mav_msgs:msg/Actuators.idl
// generated code does not contain a copyright notice
#ifndef MAV_MSGS__MSG__DETAIL__ACTUATORS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define MAV_MSGS__MSG__DETAIL__ACTUATORS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mav_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mav_msgs/msg/detail/actuators__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_msgs
bool cdr_serialize_mav_msgs__msg__Actuators(
  const mav_msgs__msg__Actuators * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_msgs
bool cdr_deserialize_mav_msgs__msg__Actuators(
  eprosima::fastcdr::Cdr &,
  mav_msgs__msg__Actuators * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_msgs
size_t get_serialized_size_mav_msgs__msg__Actuators(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_msgs
size_t max_serialized_size_mav_msgs__msg__Actuators(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_msgs
bool cdr_serialize_key_mav_msgs__msg__Actuators(
  const mav_msgs__msg__Actuators * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_msgs
size_t get_serialized_size_key_mav_msgs__msg__Actuators(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_msgs
size_t max_serialized_size_key_mav_msgs__msg__Actuators(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_msgs, msg, Actuators)();

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__ACTUATORS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
