// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from mav_planning_msgs:msg/PointCloudWithPose.idl
// generated code does not contain a copyright notice

#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "mav_planning_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "mav_planning_msgs/msg/detail/point_cloud_with_pose__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace mav_planning_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mav_planning_msgs
cdr_serialize(
  const mav_planning_msgs::msg::PointCloudWithPose & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mav_planning_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  mav_planning_msgs::msg::PointCloudWithPose & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mav_planning_msgs
get_serialized_size(
  const mav_planning_msgs::msg::PointCloudWithPose & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mav_planning_msgs
max_serialized_size_PointCloudWithPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mav_planning_msgs
cdr_serialize_key(
  const mav_planning_msgs::msg::PointCloudWithPose & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mav_planning_msgs
get_serialized_size_key(
  const mav_planning_msgs::msg::PointCloudWithPose & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mav_planning_msgs
max_serialized_size_key_PointCloudWithPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace mav_planning_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_mav_planning_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, mav_planning_msgs, msg, PointCloudWithPose)();

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
