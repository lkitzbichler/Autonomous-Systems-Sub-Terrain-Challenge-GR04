// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mav_planning_msgs:msg/PointCloudWithPose.idl
// generated code does not contain a copyright notice
#include "mav_planning_msgs/msg/detail/point_cloud_with_pose__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mav_planning_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mav_planning_msgs/msg/detail/point_cloud_with_pose__struct.h"
#include "mav_planning_msgs/msg/detail/point_cloud_with_pose__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/transform_stamped__functions.h"  // sensor_pose
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"  // cloud_in_sensor_frame
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_geometry_msgs__msg__TransformStamped(
  const geometry_msgs__msg__TransformStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_deserialize_geometry_msgs__msg__TransformStamped(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__TransformStamped * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_geometry_msgs__msg__TransformStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_geometry_msgs__msg__TransformStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_key_geometry_msgs__msg__TransformStamped(
  const geometry_msgs__msg__TransformStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_key_geometry_msgs__msg__TransformStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_key_geometry_msgs__msg__TransformStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, TransformStamped)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_sensor_msgs__msg__PointCloud2(
  const sensor_msgs__msg__PointCloud2 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_deserialize_sensor_msgs__msg__PointCloud2(
  eprosima::fastcdr::Cdr & cdr,
  sensor_msgs__msg__PointCloud2 * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_sensor_msgs__msg__PointCloud2(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_sensor_msgs__msg__PointCloud2(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_key_sensor_msgs__msg__PointCloud2(
  const sensor_msgs__msg__PointCloud2 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_key_sensor_msgs__msg__PointCloud2(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_key_sensor_msgs__msg__PointCloud2(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sensor_msgs, msg, PointCloud2)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _PointCloudWithPose__ros_msg_type = mav_planning_msgs__msg__PointCloudWithPose;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_mav_planning_msgs__msg__PointCloudWithPose(
  const mav_planning_msgs__msg__PointCloudWithPose * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: sensor_pose
  {
    cdr_serialize_geometry_msgs__msg__TransformStamped(
      &ros_message->sensor_pose, cdr);
  }

  // Field name: cloud_in_sensor_frame
  {
    cdr_serialize_sensor_msgs__msg__PointCloud2(
      &ros_message->cloud_in_sensor_frame, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_deserialize_mav_planning_msgs__msg__PointCloudWithPose(
  eprosima::fastcdr::Cdr & cdr,
  mav_planning_msgs__msg__PointCloudWithPose * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: sensor_pose
  {
    cdr_deserialize_geometry_msgs__msg__TransformStamped(cdr, &ros_message->sensor_pose);
  }

  // Field name: cloud_in_sensor_frame
  {
    cdr_deserialize_sensor_msgs__msg__PointCloud2(cdr, &ros_message->cloud_in_sensor_frame);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_mav_planning_msgs__msg__PointCloudWithPose(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PointCloudWithPose__ros_msg_type * ros_message = static_cast<const _PointCloudWithPose__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: sensor_pose
  current_alignment += get_serialized_size_geometry_msgs__msg__TransformStamped(
    &(ros_message->sensor_pose), current_alignment);

  // Field name: cloud_in_sensor_frame
  current_alignment += get_serialized_size_sensor_msgs__msg__PointCloud2(
    &(ros_message->cloud_in_sensor_frame), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_mav_planning_msgs__msg__PointCloudWithPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: sensor_pose
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__TransformStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: cloud_in_sensor_frame
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mav_planning_msgs__msg__PointCloudWithPose;
    is_plain =
      (
      offsetof(DataType, cloud_in_sensor_frame) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_key_mav_planning_msgs__msg__PointCloudWithPose(
  const mav_planning_msgs__msg__PointCloudWithPose * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: sensor_pose
  {
    cdr_serialize_key_geometry_msgs__msg__TransformStamped(
      &ros_message->sensor_pose, cdr);
  }

  // Field name: cloud_in_sensor_frame
  {
    cdr_serialize_key_sensor_msgs__msg__PointCloud2(
      &ros_message->cloud_in_sensor_frame, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_key_mav_planning_msgs__msg__PointCloudWithPose(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PointCloudWithPose__ros_msg_type * ros_message = static_cast<const _PointCloudWithPose__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: sensor_pose
  current_alignment += get_serialized_size_key_geometry_msgs__msg__TransformStamped(
    &(ros_message->sensor_pose), current_alignment);

  // Field name: cloud_in_sensor_frame
  current_alignment += get_serialized_size_key_sensor_msgs__msg__PointCloud2(
    &(ros_message->cloud_in_sensor_frame), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_key_mav_planning_msgs__msg__PointCloudWithPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: sensor_pose
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__TransformStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: cloud_in_sensor_frame
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_sensor_msgs__msg__PointCloud2(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = mav_planning_msgs__msg__PointCloudWithPose;
    is_plain =
      (
      offsetof(DataType, cloud_in_sensor_frame) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _PointCloudWithPose__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const mav_planning_msgs__msg__PointCloudWithPose * ros_message = static_cast<const mav_planning_msgs__msg__PointCloudWithPose *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_mav_planning_msgs__msg__PointCloudWithPose(ros_message, cdr);
}

static bool _PointCloudWithPose__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  mav_planning_msgs__msg__PointCloudWithPose * ros_message = static_cast<mav_planning_msgs__msg__PointCloudWithPose *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_mav_planning_msgs__msg__PointCloudWithPose(cdr, ros_message);
}

static uint32_t _PointCloudWithPose__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mav_planning_msgs__msg__PointCloudWithPose(
      untyped_ros_message, 0));
}

static size_t _PointCloudWithPose__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mav_planning_msgs__msg__PointCloudWithPose(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PointCloudWithPose = {
  "mav_planning_msgs::msg",
  "PointCloudWithPose",
  _PointCloudWithPose__cdr_serialize,
  _PointCloudWithPose__cdr_deserialize,
  _PointCloudWithPose__get_serialized_size,
  _PointCloudWithPose__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _PointCloudWithPose__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PointCloudWithPose,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__msg__PointCloudWithPose__get_type_hash,
  &mav_planning_msgs__msg__PointCloudWithPose__get_type_description,
  &mav_planning_msgs__msg__PointCloudWithPose__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, msg, PointCloudWithPose)() {
  return &_PointCloudWithPose__type_support;
}

#if defined(__cplusplus)
}
#endif
