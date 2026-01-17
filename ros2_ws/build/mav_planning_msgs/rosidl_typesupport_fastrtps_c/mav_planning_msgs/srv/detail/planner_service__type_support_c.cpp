// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from mav_planning_msgs:srv/PlannerService.idl
// generated code does not contain a copyright notice
#include "mav_planning_msgs/srv/detail/planner_service__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "mav_planning_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mav_planning_msgs/srv/detail/planner_service__struct.h"
#include "mav_planning_msgs/srv/detail/planner_service__functions.h"
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

#include "geometry_msgs/msg/detail/pose_stamped__functions.h"  // goal_pose, start_pose
#include "geometry_msgs/msg/detail/vector3__functions.h"  // bounding_box, goal_velocity, start_velocity

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_geometry_msgs__msg__PoseStamped(
  const geometry_msgs__msg__PoseStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_deserialize_geometry_msgs__msg__PoseStamped(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__PoseStamped * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_geometry_msgs__msg__PoseStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_geometry_msgs__msg__PoseStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_key_geometry_msgs__msg__PoseStamped(
  const geometry_msgs__msg__PoseStamped * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_key_geometry_msgs__msg__PoseStamped(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_key_geometry_msgs__msg__PoseStamped(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseStamped)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_geometry_msgs__msg__Vector3(
  const geometry_msgs__msg__Vector3 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_deserialize_geometry_msgs__msg__Vector3(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__Vector3 * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_key_geometry_msgs__msg__Vector3(
  const geometry_msgs__msg__Vector3 * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_key_geometry_msgs__msg__Vector3(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_key_geometry_msgs__msg__Vector3(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Vector3)();


using _PlannerService_Request__ros_msg_type = mav_planning_msgs__srv__PlannerService_Request;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_mav_planning_msgs__srv__PlannerService_Request(
  const mav_planning_msgs__srv__PlannerService_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: start_pose
  {
    cdr_serialize_geometry_msgs__msg__PoseStamped(
      &ros_message->start_pose, cdr);
  }

  // Field name: start_velocity
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->start_velocity, cdr);
  }

  // Field name: goal_pose
  {
    cdr_serialize_geometry_msgs__msg__PoseStamped(
      &ros_message->goal_pose, cdr);
  }

  // Field name: goal_velocity
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->goal_velocity, cdr);
  }

  // Field name: bounding_box
  {
    cdr_serialize_geometry_msgs__msg__Vector3(
      &ros_message->bounding_box, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_deserialize_mav_planning_msgs__srv__PlannerService_Request(
  eprosima::fastcdr::Cdr & cdr,
  mav_planning_msgs__srv__PlannerService_Request * ros_message)
{
  // Field name: start_pose
  {
    cdr_deserialize_geometry_msgs__msg__PoseStamped(cdr, &ros_message->start_pose);
  }

  // Field name: start_velocity
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->start_velocity);
  }

  // Field name: goal_pose
  {
    cdr_deserialize_geometry_msgs__msg__PoseStamped(cdr, &ros_message->goal_pose);
  }

  // Field name: goal_velocity
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->goal_velocity);
  }

  // Field name: bounding_box
  {
    cdr_deserialize_geometry_msgs__msg__Vector3(cdr, &ros_message->bounding_box);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_mav_planning_msgs__srv__PlannerService_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlannerService_Request__ros_msg_type * ros_message = static_cast<const _PlannerService_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: start_pose
  current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
    &(ros_message->start_pose), current_alignment);

  // Field name: start_velocity
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->start_velocity), current_alignment);

  // Field name: goal_pose
  current_alignment += get_serialized_size_geometry_msgs__msg__PoseStamped(
    &(ros_message->goal_pose), current_alignment);

  // Field name: goal_velocity
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->goal_velocity), current_alignment);

  // Field name: bounding_box
  current_alignment += get_serialized_size_geometry_msgs__msg__Vector3(
    &(ros_message->bounding_box), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_mav_planning_msgs__srv__PlannerService_Request(
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

  // Field name: start_pose
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: start_velocity
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: goal_pose
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: goal_velocity
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: bounding_box
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__Vector3(
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
    using DataType = mav_planning_msgs__srv__PlannerService_Request;
    is_plain =
      (
      offsetof(DataType, bounding_box) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_key_mav_planning_msgs__srv__PlannerService_Request(
  const mav_planning_msgs__srv__PlannerService_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: start_pose
  {
    cdr_serialize_key_geometry_msgs__msg__PoseStamped(
      &ros_message->start_pose, cdr);
  }

  // Field name: start_velocity
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->start_velocity, cdr);
  }

  // Field name: goal_pose
  {
    cdr_serialize_key_geometry_msgs__msg__PoseStamped(
      &ros_message->goal_pose, cdr);
  }

  // Field name: goal_velocity
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->goal_velocity, cdr);
  }

  // Field name: bounding_box
  {
    cdr_serialize_key_geometry_msgs__msg__Vector3(
      &ros_message->bounding_box, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_key_mav_planning_msgs__srv__PlannerService_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlannerService_Request__ros_msg_type * ros_message = static_cast<const _PlannerService_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: start_pose
  current_alignment += get_serialized_size_key_geometry_msgs__msg__PoseStamped(
    &(ros_message->start_pose), current_alignment);

  // Field name: start_velocity
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->start_velocity), current_alignment);

  // Field name: goal_pose
  current_alignment += get_serialized_size_key_geometry_msgs__msg__PoseStamped(
    &(ros_message->goal_pose), current_alignment);

  // Field name: goal_velocity
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->goal_velocity), current_alignment);

  // Field name: bounding_box
  current_alignment += get_serialized_size_key_geometry_msgs__msg__Vector3(
    &(ros_message->bounding_box), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_key_mav_planning_msgs__srv__PlannerService_Request(
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
  // Field name: start_pose
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: start_velocity
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: goal_pose
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__PoseStamped(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: goal_velocity
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: bounding_box
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__Vector3(
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
    using DataType = mav_planning_msgs__srv__PlannerService_Request;
    is_plain =
      (
      offsetof(DataType, bounding_box) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _PlannerService_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const mav_planning_msgs__srv__PlannerService_Request * ros_message = static_cast<const mav_planning_msgs__srv__PlannerService_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_mav_planning_msgs__srv__PlannerService_Request(ros_message, cdr);
}

static bool _PlannerService_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  mav_planning_msgs__srv__PlannerService_Request * ros_message = static_cast<mav_planning_msgs__srv__PlannerService_Request *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_mav_planning_msgs__srv__PlannerService_Request(cdr, ros_message);
}

static uint32_t _PlannerService_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mav_planning_msgs__srv__PlannerService_Request(
      untyped_ros_message, 0));
}

static size_t _PlannerService_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mav_planning_msgs__srv__PlannerService_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PlannerService_Request = {
  "mav_planning_msgs::srv",
  "PlannerService_Request",
  _PlannerService_Request__cdr_serialize,
  _PlannerService_Request__cdr_deserialize,
  _PlannerService_Request__get_serialized_size,
  _PlannerService_Request__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _PlannerService_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PlannerService_Request,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Request__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Request)() {
  return &_PlannerService_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "mav_planning_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__struct.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "mav_planning_msgs/msg/detail/polynomial_trajectory4_d__functions.h"  // polynomial_plan_4d
#include "mav_planning_msgs/msg/detail/polynomial_trajectory__functions.h"  // polynomial_plan
#include "trajectory_msgs/msg/detail/multi_dof_joint_trajectory__functions.h"  // sampled_plan

// forward declare type support functions

bool cdr_serialize_mav_planning_msgs__msg__PolynomialTrajectory(
  const mav_planning_msgs__msg__PolynomialTrajectory * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_mav_planning_msgs__msg__PolynomialTrajectory(
  eprosima::fastcdr::Cdr & cdr,
  mav_planning_msgs__msg__PolynomialTrajectory * ros_message);

size_t get_serialized_size_mav_planning_msgs__msg__PolynomialTrajectory(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_mav_planning_msgs__msg__PolynomialTrajectory(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_mav_planning_msgs__msg__PolynomialTrajectory(
  const mav_planning_msgs__msg__PolynomialTrajectory * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_mav_planning_msgs__msg__PolynomialTrajectory(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_mav_planning_msgs__msg__PolynomialTrajectory(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, msg, PolynomialTrajectory)();

bool cdr_serialize_mav_planning_msgs__msg__PolynomialTrajectory4D(
  const mav_planning_msgs__msg__PolynomialTrajectory4D * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_mav_planning_msgs__msg__PolynomialTrajectory4D(
  eprosima::fastcdr::Cdr & cdr,
  mav_planning_msgs__msg__PolynomialTrajectory4D * ros_message);

size_t get_serialized_size_mav_planning_msgs__msg__PolynomialTrajectory4D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_mav_planning_msgs__msg__PolynomialTrajectory4D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_mav_planning_msgs__msg__PolynomialTrajectory4D(
  const mav_planning_msgs__msg__PolynomialTrajectory4D * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_mav_planning_msgs__msg__PolynomialTrajectory4D(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_mav_planning_msgs__msg__PolynomialTrajectory4D(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, msg, PolynomialTrajectory4D)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_trajectory_msgs__msg__MultiDOFJointTrajectory(
  const trajectory_msgs__msg__MultiDOFJointTrajectory * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_deserialize_trajectory_msgs__msg__MultiDOFJointTrajectory(
  eprosima::fastcdr::Cdr & cdr,
  trajectory_msgs__msg__MultiDOFJointTrajectory * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_trajectory_msgs__msg__MultiDOFJointTrajectory(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_trajectory_msgs__msg__MultiDOFJointTrajectory(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_key_trajectory_msgs__msg__MultiDOFJointTrajectory(
  const trajectory_msgs__msg__MultiDOFJointTrajectory * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_key_trajectory_msgs__msg__MultiDOFJointTrajectory(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_key_trajectory_msgs__msg__MultiDOFJointTrajectory(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, trajectory_msgs, msg, MultiDOFJointTrajectory)();


using _PlannerService_Response__ros_msg_type = mav_planning_msgs__srv__PlannerService_Response;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_mav_planning_msgs__srv__PlannerService_Response(
  const mav_planning_msgs__srv__PlannerService_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: polynomial_plan
  {
    cdr_serialize_mav_planning_msgs__msg__PolynomialTrajectory(
      &ros_message->polynomial_plan, cdr);
  }

  // Field name: polynomial_plan_4d
  {
    cdr_serialize_mav_planning_msgs__msg__PolynomialTrajectory4D(
      &ros_message->polynomial_plan_4d, cdr);
  }

  // Field name: sampled_plan
  {
    cdr_serialize_trajectory_msgs__msg__MultiDOFJointTrajectory(
      &ros_message->sampled_plan, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_deserialize_mav_planning_msgs__srv__PlannerService_Response(
  eprosima::fastcdr::Cdr & cdr,
  mav_planning_msgs__srv__PlannerService_Response * ros_message)
{
  // Field name: success
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->success = tmp ? true : false;
  }

  // Field name: polynomial_plan
  {
    cdr_deserialize_mav_planning_msgs__msg__PolynomialTrajectory(cdr, &ros_message->polynomial_plan);
  }

  // Field name: polynomial_plan_4d
  {
    cdr_deserialize_mav_planning_msgs__msg__PolynomialTrajectory4D(cdr, &ros_message->polynomial_plan_4d);
  }

  // Field name: sampled_plan
  {
    cdr_deserialize_trajectory_msgs__msg__MultiDOFJointTrajectory(cdr, &ros_message->sampled_plan);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_mav_planning_msgs__srv__PlannerService_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlannerService_Response__ros_msg_type * ros_message = static_cast<const _PlannerService_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: polynomial_plan
  current_alignment += get_serialized_size_mav_planning_msgs__msg__PolynomialTrajectory(
    &(ros_message->polynomial_plan), current_alignment);

  // Field name: polynomial_plan_4d
  current_alignment += get_serialized_size_mav_planning_msgs__msg__PolynomialTrajectory4D(
    &(ros_message->polynomial_plan_4d), current_alignment);

  // Field name: sampled_plan
  current_alignment += get_serialized_size_trajectory_msgs__msg__MultiDOFJointTrajectory(
    &(ros_message->sampled_plan), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_mav_planning_msgs__srv__PlannerService_Response(
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

  // Field name: success
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: polynomial_plan
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_mav_planning_msgs__msg__PolynomialTrajectory(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: polynomial_plan_4d
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_mav_planning_msgs__msg__PolynomialTrajectory4D(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: sampled_plan
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_trajectory_msgs__msg__MultiDOFJointTrajectory(
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
    using DataType = mav_planning_msgs__srv__PlannerService_Response;
    is_plain =
      (
      offsetof(DataType, sampled_plan) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_key_mav_planning_msgs__srv__PlannerService_Response(
  const mav_planning_msgs__srv__PlannerService_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: success
  {
    cdr << (ros_message->success ? true : false);
  }

  // Field name: polynomial_plan
  {
    cdr_serialize_key_mav_planning_msgs__msg__PolynomialTrajectory(
      &ros_message->polynomial_plan, cdr);
  }

  // Field name: polynomial_plan_4d
  {
    cdr_serialize_key_mav_planning_msgs__msg__PolynomialTrajectory4D(
      &ros_message->polynomial_plan_4d, cdr);
  }

  // Field name: sampled_plan
  {
    cdr_serialize_key_trajectory_msgs__msg__MultiDOFJointTrajectory(
      &ros_message->sampled_plan, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_key_mav_planning_msgs__srv__PlannerService_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlannerService_Response__ros_msg_type * ros_message = static_cast<const _PlannerService_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: success
  {
    size_t item_size = sizeof(ros_message->success);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: polynomial_plan
  current_alignment += get_serialized_size_key_mav_planning_msgs__msg__PolynomialTrajectory(
    &(ros_message->polynomial_plan), current_alignment);

  // Field name: polynomial_plan_4d
  current_alignment += get_serialized_size_key_mav_planning_msgs__msg__PolynomialTrajectory4D(
    &(ros_message->polynomial_plan_4d), current_alignment);

  // Field name: sampled_plan
  current_alignment += get_serialized_size_key_trajectory_msgs__msg__MultiDOFJointTrajectory(
    &(ros_message->sampled_plan), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_key_mav_planning_msgs__srv__PlannerService_Response(
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
  // Field name: success
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Field name: polynomial_plan
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_mav_planning_msgs__msg__PolynomialTrajectory(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: polynomial_plan_4d
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_mav_planning_msgs__msg__PolynomialTrajectory4D(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: sampled_plan
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_trajectory_msgs__msg__MultiDOFJointTrajectory(
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
    using DataType = mav_planning_msgs__srv__PlannerService_Response;
    is_plain =
      (
      offsetof(DataType, sampled_plan) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _PlannerService_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const mav_planning_msgs__srv__PlannerService_Response * ros_message = static_cast<const mav_planning_msgs__srv__PlannerService_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_mav_planning_msgs__srv__PlannerService_Response(ros_message, cdr);
}

static bool _PlannerService_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  mav_planning_msgs__srv__PlannerService_Response * ros_message = static_cast<mav_planning_msgs__srv__PlannerService_Response *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_mav_planning_msgs__srv__PlannerService_Response(cdr, ros_message);
}

static uint32_t _PlannerService_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mav_planning_msgs__srv__PlannerService_Response(
      untyped_ros_message, 0));
}

static size_t _PlannerService_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mav_planning_msgs__srv__PlannerService_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PlannerService_Response = {
  "mav_planning_msgs::srv",
  "PlannerService_Response",
  _PlannerService_Response__cdr_serialize,
  _PlannerService_Response__cdr_deserialize,
  _PlannerService_Response__get_serialized_size,
  _PlannerService_Response__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _PlannerService_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PlannerService_Response,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Response__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Response)() {
  return &_PlannerService_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <cstddef>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "mav_planning_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__struct.h"
// already included above
// #include "mav_planning_msgs/srv/detail/planner_service__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

#include "service_msgs/msg/detail/service_event_info__functions.h"  // info

// forward declare type support functions

bool cdr_serialize_mav_planning_msgs__srv__PlannerService_Request(
  const mav_planning_msgs__srv__PlannerService_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_mav_planning_msgs__srv__PlannerService_Request(
  eprosima::fastcdr::Cdr & cdr,
  mav_planning_msgs__srv__PlannerService_Request * ros_message);

size_t get_serialized_size_mav_planning_msgs__srv__PlannerService_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_mav_planning_msgs__srv__PlannerService_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_mav_planning_msgs__srv__PlannerService_Request(
  const mav_planning_msgs__srv__PlannerService_Request * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_mav_planning_msgs__srv__PlannerService_Request(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_mav_planning_msgs__srv__PlannerService_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Request)();

bool cdr_serialize_mav_planning_msgs__srv__PlannerService_Response(
  const mav_planning_msgs__srv__PlannerService_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_mav_planning_msgs__srv__PlannerService_Response(
  eprosima::fastcdr::Cdr & cdr,
  mav_planning_msgs__srv__PlannerService_Response * ros_message);

size_t get_serialized_size_mav_planning_msgs__srv__PlannerService_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_mav_planning_msgs__srv__PlannerService_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_mav_planning_msgs__srv__PlannerService_Response(
  const mav_planning_msgs__srv__PlannerService_Response * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_mav_planning_msgs__srv__PlannerService_Response(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_mav_planning_msgs__srv__PlannerService_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Response)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_deserialize_service_msgs__msg__ServiceEventInfo(
  eprosima::fastcdr::Cdr & cdr,
  service_msgs__msg__ServiceEventInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
bool cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
  const service_msgs__msg__ServiceEventInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
size_t max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_mav_planning_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, service_msgs, msg, ServiceEventInfo)();


using _PlannerService_Event__ros_msg_type = mav_planning_msgs__srv__PlannerService_Event;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_mav_planning_msgs__srv__PlannerService_Event(
  const mav_planning_msgs__srv__PlannerService_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_mav_planning_msgs__srv__PlannerService_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_mav_planning_msgs__srv__PlannerService_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_deserialize_mav_planning_msgs__srv__PlannerService_Event(
  eprosima::fastcdr::Cdr & cdr,
  mav_planning_msgs__srv__PlannerService_Event * ros_message)
{
  // Field name: info
  {
    cdr_deserialize_service_msgs__msg__ServiceEventInfo(cdr, &ros_message->info);
  }

  // Field name: request
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->request.data) {
      mav_planning_msgs__srv__PlannerService_Request__Sequence__fini(&ros_message->request);
    }
    if (!mav_planning_msgs__srv__PlannerService_Request__Sequence__init(&ros_message->request, size)) {
      fprintf(stderr, "failed to create array for field 'request'");
      return false;
    }
    auto array_ptr = ros_message->request.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_mav_planning_msgs__srv__PlannerService_Request(cdr, &array_ptr[i]);
    }
  }

  // Field name: response
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);

    // Check there are at least 'size' remaining bytes in the CDR stream before resizing
    auto old_state = cdr.get_state();
    bool correct_size = cdr.jump(size);
    cdr.set_state(old_state);
    if (!correct_size) {
      fprintf(stderr, "sequence size exceeds remaining buffer\n");
      return false;
    }

    if (ros_message->response.data) {
      mav_planning_msgs__srv__PlannerService_Response__Sequence__fini(&ros_message->response);
    }
    if (!mav_planning_msgs__srv__PlannerService_Response__Sequence__init(&ros_message->response, size)) {
      fprintf(stderr, "failed to create array for field 'response'");
      return false;
    }
    auto array_ptr = ros_message->response.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_mav_planning_msgs__srv__PlannerService_Response(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_mav_planning_msgs__srv__PlannerService_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlannerService_Event__ros_msg_type * ros_message = static_cast<const _PlannerService_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_mav_planning_msgs__srv__PlannerService_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_mav_planning_msgs__srv__PlannerService_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_mav_planning_msgs__srv__PlannerService_Event(
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

  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_mav_planning_msgs__srv__PlannerService_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_mav_planning_msgs__srv__PlannerService_Response(
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
    using DataType = mav_planning_msgs__srv__PlannerService_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
bool cdr_serialize_key_mav_planning_msgs__srv__PlannerService_Event(
  const mav_planning_msgs__srv__PlannerService_Event * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: info
  {
    cdr_serialize_key_service_msgs__msg__ServiceEventInfo(
      &ros_message->info, cdr);
  }

  // Field name: request
  {
    size_t size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_mav_planning_msgs__srv__PlannerService_Request(
        &array_ptr[i], cdr);
    }
  }

  // Field name: response
  {
    size_t size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    if (size > 1) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_mav_planning_msgs__srv__PlannerService_Response(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t get_serialized_size_key_mav_planning_msgs__srv__PlannerService_Event(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PlannerService_Event__ros_msg_type * ros_message = static_cast<const _PlannerService_Event__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: info
  current_alignment += get_serialized_size_key_service_msgs__msg__ServiceEventInfo(
    &(ros_message->info), current_alignment);

  // Field name: request
  {
    size_t array_size = ros_message->request.size;
    auto array_ptr = ros_message->request.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_mav_planning_msgs__srv__PlannerService_Request(
        &array_ptr[index], current_alignment);
    }
  }

  // Field name: response
  {
    size_t array_size = ros_message->response.size;
    auto array_ptr = ros_message->response.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_mav_planning_msgs__srv__PlannerService_Response(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_mav_planning_msgs
size_t max_serialized_size_key_mav_planning_msgs__srv__PlannerService_Event(
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
  // Field name: info
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_service_msgs__msg__ServiceEventInfo(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: request
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_mav_planning_msgs__srv__PlannerService_Request(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: response
  {
    size_t array_size = 1;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_mav_planning_msgs__srv__PlannerService_Response(
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
    using DataType = mav_planning_msgs__srv__PlannerService_Event;
    is_plain =
      (
      offsetof(DataType, response) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _PlannerService_Event__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const mav_planning_msgs__srv__PlannerService_Event * ros_message = static_cast<const mav_planning_msgs__srv__PlannerService_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_mav_planning_msgs__srv__PlannerService_Event(ros_message, cdr);
}

static bool _PlannerService_Event__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  mav_planning_msgs__srv__PlannerService_Event * ros_message = static_cast<mav_planning_msgs__srv__PlannerService_Event *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_mav_planning_msgs__srv__PlannerService_Event(cdr, ros_message);
}

static uint32_t _PlannerService_Event__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_mav_planning_msgs__srv__PlannerService_Event(
      untyped_ros_message, 0));
}

static size_t _PlannerService_Event__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_mav_planning_msgs__srv__PlannerService_Event(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PlannerService_Event = {
  "mav_planning_msgs::srv",
  "PlannerService_Event",
  _PlannerService_Event__cdr_serialize,
  _PlannerService_Event__cdr_deserialize,
  _PlannerService_Event__get_serialized_size,
  _PlannerService_Event__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _PlannerService_Event__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PlannerService_Event,
  get_message_typesupport_handle_function,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_hash,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_description,
  &mav_planning_msgs__srv__PlannerService_Event__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Event)() {
  return &_PlannerService_Event__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "mav_planning_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "mav_planning_msgs/srv/planner_service.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t PlannerService__callbacks = {
  "mav_planning_msgs::srv",
  "PlannerService",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService_Response)(),
};

static rosidl_service_type_support_t PlannerService__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &PlannerService__callbacks,
  get_service_typesupport_handle_function,
  &_PlannerService_Request__type_support,
  &_PlannerService_Response__type_support,
  &_PlannerService_Event__type_support,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mav_planning_msgs,
    srv,
    PlannerService
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    mav_planning_msgs,
    srv,
    PlannerService
  ),
  &mav_planning_msgs__srv__PlannerService__get_type_hash,
  &mav_planning_msgs__srv__PlannerService__get_type_description,
  &mav_planning_msgs__srv__PlannerService__get_type_description_sources,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, mav_planning_msgs, srv, PlannerService)() {
  return &PlannerService__handle;
}

#if defined(__cplusplus)
}
#endif
