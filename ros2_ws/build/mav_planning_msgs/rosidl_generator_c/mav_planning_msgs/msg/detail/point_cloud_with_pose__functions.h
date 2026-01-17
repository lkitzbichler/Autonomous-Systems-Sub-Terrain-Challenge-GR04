// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mav_planning_msgs:msg/PointCloudWithPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/point_cloud_with_pose.h"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__FUNCTIONS_H_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "mav_planning_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mav_planning_msgs/msg/detail/point_cloud_with_pose__struct.h"

/// Initialize msg/PointCloudWithPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mav_planning_msgs__msg__PointCloudWithPose
 * )) before or use
 * mav_planning_msgs__msg__PointCloudWithPose__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
bool
mav_planning_msgs__msg__PointCloudWithPose__init(mav_planning_msgs__msg__PointCloudWithPose * msg);

/// Finalize msg/PointCloudWithPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
void
mav_planning_msgs__msg__PointCloudWithPose__fini(mav_planning_msgs__msg__PointCloudWithPose * msg);

/// Create msg/PointCloudWithPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mav_planning_msgs__msg__PointCloudWithPose__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
mav_planning_msgs__msg__PointCloudWithPose *
mav_planning_msgs__msg__PointCloudWithPose__create(void);

/// Destroy msg/PointCloudWithPose message.
/**
 * It calls
 * mav_planning_msgs__msg__PointCloudWithPose__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
void
mav_planning_msgs__msg__PointCloudWithPose__destroy(mav_planning_msgs__msg__PointCloudWithPose * msg);

/// Check for msg/PointCloudWithPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
bool
mav_planning_msgs__msg__PointCloudWithPose__are_equal(const mav_planning_msgs__msg__PointCloudWithPose * lhs, const mav_planning_msgs__msg__PointCloudWithPose * rhs);

/// Copy a msg/PointCloudWithPose message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
bool
mav_planning_msgs__msg__PointCloudWithPose__copy(
  const mav_planning_msgs__msg__PointCloudWithPose * input,
  mav_planning_msgs__msg__PointCloudWithPose * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_type_hash_t *
mav_planning_msgs__msg__PointCloudWithPose__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mav_planning_msgs__msg__PointCloudWithPose__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_runtime_c__type_description__TypeSource *
mav_planning_msgs__msg__PointCloudWithPose__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_planning_msgs__msg__PointCloudWithPose__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/PointCloudWithPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * mav_planning_msgs__msg__PointCloudWithPose__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
bool
mav_planning_msgs__msg__PointCloudWithPose__Sequence__init(mav_planning_msgs__msg__PointCloudWithPose__Sequence * array, size_t size);

/// Finalize array of msg/PointCloudWithPose messages.
/**
 * It calls
 * mav_planning_msgs__msg__PointCloudWithPose__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
void
mav_planning_msgs__msg__PointCloudWithPose__Sequence__fini(mav_planning_msgs__msg__PointCloudWithPose__Sequence * array);

/// Create array of msg/PointCloudWithPose messages.
/**
 * It allocates the memory for the array and calls
 * mav_planning_msgs__msg__PointCloudWithPose__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
mav_planning_msgs__msg__PointCloudWithPose__Sequence *
mav_planning_msgs__msg__PointCloudWithPose__Sequence__create(size_t size);

/// Destroy array of msg/PointCloudWithPose messages.
/**
 * It calls
 * mav_planning_msgs__msg__PointCloudWithPose__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
void
mav_planning_msgs__msg__PointCloudWithPose__Sequence__destroy(mav_planning_msgs__msg__PointCloudWithPose__Sequence * array);

/// Check for msg/PointCloudWithPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
bool
mav_planning_msgs__msg__PointCloudWithPose__Sequence__are_equal(const mav_planning_msgs__msg__PointCloudWithPose__Sequence * lhs, const mav_planning_msgs__msg__PointCloudWithPose__Sequence * rhs);

/// Copy an array of msg/PointCloudWithPose messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_planning_msgs
bool
mav_planning_msgs__msg__PointCloudWithPose__Sequence__copy(
  const mav_planning_msgs__msg__PointCloudWithPose__Sequence * input,
  mav_planning_msgs__msg__PointCloudWithPose__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POINT_CLOUD_WITH_POSE__FUNCTIONS_H_
