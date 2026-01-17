// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mav_msgs:msg/AttitudeThrust.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/attitude_thrust.h"


#ifndef MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__FUNCTIONS_H_
#define MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__FUNCTIONS_H_

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
#include "mav_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "mav_msgs/msg/detail/attitude_thrust__struct.h"

/// Initialize msg/AttitudeThrust message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mav_msgs__msg__AttitudeThrust
 * )) before or use
 * mav_msgs__msg__AttitudeThrust__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__AttitudeThrust__init(mav_msgs__msg__AttitudeThrust * msg);

/// Finalize msg/AttitudeThrust message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
void
mav_msgs__msg__AttitudeThrust__fini(mav_msgs__msg__AttitudeThrust * msg);

/// Create msg/AttitudeThrust message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mav_msgs__msg__AttitudeThrust__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
mav_msgs__msg__AttitudeThrust *
mav_msgs__msg__AttitudeThrust__create(void);

/// Destroy msg/AttitudeThrust message.
/**
 * It calls
 * mav_msgs__msg__AttitudeThrust__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
void
mav_msgs__msg__AttitudeThrust__destroy(mav_msgs__msg__AttitudeThrust * msg);

/// Check for msg/AttitudeThrust message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__AttitudeThrust__are_equal(const mav_msgs__msg__AttitudeThrust * lhs, const mav_msgs__msg__AttitudeThrust * rhs);

/// Copy a msg/AttitudeThrust message.
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
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__AttitudeThrust__copy(
  const mav_msgs__msg__AttitudeThrust * input,
  mav_msgs__msg__AttitudeThrust * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_type_hash_t *
mav_msgs__msg__AttitudeThrust__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mav_msgs__msg__AttitudeThrust__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_runtime_c__type_description__TypeSource *
mav_msgs__msg__AttitudeThrust__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_msgs__msg__AttitudeThrust__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/AttitudeThrust messages.
/**
 * It allocates the memory for the number of elements and calls
 * mav_msgs__msg__AttitudeThrust__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__AttitudeThrust__Sequence__init(mav_msgs__msg__AttitudeThrust__Sequence * array, size_t size);

/// Finalize array of msg/AttitudeThrust messages.
/**
 * It calls
 * mav_msgs__msg__AttitudeThrust__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
void
mav_msgs__msg__AttitudeThrust__Sequence__fini(mav_msgs__msg__AttitudeThrust__Sequence * array);

/// Create array of msg/AttitudeThrust messages.
/**
 * It allocates the memory for the array and calls
 * mav_msgs__msg__AttitudeThrust__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
mav_msgs__msg__AttitudeThrust__Sequence *
mav_msgs__msg__AttitudeThrust__Sequence__create(size_t size);

/// Destroy array of msg/AttitudeThrust messages.
/**
 * It calls
 * mav_msgs__msg__AttitudeThrust__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
void
mav_msgs__msg__AttitudeThrust__Sequence__destroy(mav_msgs__msg__AttitudeThrust__Sequence * array);

/// Check for msg/AttitudeThrust message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__AttitudeThrust__Sequence__are_equal(const mav_msgs__msg__AttitudeThrust__Sequence * lhs, const mav_msgs__msg__AttitudeThrust__Sequence * rhs);

/// Copy an array of msg/AttitudeThrust messages.
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
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__AttitudeThrust__Sequence__copy(
  const mav_msgs__msg__AttitudeThrust__Sequence * input,
  mav_msgs__msg__AttitudeThrust__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__ATTITUDE_THRUST__FUNCTIONS_H_
