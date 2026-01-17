// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from mav_msgs:msg/FilteredSensorData.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_msgs/msg/filtered_sensor_data.h"


#ifndef MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__FUNCTIONS_H_
#define MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__FUNCTIONS_H_

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

#include "mav_msgs/msg/detail/filtered_sensor_data__struct.h"

/// Initialize msg/FilteredSensorData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * mav_msgs__msg__FilteredSensorData
 * )) before or use
 * mav_msgs__msg__FilteredSensorData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__FilteredSensorData__init(mav_msgs__msg__FilteredSensorData * msg);

/// Finalize msg/FilteredSensorData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
void
mav_msgs__msg__FilteredSensorData__fini(mav_msgs__msg__FilteredSensorData * msg);

/// Create msg/FilteredSensorData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * mav_msgs__msg__FilteredSensorData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
mav_msgs__msg__FilteredSensorData *
mav_msgs__msg__FilteredSensorData__create(void);

/// Destroy msg/FilteredSensorData message.
/**
 * It calls
 * mav_msgs__msg__FilteredSensorData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
void
mav_msgs__msg__FilteredSensorData__destroy(mav_msgs__msg__FilteredSensorData * msg);

/// Check for msg/FilteredSensorData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__FilteredSensorData__are_equal(const mav_msgs__msg__FilteredSensorData * lhs, const mav_msgs__msg__FilteredSensorData * rhs);

/// Copy a msg/FilteredSensorData message.
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
mav_msgs__msg__FilteredSensorData__copy(
  const mav_msgs__msg__FilteredSensorData * input,
  mav_msgs__msg__FilteredSensorData * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_type_hash_t *
mav_msgs__msg__FilteredSensorData__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_runtime_c__type_description__TypeDescription *
mav_msgs__msg__FilteredSensorData__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_runtime_c__type_description__TypeSource *
mav_msgs__msg__FilteredSensorData__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
mav_msgs__msg__FilteredSensorData__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/FilteredSensorData messages.
/**
 * It allocates the memory for the number of elements and calls
 * mav_msgs__msg__FilteredSensorData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__FilteredSensorData__Sequence__init(mav_msgs__msg__FilteredSensorData__Sequence * array, size_t size);

/// Finalize array of msg/FilteredSensorData messages.
/**
 * It calls
 * mav_msgs__msg__FilteredSensorData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
void
mav_msgs__msg__FilteredSensorData__Sequence__fini(mav_msgs__msg__FilteredSensorData__Sequence * array);

/// Create array of msg/FilteredSensorData messages.
/**
 * It allocates the memory for the array and calls
 * mav_msgs__msg__FilteredSensorData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
mav_msgs__msg__FilteredSensorData__Sequence *
mav_msgs__msg__FilteredSensorData__Sequence__create(size_t size);

/// Destroy array of msg/FilteredSensorData messages.
/**
 * It calls
 * mav_msgs__msg__FilteredSensorData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
void
mav_msgs__msg__FilteredSensorData__Sequence__destroy(mav_msgs__msg__FilteredSensorData__Sequence * array);

/// Check for msg/FilteredSensorData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_mav_msgs
bool
mav_msgs__msg__FilteredSensorData__Sequence__are_equal(const mav_msgs__msg__FilteredSensorData__Sequence * lhs, const mav_msgs__msg__FilteredSensorData__Sequence * rhs);

/// Copy an array of msg/FilteredSensorData messages.
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
mav_msgs__msg__FilteredSensorData__Sequence__copy(
  const mav_msgs__msg__FilteredSensorData__Sequence * input,
  mav_msgs__msg__FilteredSensorData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MAV_MSGS__MSG__DETAIL__FILTERED_SENSOR_DATA__FUNCTIONS_H_
