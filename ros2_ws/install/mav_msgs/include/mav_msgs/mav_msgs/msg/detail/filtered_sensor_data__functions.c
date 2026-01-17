// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mav_msgs:msg/FilteredSensorData.idl
// generated code does not contain a copyright notice
#include "mav_msgs/msg/detail/filtered_sensor_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `accelerometer`
// Member `gyroscope`
// Member `magnetometer`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
mav_msgs__msg__FilteredSensorData__init(mav_msgs__msg__FilteredSensorData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mav_msgs__msg__FilteredSensorData__fini(msg);
    return false;
  }
  // accelerometer
  if (!geometry_msgs__msg__Vector3__init(&msg->accelerometer)) {
    mav_msgs__msg__FilteredSensorData__fini(msg);
    return false;
  }
  // gyroscope
  if (!geometry_msgs__msg__Vector3__init(&msg->gyroscope)) {
    mav_msgs__msg__FilteredSensorData__fini(msg);
    return false;
  }
  // magnetometer
  if (!geometry_msgs__msg__Vector3__init(&msg->magnetometer)) {
    mav_msgs__msg__FilteredSensorData__fini(msg);
    return false;
  }
  // barometer
  return true;
}

void
mav_msgs__msg__FilteredSensorData__fini(mav_msgs__msg__FilteredSensorData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // accelerometer
  geometry_msgs__msg__Vector3__fini(&msg->accelerometer);
  // gyroscope
  geometry_msgs__msg__Vector3__fini(&msg->gyroscope);
  // magnetometer
  geometry_msgs__msg__Vector3__fini(&msg->magnetometer);
  // barometer
}

bool
mav_msgs__msg__FilteredSensorData__are_equal(const mav_msgs__msg__FilteredSensorData * lhs, const mav_msgs__msg__FilteredSensorData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // accelerometer
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->accelerometer), &(rhs->accelerometer)))
  {
    return false;
  }
  // gyroscope
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->gyroscope), &(rhs->gyroscope)))
  {
    return false;
  }
  // magnetometer
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->magnetometer), &(rhs->magnetometer)))
  {
    return false;
  }
  // barometer
  if (lhs->barometer != rhs->barometer) {
    return false;
  }
  return true;
}

bool
mav_msgs__msg__FilteredSensorData__copy(
  const mav_msgs__msg__FilteredSensorData * input,
  mav_msgs__msg__FilteredSensorData * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // accelerometer
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->accelerometer), &(output->accelerometer)))
  {
    return false;
  }
  // gyroscope
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->gyroscope), &(output->gyroscope)))
  {
    return false;
  }
  // magnetometer
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->magnetometer), &(output->magnetometer)))
  {
    return false;
  }
  // barometer
  output->barometer = input->barometer;
  return true;
}

mav_msgs__msg__FilteredSensorData *
mav_msgs__msg__FilteredSensorData__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_msgs__msg__FilteredSensorData * msg = (mav_msgs__msg__FilteredSensorData *)allocator.allocate(sizeof(mav_msgs__msg__FilteredSensorData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mav_msgs__msg__FilteredSensorData));
  bool success = mav_msgs__msg__FilteredSensorData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mav_msgs__msg__FilteredSensorData__destroy(mav_msgs__msg__FilteredSensorData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mav_msgs__msg__FilteredSensorData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mav_msgs__msg__FilteredSensorData__Sequence__init(mav_msgs__msg__FilteredSensorData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_msgs__msg__FilteredSensorData * data = NULL;

  if (size) {
    data = (mav_msgs__msg__FilteredSensorData *)allocator.zero_allocate(size, sizeof(mav_msgs__msg__FilteredSensorData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mav_msgs__msg__FilteredSensorData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mav_msgs__msg__FilteredSensorData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
mav_msgs__msg__FilteredSensorData__Sequence__fini(mav_msgs__msg__FilteredSensorData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      mav_msgs__msg__FilteredSensorData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

mav_msgs__msg__FilteredSensorData__Sequence *
mav_msgs__msg__FilteredSensorData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_msgs__msg__FilteredSensorData__Sequence * array = (mav_msgs__msg__FilteredSensorData__Sequence *)allocator.allocate(sizeof(mav_msgs__msg__FilteredSensorData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mav_msgs__msg__FilteredSensorData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mav_msgs__msg__FilteredSensorData__Sequence__destroy(mav_msgs__msg__FilteredSensorData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mav_msgs__msg__FilteredSensorData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mav_msgs__msg__FilteredSensorData__Sequence__are_equal(const mav_msgs__msg__FilteredSensorData__Sequence * lhs, const mav_msgs__msg__FilteredSensorData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mav_msgs__msg__FilteredSensorData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mav_msgs__msg__FilteredSensorData__Sequence__copy(
  const mav_msgs__msg__FilteredSensorData__Sequence * input,
  mav_msgs__msg__FilteredSensorData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mav_msgs__msg__FilteredSensorData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mav_msgs__msg__FilteredSensorData * data =
      (mav_msgs__msg__FilteredSensorData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mav_msgs__msg__FilteredSensorData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mav_msgs__msg__FilteredSensorData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mav_msgs__msg__FilteredSensorData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
