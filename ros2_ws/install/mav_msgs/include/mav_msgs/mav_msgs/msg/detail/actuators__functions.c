// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mav_msgs:msg/Actuators.idl
// generated code does not contain a copyright notice
#include "mav_msgs/msg/detail/actuators__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `angles`
// Member `angular_velocities`
// Member `normalized`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mav_msgs__msg__Actuators__init(mav_msgs__msg__Actuators * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mav_msgs__msg__Actuators__fini(msg);
    return false;
  }
  // angles
  if (!rosidl_runtime_c__double__Sequence__init(&msg->angles, 0)) {
    mav_msgs__msg__Actuators__fini(msg);
    return false;
  }
  // angular_velocities
  if (!rosidl_runtime_c__double__Sequence__init(&msg->angular_velocities, 0)) {
    mav_msgs__msg__Actuators__fini(msg);
    return false;
  }
  // normalized
  if (!rosidl_runtime_c__double__Sequence__init(&msg->normalized, 0)) {
    mav_msgs__msg__Actuators__fini(msg);
    return false;
  }
  return true;
}

void
mav_msgs__msg__Actuators__fini(mav_msgs__msg__Actuators * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // angles
  rosidl_runtime_c__double__Sequence__fini(&msg->angles);
  // angular_velocities
  rosidl_runtime_c__double__Sequence__fini(&msg->angular_velocities);
  // normalized
  rosidl_runtime_c__double__Sequence__fini(&msg->normalized);
}

bool
mav_msgs__msg__Actuators__are_equal(const mav_msgs__msg__Actuators * lhs, const mav_msgs__msg__Actuators * rhs)
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
  // angles
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->angles), &(rhs->angles)))
  {
    return false;
  }
  // angular_velocities
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->angular_velocities), &(rhs->angular_velocities)))
  {
    return false;
  }
  // normalized
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->normalized), &(rhs->normalized)))
  {
    return false;
  }
  return true;
}

bool
mav_msgs__msg__Actuators__copy(
  const mav_msgs__msg__Actuators * input,
  mav_msgs__msg__Actuators * output)
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
  // angles
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->angles), &(output->angles)))
  {
    return false;
  }
  // angular_velocities
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->angular_velocities), &(output->angular_velocities)))
  {
    return false;
  }
  // normalized
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->normalized), &(output->normalized)))
  {
    return false;
  }
  return true;
}

mav_msgs__msg__Actuators *
mav_msgs__msg__Actuators__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_msgs__msg__Actuators * msg = (mav_msgs__msg__Actuators *)allocator.allocate(sizeof(mav_msgs__msg__Actuators), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mav_msgs__msg__Actuators));
  bool success = mav_msgs__msg__Actuators__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mav_msgs__msg__Actuators__destroy(mav_msgs__msg__Actuators * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mav_msgs__msg__Actuators__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mav_msgs__msg__Actuators__Sequence__init(mav_msgs__msg__Actuators__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_msgs__msg__Actuators * data = NULL;

  if (size) {
    data = (mav_msgs__msg__Actuators *)allocator.zero_allocate(size, sizeof(mav_msgs__msg__Actuators), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mav_msgs__msg__Actuators__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mav_msgs__msg__Actuators__fini(&data[i - 1]);
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
mav_msgs__msg__Actuators__Sequence__fini(mav_msgs__msg__Actuators__Sequence * array)
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
      mav_msgs__msg__Actuators__fini(&array->data[i]);
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

mav_msgs__msg__Actuators__Sequence *
mav_msgs__msg__Actuators__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_msgs__msg__Actuators__Sequence * array = (mav_msgs__msg__Actuators__Sequence *)allocator.allocate(sizeof(mav_msgs__msg__Actuators__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mav_msgs__msg__Actuators__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mav_msgs__msg__Actuators__Sequence__destroy(mav_msgs__msg__Actuators__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mav_msgs__msg__Actuators__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mav_msgs__msg__Actuators__Sequence__are_equal(const mav_msgs__msg__Actuators__Sequence * lhs, const mav_msgs__msg__Actuators__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mav_msgs__msg__Actuators__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mav_msgs__msg__Actuators__Sequence__copy(
  const mav_msgs__msg__Actuators__Sequence * input,
  mav_msgs__msg__Actuators__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mav_msgs__msg__Actuators);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mav_msgs__msg__Actuators * data =
      (mav_msgs__msg__Actuators *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mav_msgs__msg__Actuators__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mav_msgs__msg__Actuators__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mav_msgs__msg__Actuators__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
