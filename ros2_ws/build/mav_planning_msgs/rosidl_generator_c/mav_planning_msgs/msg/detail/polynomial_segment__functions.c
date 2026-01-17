// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mav_planning_msgs:msg/PolynomialSegment.idl
// generated code does not contain a copyright notice
#include "mav_planning_msgs/msg/detail/polynomial_segment__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `segment_time`
#include "builtin_interfaces/msg/detail/duration__functions.h"
// Member `x`
// Member `y`
// Member `z`
// Member `rx`
// Member `ry`
// Member `rz`
// Member `yaw`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
mav_planning_msgs__msg__PolynomialSegment__init(mav_planning_msgs__msg__PolynomialSegment * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
    return false;
  }
  // num_coeffs
  // segment_time
  if (!builtin_interfaces__msg__Duration__init(&msg->segment_time)) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->x, 0)) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->y, 0)) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__init(&msg->z, 0)) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
    return false;
  }
  // rx
  if (!rosidl_runtime_c__double__Sequence__init(&msg->rx, 0)) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
    return false;
  }
  // ry
  if (!rosidl_runtime_c__double__Sequence__init(&msg->ry, 0)) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
    return false;
  }
  // rz
  if (!rosidl_runtime_c__double__Sequence__init(&msg->rz, 0)) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
    return false;
  }
  // yaw
  if (!rosidl_runtime_c__double__Sequence__init(&msg->yaw, 0)) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
    return false;
  }
  return true;
}

void
mav_planning_msgs__msg__PolynomialSegment__fini(mav_planning_msgs__msg__PolynomialSegment * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // num_coeffs
  // segment_time
  builtin_interfaces__msg__Duration__fini(&msg->segment_time);
  // x
  rosidl_runtime_c__double__Sequence__fini(&msg->x);
  // y
  rosidl_runtime_c__double__Sequence__fini(&msg->y);
  // z
  rosidl_runtime_c__double__Sequence__fini(&msg->z);
  // rx
  rosidl_runtime_c__double__Sequence__fini(&msg->rx);
  // ry
  rosidl_runtime_c__double__Sequence__fini(&msg->ry);
  // rz
  rosidl_runtime_c__double__Sequence__fini(&msg->rz);
  // yaw
  rosidl_runtime_c__double__Sequence__fini(&msg->yaw);
}

bool
mav_planning_msgs__msg__PolynomialSegment__are_equal(const mav_planning_msgs__msg__PolynomialSegment * lhs, const mav_planning_msgs__msg__PolynomialSegment * rhs)
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
  // num_coeffs
  if (lhs->num_coeffs != rhs->num_coeffs) {
    return false;
  }
  // segment_time
  if (!builtin_interfaces__msg__Duration__are_equal(
      &(lhs->segment_time), &(rhs->segment_time)))
  {
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->x), &(rhs->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->y), &(rhs->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->z), &(rhs->z)))
  {
    return false;
  }
  // rx
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->rx), &(rhs->rx)))
  {
    return false;
  }
  // ry
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->ry), &(rhs->ry)))
  {
    return false;
  }
  // rz
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->rz), &(rhs->rz)))
  {
    return false;
  }
  // yaw
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->yaw), &(rhs->yaw)))
  {
    return false;
  }
  return true;
}

bool
mav_planning_msgs__msg__PolynomialSegment__copy(
  const mav_planning_msgs__msg__PolynomialSegment * input,
  mav_planning_msgs__msg__PolynomialSegment * output)
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
  // num_coeffs
  output->num_coeffs = input->num_coeffs;
  // segment_time
  if (!builtin_interfaces__msg__Duration__copy(
      &(input->segment_time), &(output->segment_time)))
  {
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->x), &(output->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->y), &(output->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->z), &(output->z)))
  {
    return false;
  }
  // rx
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->rx), &(output->rx)))
  {
    return false;
  }
  // ry
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->ry), &(output->ry)))
  {
    return false;
  }
  // rz
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->rz), &(output->rz)))
  {
    return false;
  }
  // yaw
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->yaw), &(output->yaw)))
  {
    return false;
  }
  return true;
}

mav_planning_msgs__msg__PolynomialSegment *
mav_planning_msgs__msg__PolynomialSegment__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__msg__PolynomialSegment * msg = (mav_planning_msgs__msg__PolynomialSegment *)allocator.allocate(sizeof(mav_planning_msgs__msg__PolynomialSegment), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mav_planning_msgs__msg__PolynomialSegment));
  bool success = mav_planning_msgs__msg__PolynomialSegment__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mav_planning_msgs__msg__PolynomialSegment__destroy(mav_planning_msgs__msg__PolynomialSegment * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mav_planning_msgs__msg__PolynomialSegment__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mav_planning_msgs__msg__PolynomialSegment__Sequence__init(mav_planning_msgs__msg__PolynomialSegment__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__msg__PolynomialSegment * data = NULL;

  if (size) {
    data = (mav_planning_msgs__msg__PolynomialSegment *)allocator.zero_allocate(size, sizeof(mav_planning_msgs__msg__PolynomialSegment), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mav_planning_msgs__msg__PolynomialSegment__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mav_planning_msgs__msg__PolynomialSegment__fini(&data[i - 1]);
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
mav_planning_msgs__msg__PolynomialSegment__Sequence__fini(mav_planning_msgs__msg__PolynomialSegment__Sequence * array)
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
      mav_planning_msgs__msg__PolynomialSegment__fini(&array->data[i]);
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

mav_planning_msgs__msg__PolynomialSegment__Sequence *
mav_planning_msgs__msg__PolynomialSegment__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__msg__PolynomialSegment__Sequence * array = (mav_planning_msgs__msg__PolynomialSegment__Sequence *)allocator.allocate(sizeof(mav_planning_msgs__msg__PolynomialSegment__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mav_planning_msgs__msg__PolynomialSegment__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mav_planning_msgs__msg__PolynomialSegment__Sequence__destroy(mav_planning_msgs__msg__PolynomialSegment__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mav_planning_msgs__msg__PolynomialSegment__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mav_planning_msgs__msg__PolynomialSegment__Sequence__are_equal(const mav_planning_msgs__msg__PolynomialSegment__Sequence * lhs, const mav_planning_msgs__msg__PolynomialSegment__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mav_planning_msgs__msg__PolynomialSegment__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mav_planning_msgs__msg__PolynomialSegment__Sequence__copy(
  const mav_planning_msgs__msg__PolynomialSegment__Sequence * input,
  mav_planning_msgs__msg__PolynomialSegment__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mav_planning_msgs__msg__PolynomialSegment);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mav_planning_msgs__msg__PolynomialSegment * data =
      (mav_planning_msgs__msg__PolynomialSegment *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mav_planning_msgs__msg__PolynomialSegment__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mav_planning_msgs__msg__PolynomialSegment__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mav_planning_msgs__msg__PolynomialSegment__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
