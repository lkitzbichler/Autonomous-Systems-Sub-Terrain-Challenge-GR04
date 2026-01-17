// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mav_planning_msgs:msg/PointCloudWithPose.idl
// generated code does not contain a copyright notice
#include "mav_planning_msgs/msg/detail/point_cloud_with_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `sensor_pose`
#include "geometry_msgs/msg/detail/transform_stamped__functions.h"
// Member `cloud_in_sensor_frame`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
mav_planning_msgs__msg__PointCloudWithPose__init(mav_planning_msgs__msg__PointCloudWithPose * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mav_planning_msgs__msg__PointCloudWithPose__fini(msg);
    return false;
  }
  // sensor_pose
  if (!geometry_msgs__msg__TransformStamped__init(&msg->sensor_pose)) {
    mav_planning_msgs__msg__PointCloudWithPose__fini(msg);
    return false;
  }
  // cloud_in_sensor_frame
  if (!sensor_msgs__msg__PointCloud2__init(&msg->cloud_in_sensor_frame)) {
    mav_planning_msgs__msg__PointCloudWithPose__fini(msg);
    return false;
  }
  return true;
}

void
mav_planning_msgs__msg__PointCloudWithPose__fini(mav_planning_msgs__msg__PointCloudWithPose * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // sensor_pose
  geometry_msgs__msg__TransformStamped__fini(&msg->sensor_pose);
  // cloud_in_sensor_frame
  sensor_msgs__msg__PointCloud2__fini(&msg->cloud_in_sensor_frame);
}

bool
mav_planning_msgs__msg__PointCloudWithPose__are_equal(const mav_planning_msgs__msg__PointCloudWithPose * lhs, const mav_planning_msgs__msg__PointCloudWithPose * rhs)
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
  // sensor_pose
  if (!geometry_msgs__msg__TransformStamped__are_equal(
      &(lhs->sensor_pose), &(rhs->sensor_pose)))
  {
    return false;
  }
  // cloud_in_sensor_frame
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->cloud_in_sensor_frame), &(rhs->cloud_in_sensor_frame)))
  {
    return false;
  }
  return true;
}

bool
mav_planning_msgs__msg__PointCloudWithPose__copy(
  const mav_planning_msgs__msg__PointCloudWithPose * input,
  mav_planning_msgs__msg__PointCloudWithPose * output)
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
  // sensor_pose
  if (!geometry_msgs__msg__TransformStamped__copy(
      &(input->sensor_pose), &(output->sensor_pose)))
  {
    return false;
  }
  // cloud_in_sensor_frame
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->cloud_in_sensor_frame), &(output->cloud_in_sensor_frame)))
  {
    return false;
  }
  return true;
}

mav_planning_msgs__msg__PointCloudWithPose *
mav_planning_msgs__msg__PointCloudWithPose__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__msg__PointCloudWithPose * msg = (mav_planning_msgs__msg__PointCloudWithPose *)allocator.allocate(sizeof(mav_planning_msgs__msg__PointCloudWithPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mav_planning_msgs__msg__PointCloudWithPose));
  bool success = mav_planning_msgs__msg__PointCloudWithPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mav_planning_msgs__msg__PointCloudWithPose__destroy(mav_planning_msgs__msg__PointCloudWithPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mav_planning_msgs__msg__PointCloudWithPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mav_planning_msgs__msg__PointCloudWithPose__Sequence__init(mav_planning_msgs__msg__PointCloudWithPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__msg__PointCloudWithPose * data = NULL;

  if (size) {
    data = (mav_planning_msgs__msg__PointCloudWithPose *)allocator.zero_allocate(size, sizeof(mav_planning_msgs__msg__PointCloudWithPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mav_planning_msgs__msg__PointCloudWithPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mav_planning_msgs__msg__PointCloudWithPose__fini(&data[i - 1]);
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
mav_planning_msgs__msg__PointCloudWithPose__Sequence__fini(mav_planning_msgs__msg__PointCloudWithPose__Sequence * array)
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
      mav_planning_msgs__msg__PointCloudWithPose__fini(&array->data[i]);
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

mav_planning_msgs__msg__PointCloudWithPose__Sequence *
mav_planning_msgs__msg__PointCloudWithPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__msg__PointCloudWithPose__Sequence * array = (mav_planning_msgs__msg__PointCloudWithPose__Sequence *)allocator.allocate(sizeof(mav_planning_msgs__msg__PointCloudWithPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mav_planning_msgs__msg__PointCloudWithPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mav_planning_msgs__msg__PointCloudWithPose__Sequence__destroy(mav_planning_msgs__msg__PointCloudWithPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mav_planning_msgs__msg__PointCloudWithPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mav_planning_msgs__msg__PointCloudWithPose__Sequence__are_equal(const mav_planning_msgs__msg__PointCloudWithPose__Sequence * lhs, const mav_planning_msgs__msg__PointCloudWithPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mav_planning_msgs__msg__PointCloudWithPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mav_planning_msgs__msg__PointCloudWithPose__Sequence__copy(
  const mav_planning_msgs__msg__PointCloudWithPose__Sequence * input,
  mav_planning_msgs__msg__PointCloudWithPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mav_planning_msgs__msg__PointCloudWithPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mav_planning_msgs__msg__PointCloudWithPose * data =
      (mav_planning_msgs__msg__PointCloudWithPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mav_planning_msgs__msg__PointCloudWithPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mav_planning_msgs__msg__PointCloudWithPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mav_planning_msgs__msg__PointCloudWithPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
