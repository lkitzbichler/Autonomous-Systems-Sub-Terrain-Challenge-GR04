// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mav_msgs:msg/Status.idl
// generated code does not contain a copyright notice
#include "mav_msgs/msg/detail/status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `vehicle_name`
// Member `vehicle_type`
// Member `rc_command_mode`
// Member `motor_status`
// Member `gps_status`
#include "rosidl_runtime_c/string_functions.h"

bool
mav_msgs__msg__Status__init(mav_msgs__msg__Status * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    mav_msgs__msg__Status__fini(msg);
    return false;
  }
  // vehicle_name
  if (!rosidl_runtime_c__String__init(&msg->vehicle_name)) {
    mav_msgs__msg__Status__fini(msg);
    return false;
  }
  // vehicle_type
  if (!rosidl_runtime_c__String__init(&msg->vehicle_type)) {
    mav_msgs__msg__Status__fini(msg);
    return false;
  }
  // battery_voltage
  // rc_command_mode
  if (!rosidl_runtime_c__String__init(&msg->rc_command_mode)) {
    mav_msgs__msg__Status__fini(msg);
    return false;
  }
  // command_interface_enabled
  // flight_time
  // system_uptime
  // cpu_load
  // motor_status
  if (!rosidl_runtime_c__String__init(&msg->motor_status)) {
    mav_msgs__msg__Status__fini(msg);
    return false;
  }
  // in_air
  // gps_status
  if (!rosidl_runtime_c__String__init(&msg->gps_status)) {
    mav_msgs__msg__Status__fini(msg);
    return false;
  }
  // gps_num_satellites
  return true;
}

void
mav_msgs__msg__Status__fini(mav_msgs__msg__Status * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // vehicle_name
  rosidl_runtime_c__String__fini(&msg->vehicle_name);
  // vehicle_type
  rosidl_runtime_c__String__fini(&msg->vehicle_type);
  // battery_voltage
  // rc_command_mode
  rosidl_runtime_c__String__fini(&msg->rc_command_mode);
  // command_interface_enabled
  // flight_time
  // system_uptime
  // cpu_load
  // motor_status
  rosidl_runtime_c__String__fini(&msg->motor_status);
  // in_air
  // gps_status
  rosidl_runtime_c__String__fini(&msg->gps_status);
  // gps_num_satellites
}

bool
mav_msgs__msg__Status__are_equal(const mav_msgs__msg__Status * lhs, const mav_msgs__msg__Status * rhs)
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
  // vehicle_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->vehicle_name), &(rhs->vehicle_name)))
  {
    return false;
  }
  // vehicle_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->vehicle_type), &(rhs->vehicle_type)))
  {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // rc_command_mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->rc_command_mode), &(rhs->rc_command_mode)))
  {
    return false;
  }
  // command_interface_enabled
  if (lhs->command_interface_enabled != rhs->command_interface_enabled) {
    return false;
  }
  // flight_time
  if (lhs->flight_time != rhs->flight_time) {
    return false;
  }
  // system_uptime
  if (lhs->system_uptime != rhs->system_uptime) {
    return false;
  }
  // cpu_load
  if (lhs->cpu_load != rhs->cpu_load) {
    return false;
  }
  // motor_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->motor_status), &(rhs->motor_status)))
  {
    return false;
  }
  // in_air
  if (lhs->in_air != rhs->in_air) {
    return false;
  }
  // gps_status
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->gps_status), &(rhs->gps_status)))
  {
    return false;
  }
  // gps_num_satellites
  if (lhs->gps_num_satellites != rhs->gps_num_satellites) {
    return false;
  }
  return true;
}

bool
mav_msgs__msg__Status__copy(
  const mav_msgs__msg__Status * input,
  mav_msgs__msg__Status * output)
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
  // vehicle_name
  if (!rosidl_runtime_c__String__copy(
      &(input->vehicle_name), &(output->vehicle_name)))
  {
    return false;
  }
  // vehicle_type
  if (!rosidl_runtime_c__String__copy(
      &(input->vehicle_type), &(output->vehicle_type)))
  {
    return false;
  }
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // rc_command_mode
  if (!rosidl_runtime_c__String__copy(
      &(input->rc_command_mode), &(output->rc_command_mode)))
  {
    return false;
  }
  // command_interface_enabled
  output->command_interface_enabled = input->command_interface_enabled;
  // flight_time
  output->flight_time = input->flight_time;
  // system_uptime
  output->system_uptime = input->system_uptime;
  // cpu_load
  output->cpu_load = input->cpu_load;
  // motor_status
  if (!rosidl_runtime_c__String__copy(
      &(input->motor_status), &(output->motor_status)))
  {
    return false;
  }
  // in_air
  output->in_air = input->in_air;
  // gps_status
  if (!rosidl_runtime_c__String__copy(
      &(input->gps_status), &(output->gps_status)))
  {
    return false;
  }
  // gps_num_satellites
  output->gps_num_satellites = input->gps_num_satellites;
  return true;
}

mav_msgs__msg__Status *
mav_msgs__msg__Status__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_msgs__msg__Status * msg = (mav_msgs__msg__Status *)allocator.allocate(sizeof(mav_msgs__msg__Status), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mav_msgs__msg__Status));
  bool success = mav_msgs__msg__Status__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mav_msgs__msg__Status__destroy(mav_msgs__msg__Status * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mav_msgs__msg__Status__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mav_msgs__msg__Status__Sequence__init(mav_msgs__msg__Status__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_msgs__msg__Status * data = NULL;

  if (size) {
    data = (mav_msgs__msg__Status *)allocator.zero_allocate(size, sizeof(mav_msgs__msg__Status), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mav_msgs__msg__Status__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mav_msgs__msg__Status__fini(&data[i - 1]);
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
mav_msgs__msg__Status__Sequence__fini(mav_msgs__msg__Status__Sequence * array)
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
      mav_msgs__msg__Status__fini(&array->data[i]);
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

mav_msgs__msg__Status__Sequence *
mav_msgs__msg__Status__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_msgs__msg__Status__Sequence * array = (mav_msgs__msg__Status__Sequence *)allocator.allocate(sizeof(mav_msgs__msg__Status__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mav_msgs__msg__Status__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mav_msgs__msg__Status__Sequence__destroy(mav_msgs__msg__Status__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mav_msgs__msg__Status__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mav_msgs__msg__Status__Sequence__are_equal(const mav_msgs__msg__Status__Sequence * lhs, const mav_msgs__msg__Status__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mav_msgs__msg__Status__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mav_msgs__msg__Status__Sequence__copy(
  const mav_msgs__msg__Status__Sequence * input,
  mav_msgs__msg__Status__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mav_msgs__msg__Status);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mav_msgs__msg__Status * data =
      (mav_msgs__msg__Status *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mav_msgs__msg__Status__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mav_msgs__msg__Status__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mav_msgs__msg__Status__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
