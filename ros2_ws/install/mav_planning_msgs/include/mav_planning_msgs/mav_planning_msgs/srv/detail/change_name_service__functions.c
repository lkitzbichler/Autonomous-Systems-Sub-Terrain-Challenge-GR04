// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from mav_planning_msgs:srv/ChangeNameService.idl
// generated code does not contain a copyright notice
#include "mav_planning_msgs/srv/detail/change_name_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
mav_planning_msgs__srv__ChangeNameService_Request__init(mav_planning_msgs__srv__ChangeNameService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    mav_planning_msgs__srv__ChangeNameService_Request__fini(msg);
    return false;
  }
  return true;
}

void
mav_planning_msgs__srv__ChangeNameService_Request__fini(mav_planning_msgs__srv__ChangeNameService_Request * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
}

bool
mav_planning_msgs__srv__ChangeNameService_Request__are_equal(const mav_planning_msgs__srv__ChangeNameService_Request * lhs, const mav_planning_msgs__srv__ChangeNameService_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  return true;
}

bool
mav_planning_msgs__srv__ChangeNameService_Request__copy(
  const mav_planning_msgs__srv__ChangeNameService_Request * input,
  mav_planning_msgs__srv__ChangeNameService_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  return true;
}

mav_planning_msgs__srv__ChangeNameService_Request *
mav_planning_msgs__srv__ChangeNameService_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__srv__ChangeNameService_Request * msg = (mav_planning_msgs__srv__ChangeNameService_Request *)allocator.allocate(sizeof(mav_planning_msgs__srv__ChangeNameService_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mav_planning_msgs__srv__ChangeNameService_Request));
  bool success = mav_planning_msgs__srv__ChangeNameService_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mav_planning_msgs__srv__ChangeNameService_Request__destroy(mav_planning_msgs__srv__ChangeNameService_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mav_planning_msgs__srv__ChangeNameService_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mav_planning_msgs__srv__ChangeNameService_Request__Sequence__init(mav_planning_msgs__srv__ChangeNameService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__srv__ChangeNameService_Request * data = NULL;

  if (size) {
    data = (mav_planning_msgs__srv__ChangeNameService_Request *)allocator.zero_allocate(size, sizeof(mav_planning_msgs__srv__ChangeNameService_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mav_planning_msgs__srv__ChangeNameService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mav_planning_msgs__srv__ChangeNameService_Request__fini(&data[i - 1]);
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
mav_planning_msgs__srv__ChangeNameService_Request__Sequence__fini(mav_planning_msgs__srv__ChangeNameService_Request__Sequence * array)
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
      mav_planning_msgs__srv__ChangeNameService_Request__fini(&array->data[i]);
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

mav_planning_msgs__srv__ChangeNameService_Request__Sequence *
mav_planning_msgs__srv__ChangeNameService_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__srv__ChangeNameService_Request__Sequence * array = (mav_planning_msgs__srv__ChangeNameService_Request__Sequence *)allocator.allocate(sizeof(mav_planning_msgs__srv__ChangeNameService_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mav_planning_msgs__srv__ChangeNameService_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mav_planning_msgs__srv__ChangeNameService_Request__Sequence__destroy(mav_planning_msgs__srv__ChangeNameService_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mav_planning_msgs__srv__ChangeNameService_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mav_planning_msgs__srv__ChangeNameService_Request__Sequence__are_equal(const mav_planning_msgs__srv__ChangeNameService_Request__Sequence * lhs, const mav_planning_msgs__srv__ChangeNameService_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mav_planning_msgs__srv__ChangeNameService_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mav_planning_msgs__srv__ChangeNameService_Request__Sequence__copy(
  const mav_planning_msgs__srv__ChangeNameService_Request__Sequence * input,
  mav_planning_msgs__srv__ChangeNameService_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mav_planning_msgs__srv__ChangeNameService_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mav_planning_msgs__srv__ChangeNameService_Request * data =
      (mav_planning_msgs__srv__ChangeNameService_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mav_planning_msgs__srv__ChangeNameService_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mav_planning_msgs__srv__ChangeNameService_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mav_planning_msgs__srv__ChangeNameService_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
mav_planning_msgs__srv__ChangeNameService_Response__init(mav_planning_msgs__srv__ChangeNameService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    mav_planning_msgs__srv__ChangeNameService_Response__fini(msg);
    return false;
  }
  return true;
}

void
mav_planning_msgs__srv__ChangeNameService_Response__fini(mav_planning_msgs__srv__ChangeNameService_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
mav_planning_msgs__srv__ChangeNameService_Response__are_equal(const mav_planning_msgs__srv__ChangeNameService_Response * lhs, const mav_planning_msgs__srv__ChangeNameService_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
mav_planning_msgs__srv__ChangeNameService_Response__copy(
  const mav_planning_msgs__srv__ChangeNameService_Response * input,
  mav_planning_msgs__srv__ChangeNameService_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

mav_planning_msgs__srv__ChangeNameService_Response *
mav_planning_msgs__srv__ChangeNameService_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__srv__ChangeNameService_Response * msg = (mav_planning_msgs__srv__ChangeNameService_Response *)allocator.allocate(sizeof(mav_planning_msgs__srv__ChangeNameService_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mav_planning_msgs__srv__ChangeNameService_Response));
  bool success = mav_planning_msgs__srv__ChangeNameService_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mav_planning_msgs__srv__ChangeNameService_Response__destroy(mav_planning_msgs__srv__ChangeNameService_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mav_planning_msgs__srv__ChangeNameService_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mav_planning_msgs__srv__ChangeNameService_Response__Sequence__init(mav_planning_msgs__srv__ChangeNameService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__srv__ChangeNameService_Response * data = NULL;

  if (size) {
    data = (mav_planning_msgs__srv__ChangeNameService_Response *)allocator.zero_allocate(size, sizeof(mav_planning_msgs__srv__ChangeNameService_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mav_planning_msgs__srv__ChangeNameService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mav_planning_msgs__srv__ChangeNameService_Response__fini(&data[i - 1]);
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
mav_planning_msgs__srv__ChangeNameService_Response__Sequence__fini(mav_planning_msgs__srv__ChangeNameService_Response__Sequence * array)
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
      mav_planning_msgs__srv__ChangeNameService_Response__fini(&array->data[i]);
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

mav_planning_msgs__srv__ChangeNameService_Response__Sequence *
mav_planning_msgs__srv__ChangeNameService_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__srv__ChangeNameService_Response__Sequence * array = (mav_planning_msgs__srv__ChangeNameService_Response__Sequence *)allocator.allocate(sizeof(mav_planning_msgs__srv__ChangeNameService_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mav_planning_msgs__srv__ChangeNameService_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mav_planning_msgs__srv__ChangeNameService_Response__Sequence__destroy(mav_planning_msgs__srv__ChangeNameService_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mav_planning_msgs__srv__ChangeNameService_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mav_planning_msgs__srv__ChangeNameService_Response__Sequence__are_equal(const mav_planning_msgs__srv__ChangeNameService_Response__Sequence * lhs, const mav_planning_msgs__srv__ChangeNameService_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mav_planning_msgs__srv__ChangeNameService_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mav_planning_msgs__srv__ChangeNameService_Response__Sequence__copy(
  const mav_planning_msgs__srv__ChangeNameService_Response__Sequence * input,
  mav_planning_msgs__srv__ChangeNameService_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mav_planning_msgs__srv__ChangeNameService_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mav_planning_msgs__srv__ChangeNameService_Response * data =
      (mav_planning_msgs__srv__ChangeNameService_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mav_planning_msgs__srv__ChangeNameService_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mav_planning_msgs__srv__ChangeNameService_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mav_planning_msgs__srv__ChangeNameService_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "mav_planning_msgs/srv/detail/change_name_service__functions.h"

bool
mav_planning_msgs__srv__ChangeNameService_Event__init(mav_planning_msgs__srv__ChangeNameService_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    mav_planning_msgs__srv__ChangeNameService_Event__fini(msg);
    return false;
  }
  // request
  if (!mav_planning_msgs__srv__ChangeNameService_Request__Sequence__init(&msg->request, 0)) {
    mav_planning_msgs__srv__ChangeNameService_Event__fini(msg);
    return false;
  }
  // response
  if (!mav_planning_msgs__srv__ChangeNameService_Response__Sequence__init(&msg->response, 0)) {
    mav_planning_msgs__srv__ChangeNameService_Event__fini(msg);
    return false;
  }
  return true;
}

void
mav_planning_msgs__srv__ChangeNameService_Event__fini(mav_planning_msgs__srv__ChangeNameService_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  mav_planning_msgs__srv__ChangeNameService_Request__Sequence__fini(&msg->request);
  // response
  mav_planning_msgs__srv__ChangeNameService_Response__Sequence__fini(&msg->response);
}

bool
mav_planning_msgs__srv__ChangeNameService_Event__are_equal(const mav_planning_msgs__srv__ChangeNameService_Event * lhs, const mav_planning_msgs__srv__ChangeNameService_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!mav_planning_msgs__srv__ChangeNameService_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!mav_planning_msgs__srv__ChangeNameService_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
mav_planning_msgs__srv__ChangeNameService_Event__copy(
  const mav_planning_msgs__srv__ChangeNameService_Event * input,
  mav_planning_msgs__srv__ChangeNameService_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!mav_planning_msgs__srv__ChangeNameService_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!mav_planning_msgs__srv__ChangeNameService_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

mav_planning_msgs__srv__ChangeNameService_Event *
mav_planning_msgs__srv__ChangeNameService_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__srv__ChangeNameService_Event * msg = (mav_planning_msgs__srv__ChangeNameService_Event *)allocator.allocate(sizeof(mav_planning_msgs__srv__ChangeNameService_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(mav_planning_msgs__srv__ChangeNameService_Event));
  bool success = mav_planning_msgs__srv__ChangeNameService_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
mav_planning_msgs__srv__ChangeNameService_Event__destroy(mav_planning_msgs__srv__ChangeNameService_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    mav_planning_msgs__srv__ChangeNameService_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
mav_planning_msgs__srv__ChangeNameService_Event__Sequence__init(mav_planning_msgs__srv__ChangeNameService_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__srv__ChangeNameService_Event * data = NULL;

  if (size) {
    data = (mav_planning_msgs__srv__ChangeNameService_Event *)allocator.zero_allocate(size, sizeof(mav_planning_msgs__srv__ChangeNameService_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = mav_planning_msgs__srv__ChangeNameService_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        mav_planning_msgs__srv__ChangeNameService_Event__fini(&data[i - 1]);
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
mav_planning_msgs__srv__ChangeNameService_Event__Sequence__fini(mav_planning_msgs__srv__ChangeNameService_Event__Sequence * array)
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
      mav_planning_msgs__srv__ChangeNameService_Event__fini(&array->data[i]);
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

mav_planning_msgs__srv__ChangeNameService_Event__Sequence *
mav_planning_msgs__srv__ChangeNameService_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  mav_planning_msgs__srv__ChangeNameService_Event__Sequence * array = (mav_planning_msgs__srv__ChangeNameService_Event__Sequence *)allocator.allocate(sizeof(mav_planning_msgs__srv__ChangeNameService_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = mav_planning_msgs__srv__ChangeNameService_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
mav_planning_msgs__srv__ChangeNameService_Event__Sequence__destroy(mav_planning_msgs__srv__ChangeNameService_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    mav_planning_msgs__srv__ChangeNameService_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
mav_planning_msgs__srv__ChangeNameService_Event__Sequence__are_equal(const mav_planning_msgs__srv__ChangeNameService_Event__Sequence * lhs, const mav_planning_msgs__srv__ChangeNameService_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!mav_planning_msgs__srv__ChangeNameService_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
mav_planning_msgs__srv__ChangeNameService_Event__Sequence__copy(
  const mav_planning_msgs__srv__ChangeNameService_Event__Sequence * input,
  mav_planning_msgs__srv__ChangeNameService_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(mav_planning_msgs__srv__ChangeNameService_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    mav_planning_msgs__srv__ChangeNameService_Event * data =
      (mav_planning_msgs__srv__ChangeNameService_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!mav_planning_msgs__srv__ChangeNameService_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          mav_planning_msgs__srv__ChangeNameService_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!mav_planning_msgs__srv__ChangeNameService_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
