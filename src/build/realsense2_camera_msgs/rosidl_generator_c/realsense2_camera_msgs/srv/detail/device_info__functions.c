// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from realsense2_camera_msgs:srv/DeviceInfo.idl
// generated code does not contain a copyright notice
#include "realsense2_camera_msgs/srv/detail/device_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
realsense2_camera_msgs__srv__DeviceInfo_Request__init(realsense2_camera_msgs__srv__DeviceInfo_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
realsense2_camera_msgs__srv__DeviceInfo_Request__fini(realsense2_camera_msgs__srv__DeviceInfo_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
realsense2_camera_msgs__srv__DeviceInfo_Request__are_equal(const realsense2_camera_msgs__srv__DeviceInfo_Request * lhs, const realsense2_camera_msgs__srv__DeviceInfo_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
realsense2_camera_msgs__srv__DeviceInfo_Request__copy(
  const realsense2_camera_msgs__srv__DeviceInfo_Request * input,
  realsense2_camera_msgs__srv__DeviceInfo_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

realsense2_camera_msgs__srv__DeviceInfo_Request *
realsense2_camera_msgs__srv__DeviceInfo_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  realsense2_camera_msgs__srv__DeviceInfo_Request * msg = (realsense2_camera_msgs__srv__DeviceInfo_Request *)allocator.allocate(sizeof(realsense2_camera_msgs__srv__DeviceInfo_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(realsense2_camera_msgs__srv__DeviceInfo_Request));
  bool success = realsense2_camera_msgs__srv__DeviceInfo_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
realsense2_camera_msgs__srv__DeviceInfo_Request__destroy(realsense2_camera_msgs__srv__DeviceInfo_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    realsense2_camera_msgs__srv__DeviceInfo_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence__init(realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  realsense2_camera_msgs__srv__DeviceInfo_Request * data = NULL;

  if (size) {
    data = (realsense2_camera_msgs__srv__DeviceInfo_Request *)allocator.zero_allocate(size, sizeof(realsense2_camera_msgs__srv__DeviceInfo_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = realsense2_camera_msgs__srv__DeviceInfo_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        realsense2_camera_msgs__srv__DeviceInfo_Request__fini(&data[i - 1]);
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
realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence__fini(realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence * array)
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
      realsense2_camera_msgs__srv__DeviceInfo_Request__fini(&array->data[i]);
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

realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence *
realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence * array = (realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence *)allocator.allocate(sizeof(realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence__destroy(realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence__are_equal(const realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence * lhs, const realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!realsense2_camera_msgs__srv__DeviceInfo_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence__copy(
  const realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence * input,
  realsense2_camera_msgs__srv__DeviceInfo_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(realsense2_camera_msgs__srv__DeviceInfo_Request);
    realsense2_camera_msgs__srv__DeviceInfo_Request * data =
      (realsense2_camera_msgs__srv__DeviceInfo_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!realsense2_camera_msgs__srv__DeviceInfo_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          realsense2_camera_msgs__srv__DeviceInfo_Request__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!realsense2_camera_msgs__srv__DeviceInfo_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `device_name`
// Member `serial_number`
// Member `firmware_version`
// Member `usb_type_descriptor`
// Member `firmware_update_id`
// Member `sensors`
// Member `physical_port`
#include "rosidl_runtime_c/string_functions.h"

bool
realsense2_camera_msgs__srv__DeviceInfo_Response__init(realsense2_camera_msgs__srv__DeviceInfo_Response * msg)
{
  if (!msg) {
    return false;
  }
  // device_name
  if (!rosidl_runtime_c__String__init(&msg->device_name)) {
    realsense2_camera_msgs__srv__DeviceInfo_Response__fini(msg);
    return false;
  }
  // serial_number
  if (!rosidl_runtime_c__String__init(&msg->serial_number)) {
    realsense2_camera_msgs__srv__DeviceInfo_Response__fini(msg);
    return false;
  }
  // firmware_version
  if (!rosidl_runtime_c__String__init(&msg->firmware_version)) {
    realsense2_camera_msgs__srv__DeviceInfo_Response__fini(msg);
    return false;
  }
  // usb_type_descriptor
  if (!rosidl_runtime_c__String__init(&msg->usb_type_descriptor)) {
    realsense2_camera_msgs__srv__DeviceInfo_Response__fini(msg);
    return false;
  }
  // firmware_update_id
  if (!rosidl_runtime_c__String__init(&msg->firmware_update_id)) {
    realsense2_camera_msgs__srv__DeviceInfo_Response__fini(msg);
    return false;
  }
  // sensors
  if (!rosidl_runtime_c__String__init(&msg->sensors)) {
    realsense2_camera_msgs__srv__DeviceInfo_Response__fini(msg);
    return false;
  }
  // physical_port
  if (!rosidl_runtime_c__String__init(&msg->physical_port)) {
    realsense2_camera_msgs__srv__DeviceInfo_Response__fini(msg);
    return false;
  }
  return true;
}

void
realsense2_camera_msgs__srv__DeviceInfo_Response__fini(realsense2_camera_msgs__srv__DeviceInfo_Response * msg)
{
  if (!msg) {
    return;
  }
  // device_name
  rosidl_runtime_c__String__fini(&msg->device_name);
  // serial_number
  rosidl_runtime_c__String__fini(&msg->serial_number);
  // firmware_version
  rosidl_runtime_c__String__fini(&msg->firmware_version);
  // usb_type_descriptor
  rosidl_runtime_c__String__fini(&msg->usb_type_descriptor);
  // firmware_update_id
  rosidl_runtime_c__String__fini(&msg->firmware_update_id);
  // sensors
  rosidl_runtime_c__String__fini(&msg->sensors);
  // physical_port
  rosidl_runtime_c__String__fini(&msg->physical_port);
}

bool
realsense2_camera_msgs__srv__DeviceInfo_Response__are_equal(const realsense2_camera_msgs__srv__DeviceInfo_Response * lhs, const realsense2_camera_msgs__srv__DeviceInfo_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // device_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->device_name), &(rhs->device_name)))
  {
    return false;
  }
  // serial_number
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->serial_number), &(rhs->serial_number)))
  {
    return false;
  }
  // firmware_version
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->firmware_version), &(rhs->firmware_version)))
  {
    return false;
  }
  // usb_type_descriptor
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->usb_type_descriptor), &(rhs->usb_type_descriptor)))
  {
    return false;
  }
  // firmware_update_id
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->firmware_update_id), &(rhs->firmware_update_id)))
  {
    return false;
  }
  // sensors
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sensors), &(rhs->sensors)))
  {
    return false;
  }
  // physical_port
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->physical_port), &(rhs->physical_port)))
  {
    return false;
  }
  return true;
}

bool
realsense2_camera_msgs__srv__DeviceInfo_Response__copy(
  const realsense2_camera_msgs__srv__DeviceInfo_Response * input,
  realsense2_camera_msgs__srv__DeviceInfo_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // device_name
  if (!rosidl_runtime_c__String__copy(
      &(input->device_name), &(output->device_name)))
  {
    return false;
  }
  // serial_number
  if (!rosidl_runtime_c__String__copy(
      &(input->serial_number), &(output->serial_number)))
  {
    return false;
  }
  // firmware_version
  if (!rosidl_runtime_c__String__copy(
      &(input->firmware_version), &(output->firmware_version)))
  {
    return false;
  }
  // usb_type_descriptor
  if (!rosidl_runtime_c__String__copy(
      &(input->usb_type_descriptor), &(output->usb_type_descriptor)))
  {
    return false;
  }
  // firmware_update_id
  if (!rosidl_runtime_c__String__copy(
      &(input->firmware_update_id), &(output->firmware_update_id)))
  {
    return false;
  }
  // sensors
  if (!rosidl_runtime_c__String__copy(
      &(input->sensors), &(output->sensors)))
  {
    return false;
  }
  // physical_port
  if (!rosidl_runtime_c__String__copy(
      &(input->physical_port), &(output->physical_port)))
  {
    return false;
  }
  return true;
}

realsense2_camera_msgs__srv__DeviceInfo_Response *
realsense2_camera_msgs__srv__DeviceInfo_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  realsense2_camera_msgs__srv__DeviceInfo_Response * msg = (realsense2_camera_msgs__srv__DeviceInfo_Response *)allocator.allocate(sizeof(realsense2_camera_msgs__srv__DeviceInfo_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(realsense2_camera_msgs__srv__DeviceInfo_Response));
  bool success = realsense2_camera_msgs__srv__DeviceInfo_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
realsense2_camera_msgs__srv__DeviceInfo_Response__destroy(realsense2_camera_msgs__srv__DeviceInfo_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    realsense2_camera_msgs__srv__DeviceInfo_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence__init(realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  realsense2_camera_msgs__srv__DeviceInfo_Response * data = NULL;

  if (size) {
    data = (realsense2_camera_msgs__srv__DeviceInfo_Response *)allocator.zero_allocate(size, sizeof(realsense2_camera_msgs__srv__DeviceInfo_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = realsense2_camera_msgs__srv__DeviceInfo_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        realsense2_camera_msgs__srv__DeviceInfo_Response__fini(&data[i - 1]);
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
realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence__fini(realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence * array)
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
      realsense2_camera_msgs__srv__DeviceInfo_Response__fini(&array->data[i]);
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

realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence *
realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence * array = (realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence *)allocator.allocate(sizeof(realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence__destroy(realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence__are_equal(const realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence * lhs, const realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!realsense2_camera_msgs__srv__DeviceInfo_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence__copy(
  const realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence * input,
  realsense2_camera_msgs__srv__DeviceInfo_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(realsense2_camera_msgs__srv__DeviceInfo_Response);
    realsense2_camera_msgs__srv__DeviceInfo_Response * data =
      (realsense2_camera_msgs__srv__DeviceInfo_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!realsense2_camera_msgs__srv__DeviceInfo_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          realsense2_camera_msgs__srv__DeviceInfo_Response__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!realsense2_camera_msgs__srv__DeviceInfo_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
