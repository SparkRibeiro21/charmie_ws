// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/MultiObjects.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/multi_objects__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `objects`
#include "rosidl_runtime_c/string_functions.h"
// Member `confidence`
// Member `distance`
// Member `position`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
charmie_interfaces__msg__MultiObjects__init(charmie_interfaces__msg__MultiObjects * msg)
{
  if (!msg) {
    return false;
  }
  // objects
  if (!rosidl_runtime_c__String__Sequence__init(&msg->objects, 0)) {
    charmie_interfaces__msg__MultiObjects__fini(msg);
    return false;
  }
  // confidence
  if (!rosidl_runtime_c__float__Sequence__init(&msg->confidence, 0)) {
    charmie_interfaces__msg__MultiObjects__fini(msg);
    return false;
  }
  // distance
  if (!rosidl_runtime_c__float__Sequence__init(&msg->distance, 0)) {
    charmie_interfaces__msg__MultiObjects__fini(msg);
    return false;
  }
  // position
  if (!rosidl_runtime_c__float__Sequence__init(&msg->position, 0)) {
    charmie_interfaces__msg__MultiObjects__fini(msg);
    return false;
  }
  return true;
}

void
charmie_interfaces__msg__MultiObjects__fini(charmie_interfaces__msg__MultiObjects * msg)
{
  if (!msg) {
    return;
  }
  // objects
  rosidl_runtime_c__String__Sequence__fini(&msg->objects);
  // confidence
  rosidl_runtime_c__float__Sequence__fini(&msg->confidence);
  // distance
  rosidl_runtime_c__float__Sequence__fini(&msg->distance);
  // position
  rosidl_runtime_c__float__Sequence__fini(&msg->position);
}

bool
charmie_interfaces__msg__MultiObjects__are_equal(const charmie_interfaces__msg__MultiObjects * lhs, const charmie_interfaces__msg__MultiObjects * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // objects
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->objects), &(rhs->objects)))
  {
    return false;
  }
  // confidence
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->confidence), &(rhs->confidence)))
  {
    return false;
  }
  // distance
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->distance), &(rhs->distance)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__MultiObjects__copy(
  const charmie_interfaces__msg__MultiObjects * input,
  charmie_interfaces__msg__MultiObjects * output)
{
  if (!input || !output) {
    return false;
  }
  // objects
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->objects), &(output->objects)))
  {
    return false;
  }
  // confidence
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->confidence), &(output->confidence)))
  {
    return false;
  }
  // distance
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->distance), &(output->distance)))
  {
    return false;
  }
  // position
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  return true;
}

charmie_interfaces__msg__MultiObjects *
charmie_interfaces__msg__MultiObjects__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__MultiObjects * msg = (charmie_interfaces__msg__MultiObjects *)allocator.allocate(sizeof(charmie_interfaces__msg__MultiObjects), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__MultiObjects));
  bool success = charmie_interfaces__msg__MultiObjects__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__MultiObjects__destroy(charmie_interfaces__msg__MultiObjects * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__MultiObjects__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__MultiObjects__Sequence__init(charmie_interfaces__msg__MultiObjects__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__MultiObjects * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__MultiObjects *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__MultiObjects), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__MultiObjects__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__MultiObjects__fini(&data[i - 1]);
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
charmie_interfaces__msg__MultiObjects__Sequence__fini(charmie_interfaces__msg__MultiObjects__Sequence * array)
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
      charmie_interfaces__msg__MultiObjects__fini(&array->data[i]);
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

charmie_interfaces__msg__MultiObjects__Sequence *
charmie_interfaces__msg__MultiObjects__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__MultiObjects__Sequence * array = (charmie_interfaces__msg__MultiObjects__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__MultiObjects__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__MultiObjects__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__MultiObjects__Sequence__destroy(charmie_interfaces__msg__MultiObjects__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__MultiObjects__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__MultiObjects__Sequence__are_equal(const charmie_interfaces__msg__MultiObjects__Sequence * lhs, const charmie_interfaces__msg__MultiObjects__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__MultiObjects__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__MultiObjects__Sequence__copy(
  const charmie_interfaces__msg__MultiObjects__Sequence * input,
  charmie_interfaces__msg__MultiObjects__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__MultiObjects);
    charmie_interfaces__msg__MultiObjects * data =
      (charmie_interfaces__msg__MultiObjects *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__MultiObjects__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__MultiObjects__fini(&data[i]);
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
    if (!charmie_interfaces__msg__MultiObjects__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
