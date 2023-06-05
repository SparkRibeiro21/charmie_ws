// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/ExampleTR.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/example_tr__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `coordinates`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
charmie_interfaces__msg__ExampleTR__init(charmie_interfaces__msg__ExampleTR * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    charmie_interfaces__msg__ExampleTR__fini(msg);
    return false;
  }
  // coordinates
  if (!geometry_msgs__msg__Point__init(&msg->coordinates)) {
    charmie_interfaces__msg__ExampleTR__fini(msg);
    return false;
  }
  return true;
}

void
charmie_interfaces__msg__ExampleTR__fini(charmie_interfaces__msg__ExampleTR * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // coordinates
  geometry_msgs__msg__Point__fini(&msg->coordinates);
}

bool
charmie_interfaces__msg__ExampleTR__are_equal(const charmie_interfaces__msg__ExampleTR * lhs, const charmie_interfaces__msg__ExampleTR * rhs)
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
  // coordinates
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->coordinates), &(rhs->coordinates)))
  {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__ExampleTR__copy(
  const charmie_interfaces__msg__ExampleTR * input,
  charmie_interfaces__msg__ExampleTR * output)
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
  // coordinates
  if (!geometry_msgs__msg__Point__copy(
      &(input->coordinates), &(output->coordinates)))
  {
    return false;
  }
  return true;
}

charmie_interfaces__msg__ExampleTR *
charmie_interfaces__msg__ExampleTR__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__ExampleTR * msg = (charmie_interfaces__msg__ExampleTR *)allocator.allocate(sizeof(charmie_interfaces__msg__ExampleTR), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__ExampleTR));
  bool success = charmie_interfaces__msg__ExampleTR__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__ExampleTR__destroy(charmie_interfaces__msg__ExampleTR * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__ExampleTR__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__ExampleTR__Sequence__init(charmie_interfaces__msg__ExampleTR__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__ExampleTR * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__ExampleTR *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__ExampleTR), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__ExampleTR__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__ExampleTR__fini(&data[i - 1]);
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
charmie_interfaces__msg__ExampleTR__Sequence__fini(charmie_interfaces__msg__ExampleTR__Sequence * array)
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
      charmie_interfaces__msg__ExampleTR__fini(&array->data[i]);
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

charmie_interfaces__msg__ExampleTR__Sequence *
charmie_interfaces__msg__ExampleTR__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__ExampleTR__Sequence * array = (charmie_interfaces__msg__ExampleTR__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__ExampleTR__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__ExampleTR__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__ExampleTR__Sequence__destroy(charmie_interfaces__msg__ExampleTR__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__ExampleTR__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__ExampleTR__Sequence__are_equal(const charmie_interfaces__msg__ExampleTR__Sequence * lhs, const charmie_interfaces__msg__ExampleTR__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__ExampleTR__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__ExampleTR__Sequence__copy(
  const charmie_interfaces__msg__ExampleTR__Sequence * input,
  charmie_interfaces__msg__ExampleTR__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__ExampleTR);
    charmie_interfaces__msg__ExampleTR * data =
      (charmie_interfaces__msg__ExampleTR *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__ExampleTR__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__ExampleTR__fini(&data[i]);
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
    if (!charmie_interfaces__msg__ExampleTR__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
