// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/Obstacles.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/obstacles__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `obstacles`
#include "charmie_interfaces/msg/detail/obstacle_info__functions.h"

bool
charmie_interfaces__msg__Obstacles__init(charmie_interfaces__msg__Obstacles * msg)
{
  if (!msg) {
    return false;
  }
  // no_obstacles
  // obstacles
  if (!charmie_interfaces__msg__ObstacleInfo__Sequence__init(&msg->obstacles, 0)) {
    charmie_interfaces__msg__Obstacles__fini(msg);
    return false;
  }
  return true;
}

void
charmie_interfaces__msg__Obstacles__fini(charmie_interfaces__msg__Obstacles * msg)
{
  if (!msg) {
    return;
  }
  // no_obstacles
  // obstacles
  charmie_interfaces__msg__ObstacleInfo__Sequence__fini(&msg->obstacles);
}

bool
charmie_interfaces__msg__Obstacles__are_equal(const charmie_interfaces__msg__Obstacles * lhs, const charmie_interfaces__msg__Obstacles * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // no_obstacles
  if (lhs->no_obstacles != rhs->no_obstacles) {
    return false;
  }
  // obstacles
  if (!charmie_interfaces__msg__ObstacleInfo__Sequence__are_equal(
      &(lhs->obstacles), &(rhs->obstacles)))
  {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__Obstacles__copy(
  const charmie_interfaces__msg__Obstacles * input,
  charmie_interfaces__msg__Obstacles * output)
{
  if (!input || !output) {
    return false;
  }
  // no_obstacles
  output->no_obstacles = input->no_obstacles;
  // obstacles
  if (!charmie_interfaces__msg__ObstacleInfo__Sequence__copy(
      &(input->obstacles), &(output->obstacles)))
  {
    return false;
  }
  return true;
}

charmie_interfaces__msg__Obstacles *
charmie_interfaces__msg__Obstacles__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Obstacles * msg = (charmie_interfaces__msg__Obstacles *)allocator.allocate(sizeof(charmie_interfaces__msg__Obstacles), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__Obstacles));
  bool success = charmie_interfaces__msg__Obstacles__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__Obstacles__destroy(charmie_interfaces__msg__Obstacles * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__Obstacles__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__Obstacles__Sequence__init(charmie_interfaces__msg__Obstacles__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Obstacles * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__Obstacles *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__Obstacles), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__Obstacles__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__Obstacles__fini(&data[i - 1]);
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
charmie_interfaces__msg__Obstacles__Sequence__fini(charmie_interfaces__msg__Obstacles__Sequence * array)
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
      charmie_interfaces__msg__Obstacles__fini(&array->data[i]);
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

charmie_interfaces__msg__Obstacles__Sequence *
charmie_interfaces__msg__Obstacles__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Obstacles__Sequence * array = (charmie_interfaces__msg__Obstacles__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__Obstacles__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__Obstacles__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__Obstacles__Sequence__destroy(charmie_interfaces__msg__Obstacles__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__Obstacles__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__Obstacles__Sequence__are_equal(const charmie_interfaces__msg__Obstacles__Sequence * lhs, const charmie_interfaces__msg__Obstacles__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__Obstacles__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__Obstacles__Sequence__copy(
  const charmie_interfaces__msg__Obstacles__Sequence * input,
  charmie_interfaces__msg__Obstacles__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__Obstacles);
    charmie_interfaces__msg__Obstacles * data =
      (charmie_interfaces__msg__Obstacles *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__Obstacles__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__Obstacles__fini(&data[i]);
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
    if (!charmie_interfaces__msg__Obstacles__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
