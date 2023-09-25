// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/TarNavSDNL.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/tar_nav_sdnl__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `move_target_coordinates`
// Member `rotate_target_coordinates`
#include "geometry_msgs/msg/detail/pose2_d__functions.h"

bool
charmie_interfaces__msg__TarNavSDNL__init(charmie_interfaces__msg__TarNavSDNL * msg)
{
  if (!msg) {
    return false;
  }
  // move_target_coordinates
  if (!geometry_msgs__msg__Pose2D__init(&msg->move_target_coordinates)) {
    charmie_interfaces__msg__TarNavSDNL__fini(msg);
    return false;
  }
  // rotate_target_coordinates
  if (!geometry_msgs__msg__Pose2D__init(&msg->rotate_target_coordinates)) {
    charmie_interfaces__msg__TarNavSDNL__fini(msg);
    return false;
  }
  // flag_not_obs
  // follow_me
  return true;
}

void
charmie_interfaces__msg__TarNavSDNL__fini(charmie_interfaces__msg__TarNavSDNL * msg)
{
  if (!msg) {
    return;
  }
  // move_target_coordinates
  geometry_msgs__msg__Pose2D__fini(&msg->move_target_coordinates);
  // rotate_target_coordinates
  geometry_msgs__msg__Pose2D__fini(&msg->rotate_target_coordinates);
  // flag_not_obs
  // follow_me
}

bool
charmie_interfaces__msg__TarNavSDNL__are_equal(const charmie_interfaces__msg__TarNavSDNL * lhs, const charmie_interfaces__msg__TarNavSDNL * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // move_target_coordinates
  if (!geometry_msgs__msg__Pose2D__are_equal(
      &(lhs->move_target_coordinates), &(rhs->move_target_coordinates)))
  {
    return false;
  }
  // rotate_target_coordinates
  if (!geometry_msgs__msg__Pose2D__are_equal(
      &(lhs->rotate_target_coordinates), &(rhs->rotate_target_coordinates)))
  {
    return false;
  }
  // flag_not_obs
  if (lhs->flag_not_obs != rhs->flag_not_obs) {
    return false;
  }
  // follow_me
  if (lhs->follow_me != rhs->follow_me) {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__TarNavSDNL__copy(
  const charmie_interfaces__msg__TarNavSDNL * input,
  charmie_interfaces__msg__TarNavSDNL * output)
{
  if (!input || !output) {
    return false;
  }
  // move_target_coordinates
  if (!geometry_msgs__msg__Pose2D__copy(
      &(input->move_target_coordinates), &(output->move_target_coordinates)))
  {
    return false;
  }
  // rotate_target_coordinates
  if (!geometry_msgs__msg__Pose2D__copy(
      &(input->rotate_target_coordinates), &(output->rotate_target_coordinates)))
  {
    return false;
  }
  // flag_not_obs
  output->flag_not_obs = input->flag_not_obs;
  // follow_me
  output->follow_me = input->follow_me;
  return true;
}

charmie_interfaces__msg__TarNavSDNL *
charmie_interfaces__msg__TarNavSDNL__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__TarNavSDNL * msg = (charmie_interfaces__msg__TarNavSDNL *)allocator.allocate(sizeof(charmie_interfaces__msg__TarNavSDNL), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__TarNavSDNL));
  bool success = charmie_interfaces__msg__TarNavSDNL__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__TarNavSDNL__destroy(charmie_interfaces__msg__TarNavSDNL * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__TarNavSDNL__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__TarNavSDNL__Sequence__init(charmie_interfaces__msg__TarNavSDNL__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__TarNavSDNL * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__TarNavSDNL *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__TarNavSDNL), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__TarNavSDNL__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__TarNavSDNL__fini(&data[i - 1]);
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
charmie_interfaces__msg__TarNavSDNL__Sequence__fini(charmie_interfaces__msg__TarNavSDNL__Sequence * array)
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
      charmie_interfaces__msg__TarNavSDNL__fini(&array->data[i]);
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

charmie_interfaces__msg__TarNavSDNL__Sequence *
charmie_interfaces__msg__TarNavSDNL__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__TarNavSDNL__Sequence * array = (charmie_interfaces__msg__TarNavSDNL__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__TarNavSDNL__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__TarNavSDNL__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__TarNavSDNL__Sequence__destroy(charmie_interfaces__msg__TarNavSDNL__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__TarNavSDNL__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__TarNavSDNL__Sequence__are_equal(const charmie_interfaces__msg__TarNavSDNL__Sequence * lhs, const charmie_interfaces__msg__TarNavSDNL__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__TarNavSDNL__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__TarNavSDNL__Sequence__copy(
  const charmie_interfaces__msg__TarNavSDNL__Sequence * input,
  charmie_interfaces__msg__TarNavSDNL__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__TarNavSDNL);
    charmie_interfaces__msg__TarNavSDNL * data =
      (charmie_interfaces__msg__TarNavSDNL *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__TarNavSDNL__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__TarNavSDNL__fini(&data[i]);
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
    if (!charmie_interfaces__msg__TarNavSDNL__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
