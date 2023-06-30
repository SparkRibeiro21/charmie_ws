// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/Yolov8Pose.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/yolov8_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `keypoints`
#include "charmie_interfaces/msg/detail/keypoints__functions.h"

bool
charmie_interfaces__msg__Yolov8Pose__init(charmie_interfaces__msg__Yolov8Pose * msg)
{
  if (!msg) {
    return false;
  }
  // num_person
  // keypoints
  if (!charmie_interfaces__msg__Keypoints__Sequence__init(&msg->keypoints, 0)) {
    charmie_interfaces__msg__Yolov8Pose__fini(msg);
    return false;
  }
  return true;
}

void
charmie_interfaces__msg__Yolov8Pose__fini(charmie_interfaces__msg__Yolov8Pose * msg)
{
  if (!msg) {
    return;
  }
  // num_person
  // keypoints
  charmie_interfaces__msg__Keypoints__Sequence__fini(&msg->keypoints);
}

bool
charmie_interfaces__msg__Yolov8Pose__are_equal(const charmie_interfaces__msg__Yolov8Pose * lhs, const charmie_interfaces__msg__Yolov8Pose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // num_person
  if (lhs->num_person != rhs->num_person) {
    return false;
  }
  // keypoints
  if (!charmie_interfaces__msg__Keypoints__Sequence__are_equal(
      &(lhs->keypoints), &(rhs->keypoints)))
  {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__Yolov8Pose__copy(
  const charmie_interfaces__msg__Yolov8Pose * input,
  charmie_interfaces__msg__Yolov8Pose * output)
{
  if (!input || !output) {
    return false;
  }
  // num_person
  output->num_person = input->num_person;
  // keypoints
  if (!charmie_interfaces__msg__Keypoints__Sequence__copy(
      &(input->keypoints), &(output->keypoints)))
  {
    return false;
  }
  return true;
}

charmie_interfaces__msg__Yolov8Pose *
charmie_interfaces__msg__Yolov8Pose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Yolov8Pose * msg = (charmie_interfaces__msg__Yolov8Pose *)allocator.allocate(sizeof(charmie_interfaces__msg__Yolov8Pose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__Yolov8Pose));
  bool success = charmie_interfaces__msg__Yolov8Pose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__Yolov8Pose__destroy(charmie_interfaces__msg__Yolov8Pose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__Yolov8Pose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__Yolov8Pose__Sequence__init(charmie_interfaces__msg__Yolov8Pose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Yolov8Pose * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__Yolov8Pose *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__Yolov8Pose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__Yolov8Pose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__Yolov8Pose__fini(&data[i - 1]);
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
charmie_interfaces__msg__Yolov8Pose__Sequence__fini(charmie_interfaces__msg__Yolov8Pose__Sequence * array)
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
      charmie_interfaces__msg__Yolov8Pose__fini(&array->data[i]);
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

charmie_interfaces__msg__Yolov8Pose__Sequence *
charmie_interfaces__msg__Yolov8Pose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Yolov8Pose__Sequence * array = (charmie_interfaces__msg__Yolov8Pose__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__Yolov8Pose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__Yolov8Pose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__Yolov8Pose__Sequence__destroy(charmie_interfaces__msg__Yolov8Pose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__Yolov8Pose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__Yolov8Pose__Sequence__are_equal(const charmie_interfaces__msg__Yolov8Pose__Sequence * lhs, const charmie_interfaces__msg__Yolov8Pose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__Yolov8Pose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__Yolov8Pose__Sequence__copy(
  const charmie_interfaces__msg__Yolov8Pose__Sequence * input,
  charmie_interfaces__msg__Yolov8Pose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__Yolov8Pose);
    charmie_interfaces__msg__Yolov8Pose * data =
      (charmie_interfaces__msg__Yolov8Pose *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__Yolov8Pose__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__Yolov8Pose__fini(&data[i]);
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
    if (!charmie_interfaces__msg__Yolov8Pose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
