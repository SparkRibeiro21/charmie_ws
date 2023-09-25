// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/Keypoints.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/keypoints__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
charmie_interfaces__msg__Keypoints__init(charmie_interfaces__msg__Keypoints * msg)
{
  if (!msg) {
    return false;
  }
  // index_person
  // x_person_relative
  // average_distance
  // standard_deviation
  // box_topx_left
  // box_topy_left
  // box_width
  // box_height
  // key_p0_x
  // key_p0_y
  // key_p1_x
  // key_p1_y
  // key_p2_x
  // key_p2_y
  // key_p3_x
  // key_p3_y
  // key_p4_x
  // key_p4_y
  // key_p5_x
  // key_p5_y
  // key_p6_x
  // key_p6_y
  // key_p7_x
  // key_p7_y
  // key_p8_x
  // key_p8_y
  // key_p9_x
  // key_p9_y
  // key_p10_x
  // key_p10_y
  // key_p11_x
  // key_p11_y
  // key_p12_x
  // key_p12_y
  // key_p13_x
  // key_p13_y
  // key_p14_x
  // key_p14_y
  // key_p15_x
  // key_p15_y
  // key_p16_x
  // key_p16_y
  return true;
}

void
charmie_interfaces__msg__Keypoints__fini(charmie_interfaces__msg__Keypoints * msg)
{
  if (!msg) {
    return;
  }
  // index_person
  // x_person_relative
  // average_distance
  // standard_deviation
  // box_topx_left
  // box_topy_left
  // box_width
  // box_height
  // key_p0_x
  // key_p0_y
  // key_p1_x
  // key_p1_y
  // key_p2_x
  // key_p2_y
  // key_p3_x
  // key_p3_y
  // key_p4_x
  // key_p4_y
  // key_p5_x
  // key_p5_y
  // key_p6_x
  // key_p6_y
  // key_p7_x
  // key_p7_y
  // key_p8_x
  // key_p8_y
  // key_p9_x
  // key_p9_y
  // key_p10_x
  // key_p10_y
  // key_p11_x
  // key_p11_y
  // key_p12_x
  // key_p12_y
  // key_p13_x
  // key_p13_y
  // key_p14_x
  // key_p14_y
  // key_p15_x
  // key_p15_y
  // key_p16_x
  // key_p16_y
}

bool
charmie_interfaces__msg__Keypoints__are_equal(const charmie_interfaces__msg__Keypoints * lhs, const charmie_interfaces__msg__Keypoints * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // index_person
  if (lhs->index_person != rhs->index_person) {
    return false;
  }
  // x_person_relative
  if (lhs->x_person_relative != rhs->x_person_relative) {
    return false;
  }
  // average_distance
  if (lhs->average_distance != rhs->average_distance) {
    return false;
  }
  // standard_deviation
  if (lhs->standard_deviation != rhs->standard_deviation) {
    return false;
  }
  // box_topx_left
  if (lhs->box_topx_left != rhs->box_topx_left) {
    return false;
  }
  // box_topy_left
  if (lhs->box_topy_left != rhs->box_topy_left) {
    return false;
  }
  // box_width
  if (lhs->box_width != rhs->box_width) {
    return false;
  }
  // box_height
  if (lhs->box_height != rhs->box_height) {
    return false;
  }
  // key_p0_x
  if (lhs->key_p0_x != rhs->key_p0_x) {
    return false;
  }
  // key_p0_y
  if (lhs->key_p0_y != rhs->key_p0_y) {
    return false;
  }
  // key_p1_x
  if (lhs->key_p1_x != rhs->key_p1_x) {
    return false;
  }
  // key_p1_y
  if (lhs->key_p1_y != rhs->key_p1_y) {
    return false;
  }
  // key_p2_x
  if (lhs->key_p2_x != rhs->key_p2_x) {
    return false;
  }
  // key_p2_y
  if (lhs->key_p2_y != rhs->key_p2_y) {
    return false;
  }
  // key_p3_x
  if (lhs->key_p3_x != rhs->key_p3_x) {
    return false;
  }
  // key_p3_y
  if (lhs->key_p3_y != rhs->key_p3_y) {
    return false;
  }
  // key_p4_x
  if (lhs->key_p4_x != rhs->key_p4_x) {
    return false;
  }
  // key_p4_y
  if (lhs->key_p4_y != rhs->key_p4_y) {
    return false;
  }
  // key_p5_x
  if (lhs->key_p5_x != rhs->key_p5_x) {
    return false;
  }
  // key_p5_y
  if (lhs->key_p5_y != rhs->key_p5_y) {
    return false;
  }
  // key_p6_x
  if (lhs->key_p6_x != rhs->key_p6_x) {
    return false;
  }
  // key_p6_y
  if (lhs->key_p6_y != rhs->key_p6_y) {
    return false;
  }
  // key_p7_x
  if (lhs->key_p7_x != rhs->key_p7_x) {
    return false;
  }
  // key_p7_y
  if (lhs->key_p7_y != rhs->key_p7_y) {
    return false;
  }
  // key_p8_x
  if (lhs->key_p8_x != rhs->key_p8_x) {
    return false;
  }
  // key_p8_y
  if (lhs->key_p8_y != rhs->key_p8_y) {
    return false;
  }
  // key_p9_x
  if (lhs->key_p9_x != rhs->key_p9_x) {
    return false;
  }
  // key_p9_y
  if (lhs->key_p9_y != rhs->key_p9_y) {
    return false;
  }
  // key_p10_x
  if (lhs->key_p10_x != rhs->key_p10_x) {
    return false;
  }
  // key_p10_y
  if (lhs->key_p10_y != rhs->key_p10_y) {
    return false;
  }
  // key_p11_x
  if (lhs->key_p11_x != rhs->key_p11_x) {
    return false;
  }
  // key_p11_y
  if (lhs->key_p11_y != rhs->key_p11_y) {
    return false;
  }
  // key_p12_x
  if (lhs->key_p12_x != rhs->key_p12_x) {
    return false;
  }
  // key_p12_y
  if (lhs->key_p12_y != rhs->key_p12_y) {
    return false;
  }
  // key_p13_x
  if (lhs->key_p13_x != rhs->key_p13_x) {
    return false;
  }
  // key_p13_y
  if (lhs->key_p13_y != rhs->key_p13_y) {
    return false;
  }
  // key_p14_x
  if (lhs->key_p14_x != rhs->key_p14_x) {
    return false;
  }
  // key_p14_y
  if (lhs->key_p14_y != rhs->key_p14_y) {
    return false;
  }
  // key_p15_x
  if (lhs->key_p15_x != rhs->key_p15_x) {
    return false;
  }
  // key_p15_y
  if (lhs->key_p15_y != rhs->key_p15_y) {
    return false;
  }
  // key_p16_x
  if (lhs->key_p16_x != rhs->key_p16_x) {
    return false;
  }
  // key_p16_y
  if (lhs->key_p16_y != rhs->key_p16_y) {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__Keypoints__copy(
  const charmie_interfaces__msg__Keypoints * input,
  charmie_interfaces__msg__Keypoints * output)
{
  if (!input || !output) {
    return false;
  }
  // index_person
  output->index_person = input->index_person;
  // x_person_relative
  output->x_person_relative = input->x_person_relative;
  // average_distance
  output->average_distance = input->average_distance;
  // standard_deviation
  output->standard_deviation = input->standard_deviation;
  // box_topx_left
  output->box_topx_left = input->box_topx_left;
  // box_topy_left
  output->box_topy_left = input->box_topy_left;
  // box_width
  output->box_width = input->box_width;
  // box_height
  output->box_height = input->box_height;
  // key_p0_x
  output->key_p0_x = input->key_p0_x;
  // key_p0_y
  output->key_p0_y = input->key_p0_y;
  // key_p1_x
  output->key_p1_x = input->key_p1_x;
  // key_p1_y
  output->key_p1_y = input->key_p1_y;
  // key_p2_x
  output->key_p2_x = input->key_p2_x;
  // key_p2_y
  output->key_p2_y = input->key_p2_y;
  // key_p3_x
  output->key_p3_x = input->key_p3_x;
  // key_p3_y
  output->key_p3_y = input->key_p3_y;
  // key_p4_x
  output->key_p4_x = input->key_p4_x;
  // key_p4_y
  output->key_p4_y = input->key_p4_y;
  // key_p5_x
  output->key_p5_x = input->key_p5_x;
  // key_p5_y
  output->key_p5_y = input->key_p5_y;
  // key_p6_x
  output->key_p6_x = input->key_p6_x;
  // key_p6_y
  output->key_p6_y = input->key_p6_y;
  // key_p7_x
  output->key_p7_x = input->key_p7_x;
  // key_p7_y
  output->key_p7_y = input->key_p7_y;
  // key_p8_x
  output->key_p8_x = input->key_p8_x;
  // key_p8_y
  output->key_p8_y = input->key_p8_y;
  // key_p9_x
  output->key_p9_x = input->key_p9_x;
  // key_p9_y
  output->key_p9_y = input->key_p9_y;
  // key_p10_x
  output->key_p10_x = input->key_p10_x;
  // key_p10_y
  output->key_p10_y = input->key_p10_y;
  // key_p11_x
  output->key_p11_x = input->key_p11_x;
  // key_p11_y
  output->key_p11_y = input->key_p11_y;
  // key_p12_x
  output->key_p12_x = input->key_p12_x;
  // key_p12_y
  output->key_p12_y = input->key_p12_y;
  // key_p13_x
  output->key_p13_x = input->key_p13_x;
  // key_p13_y
  output->key_p13_y = input->key_p13_y;
  // key_p14_x
  output->key_p14_x = input->key_p14_x;
  // key_p14_y
  output->key_p14_y = input->key_p14_y;
  // key_p15_x
  output->key_p15_x = input->key_p15_x;
  // key_p15_y
  output->key_p15_y = input->key_p15_y;
  // key_p16_x
  output->key_p16_x = input->key_p16_x;
  // key_p16_y
  output->key_p16_y = input->key_p16_y;
  return true;
}

charmie_interfaces__msg__Keypoints *
charmie_interfaces__msg__Keypoints__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Keypoints * msg = (charmie_interfaces__msg__Keypoints *)allocator.allocate(sizeof(charmie_interfaces__msg__Keypoints), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__Keypoints));
  bool success = charmie_interfaces__msg__Keypoints__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__Keypoints__destroy(charmie_interfaces__msg__Keypoints * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__Keypoints__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__Keypoints__Sequence__init(charmie_interfaces__msg__Keypoints__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Keypoints * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__Keypoints *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__Keypoints), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__Keypoints__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__Keypoints__fini(&data[i - 1]);
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
charmie_interfaces__msg__Keypoints__Sequence__fini(charmie_interfaces__msg__Keypoints__Sequence * array)
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
      charmie_interfaces__msg__Keypoints__fini(&array->data[i]);
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

charmie_interfaces__msg__Keypoints__Sequence *
charmie_interfaces__msg__Keypoints__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Keypoints__Sequence * array = (charmie_interfaces__msg__Keypoints__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__Keypoints__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__Keypoints__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__Keypoints__Sequence__destroy(charmie_interfaces__msg__Keypoints__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__Keypoints__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__Keypoints__Sequence__are_equal(const charmie_interfaces__msg__Keypoints__Sequence * lhs, const charmie_interfaces__msg__Keypoints__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__Keypoints__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__Keypoints__Sequence__copy(
  const charmie_interfaces__msg__Keypoints__Sequence * input,
  charmie_interfaces__msg__Keypoints__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__Keypoints);
    charmie_interfaces__msg__Keypoints * data =
      (charmie_interfaces__msg__Keypoints *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__Keypoints__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__Keypoints__fini(&data[i]);
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
    if (!charmie_interfaces__msg__Keypoints__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
