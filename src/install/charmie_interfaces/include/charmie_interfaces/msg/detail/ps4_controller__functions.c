// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/PS4Controller.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/ps4_controller__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
charmie_interfaces__msg__PS4Controller__init(charmie_interfaces__msg__PS4Controller * msg)
{
  if (!msg) {
    return false;
  }
  // triangle
  // circle
  // cross
  // square
  // arrow_up
  // arrow_right
  // arrow_down
  // arrow_left
  // l1
  // r1
  // l3
  // r3
  // share
  // options
  // ps
  // l3_ang
  // l3_dist
  // l3_xx
  // l3_yy
  // r3_ang
  // r3_dist
  // r3_xx
  // r3_yy
  // l2
  // r2
  return true;
}

void
charmie_interfaces__msg__PS4Controller__fini(charmie_interfaces__msg__PS4Controller * msg)
{
  if (!msg) {
    return;
  }
  // triangle
  // circle
  // cross
  // square
  // arrow_up
  // arrow_right
  // arrow_down
  // arrow_left
  // l1
  // r1
  // l3
  // r3
  // share
  // options
  // ps
  // l3_ang
  // l3_dist
  // l3_xx
  // l3_yy
  // r3_ang
  // r3_dist
  // r3_xx
  // r3_yy
  // l2
  // r2
}

bool
charmie_interfaces__msg__PS4Controller__are_equal(const charmie_interfaces__msg__PS4Controller * lhs, const charmie_interfaces__msg__PS4Controller * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // triangle
  if (lhs->triangle != rhs->triangle) {
    return false;
  }
  // circle
  if (lhs->circle != rhs->circle) {
    return false;
  }
  // cross
  if (lhs->cross != rhs->cross) {
    return false;
  }
  // square
  if (lhs->square != rhs->square) {
    return false;
  }
  // arrow_up
  if (lhs->arrow_up != rhs->arrow_up) {
    return false;
  }
  // arrow_right
  if (lhs->arrow_right != rhs->arrow_right) {
    return false;
  }
  // arrow_down
  if (lhs->arrow_down != rhs->arrow_down) {
    return false;
  }
  // arrow_left
  if (lhs->arrow_left != rhs->arrow_left) {
    return false;
  }
  // l1
  if (lhs->l1 != rhs->l1) {
    return false;
  }
  // r1
  if (lhs->r1 != rhs->r1) {
    return false;
  }
  // l3
  if (lhs->l3 != rhs->l3) {
    return false;
  }
  // r3
  if (lhs->r3 != rhs->r3) {
    return false;
  }
  // share
  if (lhs->share != rhs->share) {
    return false;
  }
  // options
  if (lhs->options != rhs->options) {
    return false;
  }
  // ps
  if (lhs->ps != rhs->ps) {
    return false;
  }
  // l3_ang
  if (lhs->l3_ang != rhs->l3_ang) {
    return false;
  }
  // l3_dist
  if (lhs->l3_dist != rhs->l3_dist) {
    return false;
  }
  // l3_xx
  if (lhs->l3_xx != rhs->l3_xx) {
    return false;
  }
  // l3_yy
  if (lhs->l3_yy != rhs->l3_yy) {
    return false;
  }
  // r3_ang
  if (lhs->r3_ang != rhs->r3_ang) {
    return false;
  }
  // r3_dist
  if (lhs->r3_dist != rhs->r3_dist) {
    return false;
  }
  // r3_xx
  if (lhs->r3_xx != rhs->r3_xx) {
    return false;
  }
  // r3_yy
  if (lhs->r3_yy != rhs->r3_yy) {
    return false;
  }
  // l2
  if (lhs->l2 != rhs->l2) {
    return false;
  }
  // r2
  if (lhs->r2 != rhs->r2) {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__PS4Controller__copy(
  const charmie_interfaces__msg__PS4Controller * input,
  charmie_interfaces__msg__PS4Controller * output)
{
  if (!input || !output) {
    return false;
  }
  // triangle
  output->triangle = input->triangle;
  // circle
  output->circle = input->circle;
  // cross
  output->cross = input->cross;
  // square
  output->square = input->square;
  // arrow_up
  output->arrow_up = input->arrow_up;
  // arrow_right
  output->arrow_right = input->arrow_right;
  // arrow_down
  output->arrow_down = input->arrow_down;
  // arrow_left
  output->arrow_left = input->arrow_left;
  // l1
  output->l1 = input->l1;
  // r1
  output->r1 = input->r1;
  // l3
  output->l3 = input->l3;
  // r3
  output->r3 = input->r3;
  // share
  output->share = input->share;
  // options
  output->options = input->options;
  // ps
  output->ps = input->ps;
  // l3_ang
  output->l3_ang = input->l3_ang;
  // l3_dist
  output->l3_dist = input->l3_dist;
  // l3_xx
  output->l3_xx = input->l3_xx;
  // l3_yy
  output->l3_yy = input->l3_yy;
  // r3_ang
  output->r3_ang = input->r3_ang;
  // r3_dist
  output->r3_dist = input->r3_dist;
  // r3_xx
  output->r3_xx = input->r3_xx;
  // r3_yy
  output->r3_yy = input->r3_yy;
  // l2
  output->l2 = input->l2;
  // r2
  output->r2 = input->r2;
  return true;
}

charmie_interfaces__msg__PS4Controller *
charmie_interfaces__msg__PS4Controller__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__PS4Controller * msg = (charmie_interfaces__msg__PS4Controller *)allocator.allocate(sizeof(charmie_interfaces__msg__PS4Controller), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__PS4Controller));
  bool success = charmie_interfaces__msg__PS4Controller__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__PS4Controller__destroy(charmie_interfaces__msg__PS4Controller * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__PS4Controller__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__PS4Controller__Sequence__init(charmie_interfaces__msg__PS4Controller__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__PS4Controller * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__PS4Controller *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__PS4Controller), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__PS4Controller__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__PS4Controller__fini(&data[i - 1]);
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
charmie_interfaces__msg__PS4Controller__Sequence__fini(charmie_interfaces__msg__PS4Controller__Sequence * array)
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
      charmie_interfaces__msg__PS4Controller__fini(&array->data[i]);
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

charmie_interfaces__msg__PS4Controller__Sequence *
charmie_interfaces__msg__PS4Controller__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__PS4Controller__Sequence * array = (charmie_interfaces__msg__PS4Controller__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__PS4Controller__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__PS4Controller__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__PS4Controller__Sequence__destroy(charmie_interfaces__msg__PS4Controller__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__PS4Controller__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__PS4Controller__Sequence__are_equal(const charmie_interfaces__msg__PS4Controller__Sequence * lhs, const charmie_interfaces__msg__PS4Controller__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__PS4Controller__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__PS4Controller__Sequence__copy(
  const charmie_interfaces__msg__PS4Controller__Sequence * input,
  charmie_interfaces__msg__PS4Controller__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__PS4Controller);
    charmie_interfaces__msg__PS4Controller * data =
      (charmie_interfaces__msg__PS4Controller *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__PS4Controller__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__PS4Controller__fini(&data[i]);
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
    if (!charmie_interfaces__msg__PS4Controller__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
