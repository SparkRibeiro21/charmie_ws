// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/encoders__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
charmie_interfaces__msg__Encoders__init(charmie_interfaces__msg__Encoders * msg)
{
  if (!msg) {
    return false;
  }
  // enc_m1
  // enc_m2
  // enc_m3
  // enc_m4
  return true;
}

void
charmie_interfaces__msg__Encoders__fini(charmie_interfaces__msg__Encoders * msg)
{
  if (!msg) {
    return;
  }
  // enc_m1
  // enc_m2
  // enc_m3
  // enc_m4
}

bool
charmie_interfaces__msg__Encoders__are_equal(const charmie_interfaces__msg__Encoders * lhs, const charmie_interfaces__msg__Encoders * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // enc_m1
  if (lhs->enc_m1 != rhs->enc_m1) {
    return false;
  }
  // enc_m2
  if (lhs->enc_m2 != rhs->enc_m2) {
    return false;
  }
  // enc_m3
  if (lhs->enc_m3 != rhs->enc_m3) {
    return false;
  }
  // enc_m4
  if (lhs->enc_m4 != rhs->enc_m4) {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__Encoders__copy(
  const charmie_interfaces__msg__Encoders * input,
  charmie_interfaces__msg__Encoders * output)
{
  if (!input || !output) {
    return false;
  }
  // enc_m1
  output->enc_m1 = input->enc_m1;
  // enc_m2
  output->enc_m2 = input->enc_m2;
  // enc_m3
  output->enc_m3 = input->enc_m3;
  // enc_m4
  output->enc_m4 = input->enc_m4;
  return true;
}

charmie_interfaces__msg__Encoders *
charmie_interfaces__msg__Encoders__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Encoders * msg = (charmie_interfaces__msg__Encoders *)allocator.allocate(sizeof(charmie_interfaces__msg__Encoders), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__Encoders));
  bool success = charmie_interfaces__msg__Encoders__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__Encoders__destroy(charmie_interfaces__msg__Encoders * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__Encoders__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__Encoders__Sequence__init(charmie_interfaces__msg__Encoders__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Encoders * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__Encoders *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__Encoders), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__Encoders__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__Encoders__fini(&data[i - 1]);
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
charmie_interfaces__msg__Encoders__Sequence__fini(charmie_interfaces__msg__Encoders__Sequence * array)
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
      charmie_interfaces__msg__Encoders__fini(&array->data[i]);
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

charmie_interfaces__msg__Encoders__Sequence *
charmie_interfaces__msg__Encoders__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__Encoders__Sequence * array = (charmie_interfaces__msg__Encoders__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__Encoders__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__Encoders__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__Encoders__Sequence__destroy(charmie_interfaces__msg__Encoders__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__Encoders__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__Encoders__Sequence__are_equal(const charmie_interfaces__msg__Encoders__Sequence * lhs, const charmie_interfaces__msg__Encoders__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__Encoders__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__Encoders__Sequence__copy(
  const charmie_interfaces__msg__Encoders__Sequence * input,
  charmie_interfaces__msg__Encoders__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__Encoders);
    charmie_interfaces__msg__Encoders * data =
      (charmie_interfaces__msg__Encoders *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__Encoders__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__Encoders__fini(&data[i]);
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
    if (!charmie_interfaces__msg__Encoders__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
