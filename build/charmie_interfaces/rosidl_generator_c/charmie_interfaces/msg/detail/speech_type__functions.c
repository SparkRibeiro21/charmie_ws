// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/SpeechType.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/speech_type__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
charmie_interfaces__msg__SpeechType__init(charmie_interfaces__msg__SpeechType * msg)
{
  if (!msg) {
    return false;
  }
  // yes_or_no
  // receptionist
  // gpsr
  // restaurant
  return true;
}

void
charmie_interfaces__msg__SpeechType__fini(charmie_interfaces__msg__SpeechType * msg)
{
  if (!msg) {
    return;
  }
  // yes_or_no
  // receptionist
  // gpsr
  // restaurant
}

bool
charmie_interfaces__msg__SpeechType__are_equal(const charmie_interfaces__msg__SpeechType * lhs, const charmie_interfaces__msg__SpeechType * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // yes_or_no
  if (lhs->yes_or_no != rhs->yes_or_no) {
    return false;
  }
  // receptionist
  if (lhs->receptionist != rhs->receptionist) {
    return false;
  }
  // gpsr
  if (lhs->gpsr != rhs->gpsr) {
    return false;
  }
  // restaurant
  if (lhs->restaurant != rhs->restaurant) {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__SpeechType__copy(
  const charmie_interfaces__msg__SpeechType * input,
  charmie_interfaces__msg__SpeechType * output)
{
  if (!input || !output) {
    return false;
  }
  // yes_or_no
  output->yes_or_no = input->yes_or_no;
  // receptionist
  output->receptionist = input->receptionist;
  // gpsr
  output->gpsr = input->gpsr;
  // restaurant
  output->restaurant = input->restaurant;
  return true;
}

charmie_interfaces__msg__SpeechType *
charmie_interfaces__msg__SpeechType__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__SpeechType * msg = (charmie_interfaces__msg__SpeechType *)allocator.allocate(sizeof(charmie_interfaces__msg__SpeechType), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__SpeechType));
  bool success = charmie_interfaces__msg__SpeechType__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__SpeechType__destroy(charmie_interfaces__msg__SpeechType * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__SpeechType__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__SpeechType__Sequence__init(charmie_interfaces__msg__SpeechType__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__SpeechType * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__SpeechType *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__SpeechType), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__SpeechType__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__SpeechType__fini(&data[i - 1]);
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
charmie_interfaces__msg__SpeechType__Sequence__fini(charmie_interfaces__msg__SpeechType__Sequence * array)
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
      charmie_interfaces__msg__SpeechType__fini(&array->data[i]);
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

charmie_interfaces__msg__SpeechType__Sequence *
charmie_interfaces__msg__SpeechType__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__SpeechType__Sequence * array = (charmie_interfaces__msg__SpeechType__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__SpeechType__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__SpeechType__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__SpeechType__Sequence__destroy(charmie_interfaces__msg__SpeechType__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__SpeechType__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__SpeechType__Sequence__are_equal(const charmie_interfaces__msg__SpeechType__Sequence * lhs, const charmie_interfaces__msg__SpeechType__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__SpeechType__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__SpeechType__Sequence__copy(
  const charmie_interfaces__msg__SpeechType__Sequence * input,
  charmie_interfaces__msg__SpeechType__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__SpeechType);
    charmie_interfaces__msg__SpeechType * data =
      (charmie_interfaces__msg__SpeechType *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__SpeechType__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__SpeechType__fini(&data[i]);
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
    if (!charmie_interfaces__msg__SpeechType__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
