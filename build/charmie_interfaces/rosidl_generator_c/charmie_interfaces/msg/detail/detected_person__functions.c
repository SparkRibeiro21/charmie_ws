// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from charmie_interfaces:msg/DetectedPerson.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/detected_person__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
charmie_interfaces__msg__DetectedPerson__init(charmie_interfaces__msg__DetectedPerson * msg)
{
  if (!msg) {
    return false;
  }
  // index_person
  // conf_person
  // x_rel
  // y_rel
  // box_top_left_x
  // box_top_left_y
  // box_width
  // box_height
  // kp_nose_x
  // kp_nose_y
  // kp_nose_conf
  // kp_eye_left_x
  // kp_eye_left_y
  // kp_eye_left_conf
  // kp_eye_right_x
  // kp_eye_right_y
  // kp_eye_right_conf
  // kp_ear_left_x
  // kp_ear_left_y
  // kp_ear_left_conf
  // kp_ear_right_x
  // kp_ear_right_y
  // kp_ear_right_conf
  // kp_shoulder_left_x
  // kp_shoulder_left_y
  // kp_shoulder_left_conf
  // kp_shoulder_right_x
  // kp_shoulder_right_y
  // kp_shoulder_right_conf
  // kp_elbow_left_x
  // kp_elbow_left_y
  // kp_elbow_left_conf
  // kp_elbow_right_x
  // kp_elbow_right_y
  // kp_elbow_right_conf
  // kp_wrist_left_x
  // kp_wrist_left_y
  // kp_wrist_left_conf
  // kp_wrist_right_x
  // kp_wrist_right_y
  // kp_wrist_right_conf
  // kp_hip_left_x
  // kp_hip_left_y
  // kp_hip_left_conf
  // kp_hip_right_x
  // kp_hip_right_y
  // kp_hip_right_conf
  // kp_knee_left_x
  // kp_knee_left_y
  // kp_knee_left_conf
  // kp_knee_right_x
  // kp_knee_right_y
  // kp_knee_right_conf
  // kp_ankle_left_x
  // kp_ankle_left_y
  // kp_ankle_left_conf
  // kp_ankle_right_x
  // kp_ankle_right_y
  // kp_ankle_right_conf
  return true;
}

void
charmie_interfaces__msg__DetectedPerson__fini(charmie_interfaces__msg__DetectedPerson * msg)
{
  if (!msg) {
    return;
  }
  // index_person
  // conf_person
  // x_rel
  // y_rel
  // box_top_left_x
  // box_top_left_y
  // box_width
  // box_height
  // kp_nose_x
  // kp_nose_y
  // kp_nose_conf
  // kp_eye_left_x
  // kp_eye_left_y
  // kp_eye_left_conf
  // kp_eye_right_x
  // kp_eye_right_y
  // kp_eye_right_conf
  // kp_ear_left_x
  // kp_ear_left_y
  // kp_ear_left_conf
  // kp_ear_right_x
  // kp_ear_right_y
  // kp_ear_right_conf
  // kp_shoulder_left_x
  // kp_shoulder_left_y
  // kp_shoulder_left_conf
  // kp_shoulder_right_x
  // kp_shoulder_right_y
  // kp_shoulder_right_conf
  // kp_elbow_left_x
  // kp_elbow_left_y
  // kp_elbow_left_conf
  // kp_elbow_right_x
  // kp_elbow_right_y
  // kp_elbow_right_conf
  // kp_wrist_left_x
  // kp_wrist_left_y
  // kp_wrist_left_conf
  // kp_wrist_right_x
  // kp_wrist_right_y
  // kp_wrist_right_conf
  // kp_hip_left_x
  // kp_hip_left_y
  // kp_hip_left_conf
  // kp_hip_right_x
  // kp_hip_right_y
  // kp_hip_right_conf
  // kp_knee_left_x
  // kp_knee_left_y
  // kp_knee_left_conf
  // kp_knee_right_x
  // kp_knee_right_y
  // kp_knee_right_conf
  // kp_ankle_left_x
  // kp_ankle_left_y
  // kp_ankle_left_conf
  // kp_ankle_right_x
  // kp_ankle_right_y
  // kp_ankle_right_conf
}

bool
charmie_interfaces__msg__DetectedPerson__are_equal(const charmie_interfaces__msg__DetectedPerson * lhs, const charmie_interfaces__msg__DetectedPerson * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // index_person
  if (lhs->index_person != rhs->index_person) {
    return false;
  }
  // conf_person
  if (lhs->conf_person != rhs->conf_person) {
    return false;
  }
  // x_rel
  if (lhs->x_rel != rhs->x_rel) {
    return false;
  }
  // y_rel
  if (lhs->y_rel != rhs->y_rel) {
    return false;
  }
  // box_top_left_x
  if (lhs->box_top_left_x != rhs->box_top_left_x) {
    return false;
  }
  // box_top_left_y
  if (lhs->box_top_left_y != rhs->box_top_left_y) {
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
  // kp_nose_x
  if (lhs->kp_nose_x != rhs->kp_nose_x) {
    return false;
  }
  // kp_nose_y
  if (lhs->kp_nose_y != rhs->kp_nose_y) {
    return false;
  }
  // kp_nose_conf
  if (lhs->kp_nose_conf != rhs->kp_nose_conf) {
    return false;
  }
  // kp_eye_left_x
  if (lhs->kp_eye_left_x != rhs->kp_eye_left_x) {
    return false;
  }
  // kp_eye_left_y
  if (lhs->kp_eye_left_y != rhs->kp_eye_left_y) {
    return false;
  }
  // kp_eye_left_conf
  if (lhs->kp_eye_left_conf != rhs->kp_eye_left_conf) {
    return false;
  }
  // kp_eye_right_x
  if (lhs->kp_eye_right_x != rhs->kp_eye_right_x) {
    return false;
  }
  // kp_eye_right_y
  if (lhs->kp_eye_right_y != rhs->kp_eye_right_y) {
    return false;
  }
  // kp_eye_right_conf
  if (lhs->kp_eye_right_conf != rhs->kp_eye_right_conf) {
    return false;
  }
  // kp_ear_left_x
  if (lhs->kp_ear_left_x != rhs->kp_ear_left_x) {
    return false;
  }
  // kp_ear_left_y
  if (lhs->kp_ear_left_y != rhs->kp_ear_left_y) {
    return false;
  }
  // kp_ear_left_conf
  if (lhs->kp_ear_left_conf != rhs->kp_ear_left_conf) {
    return false;
  }
  // kp_ear_right_x
  if (lhs->kp_ear_right_x != rhs->kp_ear_right_x) {
    return false;
  }
  // kp_ear_right_y
  if (lhs->kp_ear_right_y != rhs->kp_ear_right_y) {
    return false;
  }
  // kp_ear_right_conf
  if (lhs->kp_ear_right_conf != rhs->kp_ear_right_conf) {
    return false;
  }
  // kp_shoulder_left_x
  if (lhs->kp_shoulder_left_x != rhs->kp_shoulder_left_x) {
    return false;
  }
  // kp_shoulder_left_y
  if (lhs->kp_shoulder_left_y != rhs->kp_shoulder_left_y) {
    return false;
  }
  // kp_shoulder_left_conf
  if (lhs->kp_shoulder_left_conf != rhs->kp_shoulder_left_conf) {
    return false;
  }
  // kp_shoulder_right_x
  if (lhs->kp_shoulder_right_x != rhs->kp_shoulder_right_x) {
    return false;
  }
  // kp_shoulder_right_y
  if (lhs->kp_shoulder_right_y != rhs->kp_shoulder_right_y) {
    return false;
  }
  // kp_shoulder_right_conf
  if (lhs->kp_shoulder_right_conf != rhs->kp_shoulder_right_conf) {
    return false;
  }
  // kp_elbow_left_x
  if (lhs->kp_elbow_left_x != rhs->kp_elbow_left_x) {
    return false;
  }
  // kp_elbow_left_y
  if (lhs->kp_elbow_left_y != rhs->kp_elbow_left_y) {
    return false;
  }
  // kp_elbow_left_conf
  if (lhs->kp_elbow_left_conf != rhs->kp_elbow_left_conf) {
    return false;
  }
  // kp_elbow_right_x
  if (lhs->kp_elbow_right_x != rhs->kp_elbow_right_x) {
    return false;
  }
  // kp_elbow_right_y
  if (lhs->kp_elbow_right_y != rhs->kp_elbow_right_y) {
    return false;
  }
  // kp_elbow_right_conf
  if (lhs->kp_elbow_right_conf != rhs->kp_elbow_right_conf) {
    return false;
  }
  // kp_wrist_left_x
  if (lhs->kp_wrist_left_x != rhs->kp_wrist_left_x) {
    return false;
  }
  // kp_wrist_left_y
  if (lhs->kp_wrist_left_y != rhs->kp_wrist_left_y) {
    return false;
  }
  // kp_wrist_left_conf
  if (lhs->kp_wrist_left_conf != rhs->kp_wrist_left_conf) {
    return false;
  }
  // kp_wrist_right_x
  if (lhs->kp_wrist_right_x != rhs->kp_wrist_right_x) {
    return false;
  }
  // kp_wrist_right_y
  if (lhs->kp_wrist_right_y != rhs->kp_wrist_right_y) {
    return false;
  }
  // kp_wrist_right_conf
  if (lhs->kp_wrist_right_conf != rhs->kp_wrist_right_conf) {
    return false;
  }
  // kp_hip_left_x
  if (lhs->kp_hip_left_x != rhs->kp_hip_left_x) {
    return false;
  }
  // kp_hip_left_y
  if (lhs->kp_hip_left_y != rhs->kp_hip_left_y) {
    return false;
  }
  // kp_hip_left_conf
  if (lhs->kp_hip_left_conf != rhs->kp_hip_left_conf) {
    return false;
  }
  // kp_hip_right_x
  if (lhs->kp_hip_right_x != rhs->kp_hip_right_x) {
    return false;
  }
  // kp_hip_right_y
  if (lhs->kp_hip_right_y != rhs->kp_hip_right_y) {
    return false;
  }
  // kp_hip_right_conf
  if (lhs->kp_hip_right_conf != rhs->kp_hip_right_conf) {
    return false;
  }
  // kp_knee_left_x
  if (lhs->kp_knee_left_x != rhs->kp_knee_left_x) {
    return false;
  }
  // kp_knee_left_y
  if (lhs->kp_knee_left_y != rhs->kp_knee_left_y) {
    return false;
  }
  // kp_knee_left_conf
  if (lhs->kp_knee_left_conf != rhs->kp_knee_left_conf) {
    return false;
  }
  // kp_knee_right_x
  if (lhs->kp_knee_right_x != rhs->kp_knee_right_x) {
    return false;
  }
  // kp_knee_right_y
  if (lhs->kp_knee_right_y != rhs->kp_knee_right_y) {
    return false;
  }
  // kp_knee_right_conf
  if (lhs->kp_knee_right_conf != rhs->kp_knee_right_conf) {
    return false;
  }
  // kp_ankle_left_x
  if (lhs->kp_ankle_left_x != rhs->kp_ankle_left_x) {
    return false;
  }
  // kp_ankle_left_y
  if (lhs->kp_ankle_left_y != rhs->kp_ankle_left_y) {
    return false;
  }
  // kp_ankle_left_conf
  if (lhs->kp_ankle_left_conf != rhs->kp_ankle_left_conf) {
    return false;
  }
  // kp_ankle_right_x
  if (lhs->kp_ankle_right_x != rhs->kp_ankle_right_x) {
    return false;
  }
  // kp_ankle_right_y
  if (lhs->kp_ankle_right_y != rhs->kp_ankle_right_y) {
    return false;
  }
  // kp_ankle_right_conf
  if (lhs->kp_ankle_right_conf != rhs->kp_ankle_right_conf) {
    return false;
  }
  return true;
}

bool
charmie_interfaces__msg__DetectedPerson__copy(
  const charmie_interfaces__msg__DetectedPerson * input,
  charmie_interfaces__msg__DetectedPerson * output)
{
  if (!input || !output) {
    return false;
  }
  // index_person
  output->index_person = input->index_person;
  // conf_person
  output->conf_person = input->conf_person;
  // x_rel
  output->x_rel = input->x_rel;
  // y_rel
  output->y_rel = input->y_rel;
  // box_top_left_x
  output->box_top_left_x = input->box_top_left_x;
  // box_top_left_y
  output->box_top_left_y = input->box_top_left_y;
  // box_width
  output->box_width = input->box_width;
  // box_height
  output->box_height = input->box_height;
  // kp_nose_x
  output->kp_nose_x = input->kp_nose_x;
  // kp_nose_y
  output->kp_nose_y = input->kp_nose_y;
  // kp_nose_conf
  output->kp_nose_conf = input->kp_nose_conf;
  // kp_eye_left_x
  output->kp_eye_left_x = input->kp_eye_left_x;
  // kp_eye_left_y
  output->kp_eye_left_y = input->kp_eye_left_y;
  // kp_eye_left_conf
  output->kp_eye_left_conf = input->kp_eye_left_conf;
  // kp_eye_right_x
  output->kp_eye_right_x = input->kp_eye_right_x;
  // kp_eye_right_y
  output->kp_eye_right_y = input->kp_eye_right_y;
  // kp_eye_right_conf
  output->kp_eye_right_conf = input->kp_eye_right_conf;
  // kp_ear_left_x
  output->kp_ear_left_x = input->kp_ear_left_x;
  // kp_ear_left_y
  output->kp_ear_left_y = input->kp_ear_left_y;
  // kp_ear_left_conf
  output->kp_ear_left_conf = input->kp_ear_left_conf;
  // kp_ear_right_x
  output->kp_ear_right_x = input->kp_ear_right_x;
  // kp_ear_right_y
  output->kp_ear_right_y = input->kp_ear_right_y;
  // kp_ear_right_conf
  output->kp_ear_right_conf = input->kp_ear_right_conf;
  // kp_shoulder_left_x
  output->kp_shoulder_left_x = input->kp_shoulder_left_x;
  // kp_shoulder_left_y
  output->kp_shoulder_left_y = input->kp_shoulder_left_y;
  // kp_shoulder_left_conf
  output->kp_shoulder_left_conf = input->kp_shoulder_left_conf;
  // kp_shoulder_right_x
  output->kp_shoulder_right_x = input->kp_shoulder_right_x;
  // kp_shoulder_right_y
  output->kp_shoulder_right_y = input->kp_shoulder_right_y;
  // kp_shoulder_right_conf
  output->kp_shoulder_right_conf = input->kp_shoulder_right_conf;
  // kp_elbow_left_x
  output->kp_elbow_left_x = input->kp_elbow_left_x;
  // kp_elbow_left_y
  output->kp_elbow_left_y = input->kp_elbow_left_y;
  // kp_elbow_left_conf
  output->kp_elbow_left_conf = input->kp_elbow_left_conf;
  // kp_elbow_right_x
  output->kp_elbow_right_x = input->kp_elbow_right_x;
  // kp_elbow_right_y
  output->kp_elbow_right_y = input->kp_elbow_right_y;
  // kp_elbow_right_conf
  output->kp_elbow_right_conf = input->kp_elbow_right_conf;
  // kp_wrist_left_x
  output->kp_wrist_left_x = input->kp_wrist_left_x;
  // kp_wrist_left_y
  output->kp_wrist_left_y = input->kp_wrist_left_y;
  // kp_wrist_left_conf
  output->kp_wrist_left_conf = input->kp_wrist_left_conf;
  // kp_wrist_right_x
  output->kp_wrist_right_x = input->kp_wrist_right_x;
  // kp_wrist_right_y
  output->kp_wrist_right_y = input->kp_wrist_right_y;
  // kp_wrist_right_conf
  output->kp_wrist_right_conf = input->kp_wrist_right_conf;
  // kp_hip_left_x
  output->kp_hip_left_x = input->kp_hip_left_x;
  // kp_hip_left_y
  output->kp_hip_left_y = input->kp_hip_left_y;
  // kp_hip_left_conf
  output->kp_hip_left_conf = input->kp_hip_left_conf;
  // kp_hip_right_x
  output->kp_hip_right_x = input->kp_hip_right_x;
  // kp_hip_right_y
  output->kp_hip_right_y = input->kp_hip_right_y;
  // kp_hip_right_conf
  output->kp_hip_right_conf = input->kp_hip_right_conf;
  // kp_knee_left_x
  output->kp_knee_left_x = input->kp_knee_left_x;
  // kp_knee_left_y
  output->kp_knee_left_y = input->kp_knee_left_y;
  // kp_knee_left_conf
  output->kp_knee_left_conf = input->kp_knee_left_conf;
  // kp_knee_right_x
  output->kp_knee_right_x = input->kp_knee_right_x;
  // kp_knee_right_y
  output->kp_knee_right_y = input->kp_knee_right_y;
  // kp_knee_right_conf
  output->kp_knee_right_conf = input->kp_knee_right_conf;
  // kp_ankle_left_x
  output->kp_ankle_left_x = input->kp_ankle_left_x;
  // kp_ankle_left_y
  output->kp_ankle_left_y = input->kp_ankle_left_y;
  // kp_ankle_left_conf
  output->kp_ankle_left_conf = input->kp_ankle_left_conf;
  // kp_ankle_right_x
  output->kp_ankle_right_x = input->kp_ankle_right_x;
  // kp_ankle_right_y
  output->kp_ankle_right_y = input->kp_ankle_right_y;
  // kp_ankle_right_conf
  output->kp_ankle_right_conf = input->kp_ankle_right_conf;
  return true;
}

charmie_interfaces__msg__DetectedPerson *
charmie_interfaces__msg__DetectedPerson__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__DetectedPerson * msg = (charmie_interfaces__msg__DetectedPerson *)allocator.allocate(sizeof(charmie_interfaces__msg__DetectedPerson), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(charmie_interfaces__msg__DetectedPerson));
  bool success = charmie_interfaces__msg__DetectedPerson__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
charmie_interfaces__msg__DetectedPerson__destroy(charmie_interfaces__msg__DetectedPerson * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    charmie_interfaces__msg__DetectedPerson__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
charmie_interfaces__msg__DetectedPerson__Sequence__init(charmie_interfaces__msg__DetectedPerson__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__DetectedPerson * data = NULL;

  if (size) {
    data = (charmie_interfaces__msg__DetectedPerson *)allocator.zero_allocate(size, sizeof(charmie_interfaces__msg__DetectedPerson), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = charmie_interfaces__msg__DetectedPerson__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        charmie_interfaces__msg__DetectedPerson__fini(&data[i - 1]);
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
charmie_interfaces__msg__DetectedPerson__Sequence__fini(charmie_interfaces__msg__DetectedPerson__Sequence * array)
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
      charmie_interfaces__msg__DetectedPerson__fini(&array->data[i]);
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

charmie_interfaces__msg__DetectedPerson__Sequence *
charmie_interfaces__msg__DetectedPerson__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  charmie_interfaces__msg__DetectedPerson__Sequence * array = (charmie_interfaces__msg__DetectedPerson__Sequence *)allocator.allocate(sizeof(charmie_interfaces__msg__DetectedPerson__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = charmie_interfaces__msg__DetectedPerson__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
charmie_interfaces__msg__DetectedPerson__Sequence__destroy(charmie_interfaces__msg__DetectedPerson__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    charmie_interfaces__msg__DetectedPerson__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
charmie_interfaces__msg__DetectedPerson__Sequence__are_equal(const charmie_interfaces__msg__DetectedPerson__Sequence * lhs, const charmie_interfaces__msg__DetectedPerson__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!charmie_interfaces__msg__DetectedPerson__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
charmie_interfaces__msg__DetectedPerson__Sequence__copy(
  const charmie_interfaces__msg__DetectedPerson__Sequence * input,
  charmie_interfaces__msg__DetectedPerson__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(charmie_interfaces__msg__DetectedPerson);
    charmie_interfaces__msg__DetectedPerson * data =
      (charmie_interfaces__msg__DetectedPerson *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!charmie_interfaces__msg__DetectedPerson__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          charmie_interfaces__msg__DetectedPerson__fini(&data[i]);
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
    if (!charmie_interfaces__msg__DetectedPerson__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
