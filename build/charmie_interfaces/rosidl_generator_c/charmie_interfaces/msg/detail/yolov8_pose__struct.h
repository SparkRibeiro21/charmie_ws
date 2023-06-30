// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/Yolov8Pose.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'keypoints'
#include "charmie_interfaces/msg/detail/keypoints__struct.h"

// Struct defined in msg/Yolov8Pose in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__Yolov8Pose
{
  int32_t num_person;
  charmie_interfaces__msg__Keypoints__Sequence keypoints;
} charmie_interfaces__msg__Yolov8Pose;

// Struct for a sequence of charmie_interfaces__msg__Yolov8Pose.
typedef struct charmie_interfaces__msg__Yolov8Pose__Sequence
{
  charmie_interfaces__msg__Yolov8Pose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__Yolov8Pose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__STRUCT_H_
