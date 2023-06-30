// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/Keypoints.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__KEYPOINTS__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__KEYPOINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Keypoints in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__Keypoints
{
  int32_t index_person;
  float average_distance;
  float standard_deviation;
  int32_t key_p0_x;
  int32_t key_p0_y;
  int32_t key_p1_x;
  int32_t key_p1_y;
  int32_t key_p2_x;
  int32_t key_p2_y;
  int32_t key_p3_x;
  int32_t key_p3_y;
  int32_t key_p4_x;
  int32_t key_p4_y;
  int32_t key_p5_x;
  int32_t key_p5_y;
  int32_t key_p6_x;
  int32_t key_p6_y;
  int32_t key_p7_x;
  int32_t key_p7_y;
  int32_t key_p8_x;
  int32_t key_p8_y;
  int32_t key_p9_x;
  int32_t key_p9_y;
  int32_t key_p10_x;
  int32_t key_p10_y;
  int32_t key_p11_x;
  int32_t key_p11_y;
  int32_t key_p12_x;
  int32_t key_p12_y;
  int32_t key_p13_x;
  int32_t key_p13_y;
  int32_t key_p14_x;
  int32_t key_p14_y;
  int32_t key_p15_x;
  int32_t key_p15_y;
  int32_t key_p16_x;
  int32_t key_p16_y;
} charmie_interfaces__msg__Keypoints;

// Struct for a sequence of charmie_interfaces__msg__Keypoints.
typedef struct charmie_interfaces__msg__Keypoints__Sequence
{
  charmie_interfaces__msg__Keypoints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__Keypoints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__KEYPOINTS__STRUCT_H_
