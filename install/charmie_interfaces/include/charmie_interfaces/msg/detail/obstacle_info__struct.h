// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/ObstacleInfo.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLE_INFO__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLE_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/ObstacleInfo in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__ObstacleInfo
{
  float alfa;
  float dist;
  float length_cm;
  float length_degrees;
} charmie_interfaces__msg__ObstacleInfo;

// Struct for a sequence of charmie_interfaces__msg__ObstacleInfo.
typedef struct charmie_interfaces__msg__ObstacleInfo__Sequence
{
  charmie_interfaces__msg__ObstacleInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__ObstacleInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLE_INFO__STRUCT_H_
