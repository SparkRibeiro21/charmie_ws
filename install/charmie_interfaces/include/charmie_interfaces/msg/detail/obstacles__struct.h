// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/Obstacles.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLES__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'obstacles'
#include "charmie_interfaces/msg/detail/obstacle_info__struct.h"

// Struct defined in msg/Obstacles in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__Obstacles
{
  uint16_t no_obstacles;
  charmie_interfaces__msg__ObstacleInfo__Sequence obstacles;
} charmie_interfaces__msg__Obstacles;

// Struct for a sequence of charmie_interfaces__msg__Obstacles.
typedef struct charmie_interfaces__msg__Obstacles__Sequence
{
  charmie_interfaces__msg__Obstacles * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__Obstacles__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLES__STRUCT_H_
