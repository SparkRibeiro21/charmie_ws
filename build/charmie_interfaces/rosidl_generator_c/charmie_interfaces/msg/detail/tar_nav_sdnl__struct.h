// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/TarNavSDNL.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'move_target_coordinates'
// Member 'rotate_target_coordinates'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"

// Struct defined in msg/TarNavSDNL in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__TarNavSDNL
{
  geometry_msgs__msg__Pose2D move_target_coordinates;
  geometry_msgs__msg__Pose2D rotate_target_coordinates;
  bool flag_not_obs;
} charmie_interfaces__msg__TarNavSDNL;

// Struct for a sequence of charmie_interfaces__msg__TarNavSDNL.
typedef struct charmie_interfaces__msg__TarNavSDNL__Sequence
{
  charmie_interfaces__msg__TarNavSDNL * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__TarNavSDNL__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__STRUCT_H_
