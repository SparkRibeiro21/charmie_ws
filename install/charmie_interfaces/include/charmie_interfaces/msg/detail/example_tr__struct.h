// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/ExampleTR.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__EXAMPLE_TR__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__EXAMPLE_TR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'coordinates'
#include "geometry_msgs/msg/detail/point__struct.h"

// Struct defined in msg/ExampleTR in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__ExampleTR
{
  rosidl_runtime_c__String name;
  geometry_msgs__msg__Point coordinates;
} charmie_interfaces__msg__ExampleTR;

// Struct for a sequence of charmie_interfaces__msg__ExampleTR.
typedef struct charmie_interfaces__msg__ExampleTR__Sequence
{
  charmie_interfaces__msg__ExampleTR * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__ExampleTR__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__EXAMPLE_TR__STRUCT_H_
