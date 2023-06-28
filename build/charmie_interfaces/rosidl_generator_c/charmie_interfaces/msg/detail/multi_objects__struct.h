// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/MultiObjects.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__MULTI_OBJECTS__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__MULTI_OBJECTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'objects'
#include "rosidl_runtime_c/string.h"
// Member 'confidence'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/MultiObjects in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__MultiObjects
{
  rosidl_runtime_c__String__Sequence objects;
  rosidl_runtime_c__float__Sequence confidence;
} charmie_interfaces__msg__MultiObjects;

// Struct for a sequence of charmie_interfaces__msg__MultiObjects.
typedef struct charmie_interfaces__msg__MultiObjects__Sequence
{
  charmie_interfaces__msg__MultiObjects * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__MultiObjects__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__MULTI_OBJECTS__STRUCT_H_
