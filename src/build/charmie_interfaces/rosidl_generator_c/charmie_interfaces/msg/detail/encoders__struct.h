// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/Encoders in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__Encoders
{
  uint32_t enc_m1;
  uint32_t enc_m2;
  uint32_t enc_m3;
  uint32_t enc_m4;
} charmie_interfaces__msg__Encoders;

// Struct for a sequence of charmie_interfaces__msg__Encoders.
typedef struct charmie_interfaces__msg__Encoders__Sequence
{
  charmie_interfaces__msg__Encoders * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__Encoders__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_H_
