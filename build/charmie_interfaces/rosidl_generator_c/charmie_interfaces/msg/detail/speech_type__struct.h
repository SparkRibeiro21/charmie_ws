// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/SpeechType.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__SPEECH_TYPE__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__SPEECH_TYPE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/SpeechType in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__SpeechType
{
  bool yes_or_no;
  bool receptionist;
  bool gpsr;
  bool restaurant;
} charmie_interfaces__msg__SpeechType;

// Struct for a sequence of charmie_interfaces__msg__SpeechType.
typedef struct charmie_interfaces__msg__SpeechType__Sequence
{
  charmie_interfaces__msg__SpeechType * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__SpeechType__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__SPEECH_TYPE__STRUCT_H_
