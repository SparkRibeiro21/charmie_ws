// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/RobotSpeech.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__ROBOT_SPEECH__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__ROBOT_SPEECH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
// Member 'language'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/RobotSpeech in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__RobotSpeech
{
  rosidl_runtime_c__String command;
  rosidl_runtime_c__String language;
} charmie_interfaces__msg__RobotSpeech;

// Struct for a sequence of charmie_interfaces__msg__RobotSpeech.
typedef struct charmie_interfaces__msg__RobotSpeech__Sequence
{
  charmie_interfaces__msg__RobotSpeech * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__RobotSpeech__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__ROBOT_SPEECH__STRUCT_H_
