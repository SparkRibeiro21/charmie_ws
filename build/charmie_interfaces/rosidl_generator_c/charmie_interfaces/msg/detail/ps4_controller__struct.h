// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/PS4Controller.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__PS4_CONTROLLER__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__PS4_CONTROLLER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/PS4Controller in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__PS4Controller
{
  uint8_t triangle;
  uint8_t circle;
  uint8_t cross;
  uint8_t square;
  uint8_t arrow_up;
  uint8_t arrow_right;
  uint8_t arrow_down;
  uint8_t arrow_left;
  uint8_t l1;
  uint8_t r1;
  uint8_t l3;
  uint8_t r3;
  uint8_t share;
  uint8_t options;
  uint8_t ps;
  float l3_ang;
  float l3_dist;
  float l3_xx;
  float l3_yy;
  float r3_ang;
  float r3_dist;
  float r3_xx;
  float r3_yy;
  float l2;
  float r2;
} charmie_interfaces__msg__PS4Controller;

// Struct for a sequence of charmie_interfaces__msg__PS4Controller.
typedef struct charmie_interfaces__msg__PS4Controller__Sequence
{
  charmie_interfaces__msg__PS4Controller * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__PS4Controller__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__PS4_CONTROLLER__STRUCT_H_
