// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from charmie_interfaces:msg/DetectedPerson.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__DETECTED_PERSON__STRUCT_H_
#define CHARMIE_INTERFACES__MSG__DETAIL__DETECTED_PERSON__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/DetectedPerson in the package charmie_interfaces.
typedef struct charmie_interfaces__msg__DetectedPerson
{
  int32_t index_person;
  float conf_person;
  float x_rel;
  float y_rel;
  int32_t box_top_left_x;
  int32_t box_top_left_y;
  int32_t box_width;
  int32_t box_height;
  int32_t kp_nose_x;
  int32_t kp_nose_y;
  float kp_nose_conf;
  int32_t kp_eye_left_x;
  int32_t kp_eye_left_y;
  float kp_eye_left_conf;
  int32_t kp_eye_right_x;
  int32_t kp_eye_right_y;
  float kp_eye_right_conf;
  int32_t kp_ear_left_x;
  int32_t kp_ear_left_y;
  float kp_ear_left_conf;
  int32_t kp_ear_right_x;
  int32_t kp_ear_right_y;
  float kp_ear_right_conf;
  int32_t kp_shoulder_left_x;
  int32_t kp_shoulder_left_y;
  float kp_shoulder_left_conf;
  int32_t kp_shoulder_right_x;
  int32_t kp_shoulder_right_y;
  float kp_shoulder_right_conf;
  int32_t kp_elbow_left_x;
  int32_t kp_elbow_left_y;
  float kp_elbow_left_conf;
  int32_t kp_elbow_right_x;
  int32_t kp_elbow_right_y;
  float kp_elbow_right_conf;
  int32_t kp_wrist_left_x;
  int32_t kp_wrist_left_y;
  float kp_wrist_left_conf;
  int32_t kp_wrist_right_x;
  int32_t kp_wrist_right_y;
  float kp_wrist_right_conf;
  int32_t kp_hip_left_x;
  int32_t kp_hip_left_y;
  float kp_hip_left_conf;
  int32_t kp_hip_right_x;
  int32_t kp_hip_right_y;
  float kp_hip_right_conf;
  int32_t kp_knee_left_x;
  int32_t kp_knee_left_y;
  float kp_knee_left_conf;
  int32_t kp_knee_right_x;
  int32_t kp_knee_right_y;
  float kp_knee_right_conf;
  int32_t kp_ankle_left_x;
  int32_t kp_ankle_left_y;
  float kp_ankle_left_conf;
  int32_t kp_ankle_right_x;
  int32_t kp_ankle_right_y;
  float kp_ankle_right_conf;
} charmie_interfaces__msg__DetectedPerson;

// Struct for a sequence of charmie_interfaces__msg__DetectedPerson.
typedef struct charmie_interfaces__msg__DetectedPerson__Sequence
{
  charmie_interfaces__msg__DetectedPerson * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} charmie_interfaces__msg__DetectedPerson__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__DETECTED_PERSON__STRUCT_H_
