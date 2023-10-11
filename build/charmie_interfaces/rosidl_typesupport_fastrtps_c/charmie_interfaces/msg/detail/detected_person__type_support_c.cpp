// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from charmie_interfaces:msg/DetectedPerson.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/detected_person__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "charmie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "charmie_interfaces/msg/detail/detected_person__struct.h"
#include "charmie_interfaces/msg/detail/detected_person__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _DetectedPerson__ros_msg_type = charmie_interfaces__msg__DetectedPerson;

static bool _DetectedPerson__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _DetectedPerson__ros_msg_type * ros_message = static_cast<const _DetectedPerson__ros_msg_type *>(untyped_ros_message);
  // Field name: index_person
  {
    cdr << ros_message->index_person;
  }

  // Field name: conf_person
  {
    cdr << ros_message->conf_person;
  }

  // Field name: x_rel
  {
    cdr << ros_message->x_rel;
  }

  // Field name: y_rel
  {
    cdr << ros_message->y_rel;
  }

  // Field name: box_top_left_x
  {
    cdr << ros_message->box_top_left_x;
  }

  // Field name: box_top_left_y
  {
    cdr << ros_message->box_top_left_y;
  }

  // Field name: box_width
  {
    cdr << ros_message->box_width;
  }

  // Field name: box_height
  {
    cdr << ros_message->box_height;
  }

  // Field name: kp_nose_x
  {
    cdr << ros_message->kp_nose_x;
  }

  // Field name: kp_nose_y
  {
    cdr << ros_message->kp_nose_y;
  }

  // Field name: kp_nose_conf
  {
    cdr << ros_message->kp_nose_conf;
  }

  // Field name: kp_eye_left_x
  {
    cdr << ros_message->kp_eye_left_x;
  }

  // Field name: kp_eye_left_y
  {
    cdr << ros_message->kp_eye_left_y;
  }

  // Field name: kp_eye_left_conf
  {
    cdr << ros_message->kp_eye_left_conf;
  }

  // Field name: kp_eye_right_x
  {
    cdr << ros_message->kp_eye_right_x;
  }

  // Field name: kp_eye_right_y
  {
    cdr << ros_message->kp_eye_right_y;
  }

  // Field name: kp_eye_right_conf
  {
    cdr << ros_message->kp_eye_right_conf;
  }

  // Field name: kp_ear_left_x
  {
    cdr << ros_message->kp_ear_left_x;
  }

  // Field name: kp_ear_left_y
  {
    cdr << ros_message->kp_ear_left_y;
  }

  // Field name: kp_ear_left_conf
  {
    cdr << ros_message->kp_ear_left_conf;
  }

  // Field name: kp_ear_right_x
  {
    cdr << ros_message->kp_ear_right_x;
  }

  // Field name: kp_ear_right_y
  {
    cdr << ros_message->kp_ear_right_y;
  }

  // Field name: kp_ear_right_conf
  {
    cdr << ros_message->kp_ear_right_conf;
  }

  // Field name: kp_shoulder_left_x
  {
    cdr << ros_message->kp_shoulder_left_x;
  }

  // Field name: kp_shoulder_left_y
  {
    cdr << ros_message->kp_shoulder_left_y;
  }

  // Field name: kp_shoulder_left_conf
  {
    cdr << ros_message->kp_shoulder_left_conf;
  }

  // Field name: kp_shoulder_right_x
  {
    cdr << ros_message->kp_shoulder_right_x;
  }

  // Field name: kp_shoulder_right_y
  {
    cdr << ros_message->kp_shoulder_right_y;
  }

  // Field name: kp_shoulder_right_conf
  {
    cdr << ros_message->kp_shoulder_right_conf;
  }

  // Field name: kp_elbow_left_x
  {
    cdr << ros_message->kp_elbow_left_x;
  }

  // Field name: kp_elbow_left_y
  {
    cdr << ros_message->kp_elbow_left_y;
  }

  // Field name: kp_elbow_left_conf
  {
    cdr << ros_message->kp_elbow_left_conf;
  }

  // Field name: kp_elbow_right_x
  {
    cdr << ros_message->kp_elbow_right_x;
  }

  // Field name: kp_elbow_right_y
  {
    cdr << ros_message->kp_elbow_right_y;
  }

  // Field name: kp_elbow_right_conf
  {
    cdr << ros_message->kp_elbow_right_conf;
  }

  // Field name: kp_wrist_left_x
  {
    cdr << ros_message->kp_wrist_left_x;
  }

  // Field name: kp_wrist_left_y
  {
    cdr << ros_message->kp_wrist_left_y;
  }

  // Field name: kp_wrist_left_conf
  {
    cdr << ros_message->kp_wrist_left_conf;
  }

  // Field name: kp_wrist_right_x
  {
    cdr << ros_message->kp_wrist_right_x;
  }

  // Field name: kp_wrist_right_y
  {
    cdr << ros_message->kp_wrist_right_y;
  }

  // Field name: kp_wrist_right_conf
  {
    cdr << ros_message->kp_wrist_right_conf;
  }

  // Field name: kp_hip_left_x
  {
    cdr << ros_message->kp_hip_left_x;
  }

  // Field name: kp_hip_left_y
  {
    cdr << ros_message->kp_hip_left_y;
  }

  // Field name: kp_hip_left_conf
  {
    cdr << ros_message->kp_hip_left_conf;
  }

  // Field name: kp_hip_right_x
  {
    cdr << ros_message->kp_hip_right_x;
  }

  // Field name: kp_hip_right_y
  {
    cdr << ros_message->kp_hip_right_y;
  }

  // Field name: kp_hip_right_conf
  {
    cdr << ros_message->kp_hip_right_conf;
  }

  // Field name: kp_knee_left_x
  {
    cdr << ros_message->kp_knee_left_x;
  }

  // Field name: kp_knee_left_y
  {
    cdr << ros_message->kp_knee_left_y;
  }

  // Field name: kp_knee_left_conf
  {
    cdr << ros_message->kp_knee_left_conf;
  }

  // Field name: kp_knee_right_x
  {
    cdr << ros_message->kp_knee_right_x;
  }

  // Field name: kp_knee_right_y
  {
    cdr << ros_message->kp_knee_right_y;
  }

  // Field name: kp_knee_right_conf
  {
    cdr << ros_message->kp_knee_right_conf;
  }

  // Field name: kp_ankle_left_x
  {
    cdr << ros_message->kp_ankle_left_x;
  }

  // Field name: kp_ankle_left_y
  {
    cdr << ros_message->kp_ankle_left_y;
  }

  // Field name: kp_ankle_left_conf
  {
    cdr << ros_message->kp_ankle_left_conf;
  }

  // Field name: kp_ankle_right_x
  {
    cdr << ros_message->kp_ankle_right_x;
  }

  // Field name: kp_ankle_right_y
  {
    cdr << ros_message->kp_ankle_right_y;
  }

  // Field name: kp_ankle_right_conf
  {
    cdr << ros_message->kp_ankle_right_conf;
  }

  return true;
}

static bool _DetectedPerson__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _DetectedPerson__ros_msg_type * ros_message = static_cast<_DetectedPerson__ros_msg_type *>(untyped_ros_message);
  // Field name: index_person
  {
    cdr >> ros_message->index_person;
  }

  // Field name: conf_person
  {
    cdr >> ros_message->conf_person;
  }

  // Field name: x_rel
  {
    cdr >> ros_message->x_rel;
  }

  // Field name: y_rel
  {
    cdr >> ros_message->y_rel;
  }

  // Field name: box_top_left_x
  {
    cdr >> ros_message->box_top_left_x;
  }

  // Field name: box_top_left_y
  {
    cdr >> ros_message->box_top_left_y;
  }

  // Field name: box_width
  {
    cdr >> ros_message->box_width;
  }

  // Field name: box_height
  {
    cdr >> ros_message->box_height;
  }

  // Field name: kp_nose_x
  {
    cdr >> ros_message->kp_nose_x;
  }

  // Field name: kp_nose_y
  {
    cdr >> ros_message->kp_nose_y;
  }

  // Field name: kp_nose_conf
  {
    cdr >> ros_message->kp_nose_conf;
  }

  // Field name: kp_eye_left_x
  {
    cdr >> ros_message->kp_eye_left_x;
  }

  // Field name: kp_eye_left_y
  {
    cdr >> ros_message->kp_eye_left_y;
  }

  // Field name: kp_eye_left_conf
  {
    cdr >> ros_message->kp_eye_left_conf;
  }

  // Field name: kp_eye_right_x
  {
    cdr >> ros_message->kp_eye_right_x;
  }

  // Field name: kp_eye_right_y
  {
    cdr >> ros_message->kp_eye_right_y;
  }

  // Field name: kp_eye_right_conf
  {
    cdr >> ros_message->kp_eye_right_conf;
  }

  // Field name: kp_ear_left_x
  {
    cdr >> ros_message->kp_ear_left_x;
  }

  // Field name: kp_ear_left_y
  {
    cdr >> ros_message->kp_ear_left_y;
  }

  // Field name: kp_ear_left_conf
  {
    cdr >> ros_message->kp_ear_left_conf;
  }

  // Field name: kp_ear_right_x
  {
    cdr >> ros_message->kp_ear_right_x;
  }

  // Field name: kp_ear_right_y
  {
    cdr >> ros_message->kp_ear_right_y;
  }

  // Field name: kp_ear_right_conf
  {
    cdr >> ros_message->kp_ear_right_conf;
  }

  // Field name: kp_shoulder_left_x
  {
    cdr >> ros_message->kp_shoulder_left_x;
  }

  // Field name: kp_shoulder_left_y
  {
    cdr >> ros_message->kp_shoulder_left_y;
  }

  // Field name: kp_shoulder_left_conf
  {
    cdr >> ros_message->kp_shoulder_left_conf;
  }

  // Field name: kp_shoulder_right_x
  {
    cdr >> ros_message->kp_shoulder_right_x;
  }

  // Field name: kp_shoulder_right_y
  {
    cdr >> ros_message->kp_shoulder_right_y;
  }

  // Field name: kp_shoulder_right_conf
  {
    cdr >> ros_message->kp_shoulder_right_conf;
  }

  // Field name: kp_elbow_left_x
  {
    cdr >> ros_message->kp_elbow_left_x;
  }

  // Field name: kp_elbow_left_y
  {
    cdr >> ros_message->kp_elbow_left_y;
  }

  // Field name: kp_elbow_left_conf
  {
    cdr >> ros_message->kp_elbow_left_conf;
  }

  // Field name: kp_elbow_right_x
  {
    cdr >> ros_message->kp_elbow_right_x;
  }

  // Field name: kp_elbow_right_y
  {
    cdr >> ros_message->kp_elbow_right_y;
  }

  // Field name: kp_elbow_right_conf
  {
    cdr >> ros_message->kp_elbow_right_conf;
  }

  // Field name: kp_wrist_left_x
  {
    cdr >> ros_message->kp_wrist_left_x;
  }

  // Field name: kp_wrist_left_y
  {
    cdr >> ros_message->kp_wrist_left_y;
  }

  // Field name: kp_wrist_left_conf
  {
    cdr >> ros_message->kp_wrist_left_conf;
  }

  // Field name: kp_wrist_right_x
  {
    cdr >> ros_message->kp_wrist_right_x;
  }

  // Field name: kp_wrist_right_y
  {
    cdr >> ros_message->kp_wrist_right_y;
  }

  // Field name: kp_wrist_right_conf
  {
    cdr >> ros_message->kp_wrist_right_conf;
  }

  // Field name: kp_hip_left_x
  {
    cdr >> ros_message->kp_hip_left_x;
  }

  // Field name: kp_hip_left_y
  {
    cdr >> ros_message->kp_hip_left_y;
  }

  // Field name: kp_hip_left_conf
  {
    cdr >> ros_message->kp_hip_left_conf;
  }

  // Field name: kp_hip_right_x
  {
    cdr >> ros_message->kp_hip_right_x;
  }

  // Field name: kp_hip_right_y
  {
    cdr >> ros_message->kp_hip_right_y;
  }

  // Field name: kp_hip_right_conf
  {
    cdr >> ros_message->kp_hip_right_conf;
  }

  // Field name: kp_knee_left_x
  {
    cdr >> ros_message->kp_knee_left_x;
  }

  // Field name: kp_knee_left_y
  {
    cdr >> ros_message->kp_knee_left_y;
  }

  // Field name: kp_knee_left_conf
  {
    cdr >> ros_message->kp_knee_left_conf;
  }

  // Field name: kp_knee_right_x
  {
    cdr >> ros_message->kp_knee_right_x;
  }

  // Field name: kp_knee_right_y
  {
    cdr >> ros_message->kp_knee_right_y;
  }

  // Field name: kp_knee_right_conf
  {
    cdr >> ros_message->kp_knee_right_conf;
  }

  // Field name: kp_ankle_left_x
  {
    cdr >> ros_message->kp_ankle_left_x;
  }

  // Field name: kp_ankle_left_y
  {
    cdr >> ros_message->kp_ankle_left_y;
  }

  // Field name: kp_ankle_left_conf
  {
    cdr >> ros_message->kp_ankle_left_conf;
  }

  // Field name: kp_ankle_right_x
  {
    cdr >> ros_message->kp_ankle_right_x;
  }

  // Field name: kp_ankle_right_y
  {
    cdr >> ros_message->kp_ankle_right_y;
  }

  // Field name: kp_ankle_right_conf
  {
    cdr >> ros_message->kp_ankle_right_conf;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t get_serialized_size_charmie_interfaces__msg__DetectedPerson(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _DetectedPerson__ros_msg_type * ros_message = static_cast<const _DetectedPerson__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name index_person
  {
    size_t item_size = sizeof(ros_message->index_person);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name conf_person
  {
    size_t item_size = sizeof(ros_message->conf_person);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name x_rel
  {
    size_t item_size = sizeof(ros_message->x_rel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name y_rel
  {
    size_t item_size = sizeof(ros_message->y_rel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name box_top_left_x
  {
    size_t item_size = sizeof(ros_message->box_top_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name box_top_left_y
  {
    size_t item_size = sizeof(ros_message->box_top_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name box_width
  {
    size_t item_size = sizeof(ros_message->box_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name box_height
  {
    size_t item_size = sizeof(ros_message->box_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_nose_x
  {
    size_t item_size = sizeof(ros_message->kp_nose_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_nose_y
  {
    size_t item_size = sizeof(ros_message->kp_nose_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_nose_conf
  {
    size_t item_size = sizeof(ros_message->kp_nose_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_eye_left_x
  {
    size_t item_size = sizeof(ros_message->kp_eye_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_eye_left_y
  {
    size_t item_size = sizeof(ros_message->kp_eye_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_eye_left_conf
  {
    size_t item_size = sizeof(ros_message->kp_eye_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_eye_right_x
  {
    size_t item_size = sizeof(ros_message->kp_eye_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_eye_right_y
  {
    size_t item_size = sizeof(ros_message->kp_eye_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_eye_right_conf
  {
    size_t item_size = sizeof(ros_message->kp_eye_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ear_left_x
  {
    size_t item_size = sizeof(ros_message->kp_ear_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ear_left_y
  {
    size_t item_size = sizeof(ros_message->kp_ear_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ear_left_conf
  {
    size_t item_size = sizeof(ros_message->kp_ear_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ear_right_x
  {
    size_t item_size = sizeof(ros_message->kp_ear_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ear_right_y
  {
    size_t item_size = sizeof(ros_message->kp_ear_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ear_right_conf
  {
    size_t item_size = sizeof(ros_message->kp_ear_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_shoulder_left_x
  {
    size_t item_size = sizeof(ros_message->kp_shoulder_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_shoulder_left_y
  {
    size_t item_size = sizeof(ros_message->kp_shoulder_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_shoulder_left_conf
  {
    size_t item_size = sizeof(ros_message->kp_shoulder_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_shoulder_right_x
  {
    size_t item_size = sizeof(ros_message->kp_shoulder_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_shoulder_right_y
  {
    size_t item_size = sizeof(ros_message->kp_shoulder_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_shoulder_right_conf
  {
    size_t item_size = sizeof(ros_message->kp_shoulder_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_elbow_left_x
  {
    size_t item_size = sizeof(ros_message->kp_elbow_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_elbow_left_y
  {
    size_t item_size = sizeof(ros_message->kp_elbow_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_elbow_left_conf
  {
    size_t item_size = sizeof(ros_message->kp_elbow_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_elbow_right_x
  {
    size_t item_size = sizeof(ros_message->kp_elbow_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_elbow_right_y
  {
    size_t item_size = sizeof(ros_message->kp_elbow_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_elbow_right_conf
  {
    size_t item_size = sizeof(ros_message->kp_elbow_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_wrist_left_x
  {
    size_t item_size = sizeof(ros_message->kp_wrist_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_wrist_left_y
  {
    size_t item_size = sizeof(ros_message->kp_wrist_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_wrist_left_conf
  {
    size_t item_size = sizeof(ros_message->kp_wrist_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_wrist_right_x
  {
    size_t item_size = sizeof(ros_message->kp_wrist_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_wrist_right_y
  {
    size_t item_size = sizeof(ros_message->kp_wrist_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_wrist_right_conf
  {
    size_t item_size = sizeof(ros_message->kp_wrist_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_hip_left_x
  {
    size_t item_size = sizeof(ros_message->kp_hip_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_hip_left_y
  {
    size_t item_size = sizeof(ros_message->kp_hip_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_hip_left_conf
  {
    size_t item_size = sizeof(ros_message->kp_hip_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_hip_right_x
  {
    size_t item_size = sizeof(ros_message->kp_hip_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_hip_right_y
  {
    size_t item_size = sizeof(ros_message->kp_hip_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_hip_right_conf
  {
    size_t item_size = sizeof(ros_message->kp_hip_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_knee_left_x
  {
    size_t item_size = sizeof(ros_message->kp_knee_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_knee_left_y
  {
    size_t item_size = sizeof(ros_message->kp_knee_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_knee_left_conf
  {
    size_t item_size = sizeof(ros_message->kp_knee_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_knee_right_x
  {
    size_t item_size = sizeof(ros_message->kp_knee_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_knee_right_y
  {
    size_t item_size = sizeof(ros_message->kp_knee_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_knee_right_conf
  {
    size_t item_size = sizeof(ros_message->kp_knee_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ankle_left_x
  {
    size_t item_size = sizeof(ros_message->kp_ankle_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ankle_left_y
  {
    size_t item_size = sizeof(ros_message->kp_ankle_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ankle_left_conf
  {
    size_t item_size = sizeof(ros_message->kp_ankle_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ankle_right_x
  {
    size_t item_size = sizeof(ros_message->kp_ankle_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ankle_right_y
  {
    size_t item_size = sizeof(ros_message->kp_ankle_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kp_ankle_right_conf
  {
    size_t item_size = sizeof(ros_message->kp_ankle_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _DetectedPerson__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_charmie_interfaces__msg__DetectedPerson(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t max_serialized_size_charmie_interfaces__msg__DetectedPerson(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: index_person
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: conf_person
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: x_rel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: y_rel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: box_top_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: box_top_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: box_width
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: box_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_nose_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_nose_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_nose_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_eye_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_eye_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_eye_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_eye_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_eye_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_eye_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ear_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ear_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ear_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ear_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ear_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ear_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_shoulder_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_shoulder_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_shoulder_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_shoulder_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_shoulder_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_shoulder_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_elbow_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_elbow_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_elbow_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_elbow_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_elbow_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_elbow_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_wrist_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_wrist_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_wrist_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_wrist_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_wrist_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_wrist_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_hip_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_hip_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_hip_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_hip_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_hip_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_hip_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_knee_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_knee_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_knee_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_knee_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_knee_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_knee_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ankle_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ankle_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ankle_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ankle_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ankle_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: kp_ankle_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _DetectedPerson__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_charmie_interfaces__msg__DetectedPerson(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_DetectedPerson = {
  "charmie_interfaces::msg",
  "DetectedPerson",
  _DetectedPerson__cdr_serialize,
  _DetectedPerson__cdr_deserialize,
  _DetectedPerson__get_serialized_size,
  _DetectedPerson__max_serialized_size
};

static rosidl_message_type_support_t _DetectedPerson__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_DetectedPerson,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, DetectedPerson)() {
  return &_DetectedPerson__type_support;
}

#if defined(__cplusplus)
}
#endif
