// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/DetectedPerson.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/detected_person__rosidl_typesupport_fastrtps_cpp.hpp"
#include "charmie_interfaces/msg/detail/detected_person__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace charmie_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_serialize(
  const charmie_interfaces::msg::DetectedPerson & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: index_person
  cdr << ros_message.index_person;
  // Member: conf_person
  cdr << ros_message.conf_person;
  // Member: x_rel
  cdr << ros_message.x_rel;
  // Member: y_rel
  cdr << ros_message.y_rel;
  // Member: box_top_left_x
  cdr << ros_message.box_top_left_x;
  // Member: box_top_left_y
  cdr << ros_message.box_top_left_y;
  // Member: box_width
  cdr << ros_message.box_width;
  // Member: box_height
  cdr << ros_message.box_height;
  // Member: kp_nose_x
  cdr << ros_message.kp_nose_x;
  // Member: kp_nose_y
  cdr << ros_message.kp_nose_y;
  // Member: kp_nose_conf
  cdr << ros_message.kp_nose_conf;
  // Member: kp_eye_left_x
  cdr << ros_message.kp_eye_left_x;
  // Member: kp_eye_left_y
  cdr << ros_message.kp_eye_left_y;
  // Member: kp_eye_left_conf
  cdr << ros_message.kp_eye_left_conf;
  // Member: kp_eye_right_x
  cdr << ros_message.kp_eye_right_x;
  // Member: kp_eye_right_y
  cdr << ros_message.kp_eye_right_y;
  // Member: kp_eye_right_conf
  cdr << ros_message.kp_eye_right_conf;
  // Member: kp_ear_left_x
  cdr << ros_message.kp_ear_left_x;
  // Member: kp_ear_left_y
  cdr << ros_message.kp_ear_left_y;
  // Member: kp_ear_left_conf
  cdr << ros_message.kp_ear_left_conf;
  // Member: kp_ear_right_x
  cdr << ros_message.kp_ear_right_x;
  // Member: kp_ear_right_y
  cdr << ros_message.kp_ear_right_y;
  // Member: kp_ear_right_conf
  cdr << ros_message.kp_ear_right_conf;
  // Member: kp_shoulder_left_x
  cdr << ros_message.kp_shoulder_left_x;
  // Member: kp_shoulder_left_y
  cdr << ros_message.kp_shoulder_left_y;
  // Member: kp_shoulder_left_conf
  cdr << ros_message.kp_shoulder_left_conf;
  // Member: kp_shoulder_right_x
  cdr << ros_message.kp_shoulder_right_x;
  // Member: kp_shoulder_right_y
  cdr << ros_message.kp_shoulder_right_y;
  // Member: kp_shoulder_right_conf
  cdr << ros_message.kp_shoulder_right_conf;
  // Member: kp_elbow_left_x
  cdr << ros_message.kp_elbow_left_x;
  // Member: kp_elbow_left_y
  cdr << ros_message.kp_elbow_left_y;
  // Member: kp_elbow_left_conf
  cdr << ros_message.kp_elbow_left_conf;
  // Member: kp_elbow_right_x
  cdr << ros_message.kp_elbow_right_x;
  // Member: kp_elbow_right_y
  cdr << ros_message.kp_elbow_right_y;
  // Member: kp_elbow_right_conf
  cdr << ros_message.kp_elbow_right_conf;
  // Member: kp_wrist_left_x
  cdr << ros_message.kp_wrist_left_x;
  // Member: kp_wrist_left_y
  cdr << ros_message.kp_wrist_left_y;
  // Member: kp_wrist_left_conf
  cdr << ros_message.kp_wrist_left_conf;
  // Member: kp_wrist_right_x
  cdr << ros_message.kp_wrist_right_x;
  // Member: kp_wrist_right_y
  cdr << ros_message.kp_wrist_right_y;
  // Member: kp_wrist_right_conf
  cdr << ros_message.kp_wrist_right_conf;
  // Member: kp_hip_left_x
  cdr << ros_message.kp_hip_left_x;
  // Member: kp_hip_left_y
  cdr << ros_message.kp_hip_left_y;
  // Member: kp_hip_left_conf
  cdr << ros_message.kp_hip_left_conf;
  // Member: kp_hip_right_x
  cdr << ros_message.kp_hip_right_x;
  // Member: kp_hip_right_y
  cdr << ros_message.kp_hip_right_y;
  // Member: kp_hip_right_conf
  cdr << ros_message.kp_hip_right_conf;
  // Member: kp_knee_left_x
  cdr << ros_message.kp_knee_left_x;
  // Member: kp_knee_left_y
  cdr << ros_message.kp_knee_left_y;
  // Member: kp_knee_left_conf
  cdr << ros_message.kp_knee_left_conf;
  // Member: kp_knee_right_x
  cdr << ros_message.kp_knee_right_x;
  // Member: kp_knee_right_y
  cdr << ros_message.kp_knee_right_y;
  // Member: kp_knee_right_conf
  cdr << ros_message.kp_knee_right_conf;
  // Member: kp_ankle_left_x
  cdr << ros_message.kp_ankle_left_x;
  // Member: kp_ankle_left_y
  cdr << ros_message.kp_ankle_left_y;
  // Member: kp_ankle_left_conf
  cdr << ros_message.kp_ankle_left_conf;
  // Member: kp_ankle_right_x
  cdr << ros_message.kp_ankle_right_x;
  // Member: kp_ankle_right_y
  cdr << ros_message.kp_ankle_right_y;
  // Member: kp_ankle_right_conf
  cdr << ros_message.kp_ankle_right_conf;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  charmie_interfaces::msg::DetectedPerson & ros_message)
{
  // Member: index_person
  cdr >> ros_message.index_person;

  // Member: conf_person
  cdr >> ros_message.conf_person;

  // Member: x_rel
  cdr >> ros_message.x_rel;

  // Member: y_rel
  cdr >> ros_message.y_rel;

  // Member: box_top_left_x
  cdr >> ros_message.box_top_left_x;

  // Member: box_top_left_y
  cdr >> ros_message.box_top_left_y;

  // Member: box_width
  cdr >> ros_message.box_width;

  // Member: box_height
  cdr >> ros_message.box_height;

  // Member: kp_nose_x
  cdr >> ros_message.kp_nose_x;

  // Member: kp_nose_y
  cdr >> ros_message.kp_nose_y;

  // Member: kp_nose_conf
  cdr >> ros_message.kp_nose_conf;

  // Member: kp_eye_left_x
  cdr >> ros_message.kp_eye_left_x;

  // Member: kp_eye_left_y
  cdr >> ros_message.kp_eye_left_y;

  // Member: kp_eye_left_conf
  cdr >> ros_message.kp_eye_left_conf;

  // Member: kp_eye_right_x
  cdr >> ros_message.kp_eye_right_x;

  // Member: kp_eye_right_y
  cdr >> ros_message.kp_eye_right_y;

  // Member: kp_eye_right_conf
  cdr >> ros_message.kp_eye_right_conf;

  // Member: kp_ear_left_x
  cdr >> ros_message.kp_ear_left_x;

  // Member: kp_ear_left_y
  cdr >> ros_message.kp_ear_left_y;

  // Member: kp_ear_left_conf
  cdr >> ros_message.kp_ear_left_conf;

  // Member: kp_ear_right_x
  cdr >> ros_message.kp_ear_right_x;

  // Member: kp_ear_right_y
  cdr >> ros_message.kp_ear_right_y;

  // Member: kp_ear_right_conf
  cdr >> ros_message.kp_ear_right_conf;

  // Member: kp_shoulder_left_x
  cdr >> ros_message.kp_shoulder_left_x;

  // Member: kp_shoulder_left_y
  cdr >> ros_message.kp_shoulder_left_y;

  // Member: kp_shoulder_left_conf
  cdr >> ros_message.kp_shoulder_left_conf;

  // Member: kp_shoulder_right_x
  cdr >> ros_message.kp_shoulder_right_x;

  // Member: kp_shoulder_right_y
  cdr >> ros_message.kp_shoulder_right_y;

  // Member: kp_shoulder_right_conf
  cdr >> ros_message.kp_shoulder_right_conf;

  // Member: kp_elbow_left_x
  cdr >> ros_message.kp_elbow_left_x;

  // Member: kp_elbow_left_y
  cdr >> ros_message.kp_elbow_left_y;

  // Member: kp_elbow_left_conf
  cdr >> ros_message.kp_elbow_left_conf;

  // Member: kp_elbow_right_x
  cdr >> ros_message.kp_elbow_right_x;

  // Member: kp_elbow_right_y
  cdr >> ros_message.kp_elbow_right_y;

  // Member: kp_elbow_right_conf
  cdr >> ros_message.kp_elbow_right_conf;

  // Member: kp_wrist_left_x
  cdr >> ros_message.kp_wrist_left_x;

  // Member: kp_wrist_left_y
  cdr >> ros_message.kp_wrist_left_y;

  // Member: kp_wrist_left_conf
  cdr >> ros_message.kp_wrist_left_conf;

  // Member: kp_wrist_right_x
  cdr >> ros_message.kp_wrist_right_x;

  // Member: kp_wrist_right_y
  cdr >> ros_message.kp_wrist_right_y;

  // Member: kp_wrist_right_conf
  cdr >> ros_message.kp_wrist_right_conf;

  // Member: kp_hip_left_x
  cdr >> ros_message.kp_hip_left_x;

  // Member: kp_hip_left_y
  cdr >> ros_message.kp_hip_left_y;

  // Member: kp_hip_left_conf
  cdr >> ros_message.kp_hip_left_conf;

  // Member: kp_hip_right_x
  cdr >> ros_message.kp_hip_right_x;

  // Member: kp_hip_right_y
  cdr >> ros_message.kp_hip_right_y;

  // Member: kp_hip_right_conf
  cdr >> ros_message.kp_hip_right_conf;

  // Member: kp_knee_left_x
  cdr >> ros_message.kp_knee_left_x;

  // Member: kp_knee_left_y
  cdr >> ros_message.kp_knee_left_y;

  // Member: kp_knee_left_conf
  cdr >> ros_message.kp_knee_left_conf;

  // Member: kp_knee_right_x
  cdr >> ros_message.kp_knee_right_x;

  // Member: kp_knee_right_y
  cdr >> ros_message.kp_knee_right_y;

  // Member: kp_knee_right_conf
  cdr >> ros_message.kp_knee_right_conf;

  // Member: kp_ankle_left_x
  cdr >> ros_message.kp_ankle_left_x;

  // Member: kp_ankle_left_y
  cdr >> ros_message.kp_ankle_left_y;

  // Member: kp_ankle_left_conf
  cdr >> ros_message.kp_ankle_left_conf;

  // Member: kp_ankle_right_x
  cdr >> ros_message.kp_ankle_right_x;

  // Member: kp_ankle_right_y
  cdr >> ros_message.kp_ankle_right_y;

  // Member: kp_ankle_right_conf
  cdr >> ros_message.kp_ankle_right_conf;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
get_serialized_size(
  const charmie_interfaces::msg::DetectedPerson & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: index_person
  {
    size_t item_size = sizeof(ros_message.index_person);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: conf_person
  {
    size_t item_size = sizeof(ros_message.conf_person);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: x_rel
  {
    size_t item_size = sizeof(ros_message.x_rel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: y_rel
  {
    size_t item_size = sizeof(ros_message.y_rel);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: box_top_left_x
  {
    size_t item_size = sizeof(ros_message.box_top_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: box_top_left_y
  {
    size_t item_size = sizeof(ros_message.box_top_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: box_width
  {
    size_t item_size = sizeof(ros_message.box_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: box_height
  {
    size_t item_size = sizeof(ros_message.box_height);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_nose_x
  {
    size_t item_size = sizeof(ros_message.kp_nose_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_nose_y
  {
    size_t item_size = sizeof(ros_message.kp_nose_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_nose_conf
  {
    size_t item_size = sizeof(ros_message.kp_nose_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_eye_left_x
  {
    size_t item_size = sizeof(ros_message.kp_eye_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_eye_left_y
  {
    size_t item_size = sizeof(ros_message.kp_eye_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_eye_left_conf
  {
    size_t item_size = sizeof(ros_message.kp_eye_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_eye_right_x
  {
    size_t item_size = sizeof(ros_message.kp_eye_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_eye_right_y
  {
    size_t item_size = sizeof(ros_message.kp_eye_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_eye_right_conf
  {
    size_t item_size = sizeof(ros_message.kp_eye_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ear_left_x
  {
    size_t item_size = sizeof(ros_message.kp_ear_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ear_left_y
  {
    size_t item_size = sizeof(ros_message.kp_ear_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ear_left_conf
  {
    size_t item_size = sizeof(ros_message.kp_ear_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ear_right_x
  {
    size_t item_size = sizeof(ros_message.kp_ear_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ear_right_y
  {
    size_t item_size = sizeof(ros_message.kp_ear_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ear_right_conf
  {
    size_t item_size = sizeof(ros_message.kp_ear_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_shoulder_left_x
  {
    size_t item_size = sizeof(ros_message.kp_shoulder_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_shoulder_left_y
  {
    size_t item_size = sizeof(ros_message.kp_shoulder_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_shoulder_left_conf
  {
    size_t item_size = sizeof(ros_message.kp_shoulder_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_shoulder_right_x
  {
    size_t item_size = sizeof(ros_message.kp_shoulder_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_shoulder_right_y
  {
    size_t item_size = sizeof(ros_message.kp_shoulder_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_shoulder_right_conf
  {
    size_t item_size = sizeof(ros_message.kp_shoulder_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_elbow_left_x
  {
    size_t item_size = sizeof(ros_message.kp_elbow_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_elbow_left_y
  {
    size_t item_size = sizeof(ros_message.kp_elbow_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_elbow_left_conf
  {
    size_t item_size = sizeof(ros_message.kp_elbow_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_elbow_right_x
  {
    size_t item_size = sizeof(ros_message.kp_elbow_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_elbow_right_y
  {
    size_t item_size = sizeof(ros_message.kp_elbow_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_elbow_right_conf
  {
    size_t item_size = sizeof(ros_message.kp_elbow_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_wrist_left_x
  {
    size_t item_size = sizeof(ros_message.kp_wrist_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_wrist_left_y
  {
    size_t item_size = sizeof(ros_message.kp_wrist_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_wrist_left_conf
  {
    size_t item_size = sizeof(ros_message.kp_wrist_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_wrist_right_x
  {
    size_t item_size = sizeof(ros_message.kp_wrist_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_wrist_right_y
  {
    size_t item_size = sizeof(ros_message.kp_wrist_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_wrist_right_conf
  {
    size_t item_size = sizeof(ros_message.kp_wrist_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_hip_left_x
  {
    size_t item_size = sizeof(ros_message.kp_hip_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_hip_left_y
  {
    size_t item_size = sizeof(ros_message.kp_hip_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_hip_left_conf
  {
    size_t item_size = sizeof(ros_message.kp_hip_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_hip_right_x
  {
    size_t item_size = sizeof(ros_message.kp_hip_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_hip_right_y
  {
    size_t item_size = sizeof(ros_message.kp_hip_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_hip_right_conf
  {
    size_t item_size = sizeof(ros_message.kp_hip_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_knee_left_x
  {
    size_t item_size = sizeof(ros_message.kp_knee_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_knee_left_y
  {
    size_t item_size = sizeof(ros_message.kp_knee_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_knee_left_conf
  {
    size_t item_size = sizeof(ros_message.kp_knee_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_knee_right_x
  {
    size_t item_size = sizeof(ros_message.kp_knee_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_knee_right_y
  {
    size_t item_size = sizeof(ros_message.kp_knee_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_knee_right_conf
  {
    size_t item_size = sizeof(ros_message.kp_knee_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ankle_left_x
  {
    size_t item_size = sizeof(ros_message.kp_ankle_left_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ankle_left_y
  {
    size_t item_size = sizeof(ros_message.kp_ankle_left_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ankle_left_conf
  {
    size_t item_size = sizeof(ros_message.kp_ankle_left_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ankle_right_x
  {
    size_t item_size = sizeof(ros_message.kp_ankle_right_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ankle_right_y
  {
    size_t item_size = sizeof(ros_message.kp_ankle_right_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: kp_ankle_right_conf
  {
    size_t item_size = sizeof(ros_message.kp_ankle_right_conf);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
max_serialized_size_DetectedPerson(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: index_person
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: conf_person
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: x_rel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: y_rel
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: box_top_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: box_top_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: box_width
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: box_height
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_nose_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_nose_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_nose_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_eye_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_eye_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_eye_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_eye_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_eye_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_eye_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ear_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ear_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ear_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ear_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ear_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ear_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_shoulder_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_shoulder_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_shoulder_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_shoulder_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_shoulder_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_shoulder_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_elbow_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_elbow_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_elbow_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_elbow_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_elbow_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_elbow_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_wrist_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_wrist_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_wrist_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_wrist_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_wrist_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_wrist_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_hip_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_hip_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_hip_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_hip_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_hip_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_hip_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_knee_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_knee_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_knee_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_knee_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_knee_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_knee_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ankle_left_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ankle_left_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ankle_left_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ankle_right_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ankle_right_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: kp_ankle_right_conf
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _DetectedPerson__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::DetectedPerson *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _DetectedPerson__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<charmie_interfaces::msg::DetectedPerson *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _DetectedPerson__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::DetectedPerson *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _DetectedPerson__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_DetectedPerson(full_bounded, 0);
}

static message_type_support_callbacks_t _DetectedPerson__callbacks = {
  "charmie_interfaces::msg",
  "DetectedPerson",
  _DetectedPerson__cdr_serialize,
  _DetectedPerson__cdr_deserialize,
  _DetectedPerson__get_serialized_size,
  _DetectedPerson__max_serialized_size
};

static rosidl_message_type_support_t _DetectedPerson__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_DetectedPerson__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace charmie_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<charmie_interfaces::msg::DetectedPerson>()
{
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_DetectedPerson__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, charmie_interfaces, msg, DetectedPerson)() {
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_DetectedPerson__handle;
}

#ifdef __cplusplus
}
#endif
