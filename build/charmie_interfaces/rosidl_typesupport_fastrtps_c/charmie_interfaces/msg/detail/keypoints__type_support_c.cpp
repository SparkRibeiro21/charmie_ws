// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from charmie_interfaces:msg/Keypoints.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/keypoints__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "charmie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "charmie_interfaces/msg/detail/keypoints__struct.h"
#include "charmie_interfaces/msg/detail/keypoints__functions.h"
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


using _Keypoints__ros_msg_type = charmie_interfaces__msg__Keypoints;

static bool _Keypoints__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Keypoints__ros_msg_type * ros_message = static_cast<const _Keypoints__ros_msg_type *>(untyped_ros_message);
  // Field name: index_person
  {
    cdr << ros_message->index_person;
  }

  // Field name: average_distance
  {
    cdr << ros_message->average_distance;
  }

  // Field name: standard_deviation
  {
    cdr << ros_message->standard_deviation;
  }

  // Field name: key_p0_x
  {
    cdr << ros_message->key_p0_x;
  }

  // Field name: key_p0_y
  {
    cdr << ros_message->key_p0_y;
  }

  // Field name: key_p1_x
  {
    cdr << ros_message->key_p1_x;
  }

  // Field name: key_p1_y
  {
    cdr << ros_message->key_p1_y;
  }

  // Field name: key_p2_x
  {
    cdr << ros_message->key_p2_x;
  }

  // Field name: key_p2_y
  {
    cdr << ros_message->key_p2_y;
  }

  // Field name: key_p3_x
  {
    cdr << ros_message->key_p3_x;
  }

  // Field name: key_p3_y
  {
    cdr << ros_message->key_p3_y;
  }

  // Field name: key_p4_x
  {
    cdr << ros_message->key_p4_x;
  }

  // Field name: key_p4_y
  {
    cdr << ros_message->key_p4_y;
  }

  // Field name: key_p5_x
  {
    cdr << ros_message->key_p5_x;
  }

  // Field name: key_p5_y
  {
    cdr << ros_message->key_p5_y;
  }

  // Field name: key_p6_x
  {
    cdr << ros_message->key_p6_x;
  }

  // Field name: key_p6_y
  {
    cdr << ros_message->key_p6_y;
  }

  // Field name: key_p7_x
  {
    cdr << ros_message->key_p7_x;
  }

  // Field name: key_p7_y
  {
    cdr << ros_message->key_p7_y;
  }

  // Field name: key_p8_x
  {
    cdr << ros_message->key_p8_x;
  }

  // Field name: key_p8_y
  {
    cdr << ros_message->key_p8_y;
  }

  // Field name: key_p9_x
  {
    cdr << ros_message->key_p9_x;
  }

  // Field name: key_p9_y
  {
    cdr << ros_message->key_p9_y;
  }

  // Field name: key_p10_x
  {
    cdr << ros_message->key_p10_x;
  }

  // Field name: key_p10_y
  {
    cdr << ros_message->key_p10_y;
  }

  // Field name: key_p11_x
  {
    cdr << ros_message->key_p11_x;
  }

  // Field name: key_p11_y
  {
    cdr << ros_message->key_p11_y;
  }

  // Field name: key_p12_x
  {
    cdr << ros_message->key_p12_x;
  }

  // Field name: key_p12_y
  {
    cdr << ros_message->key_p12_y;
  }

  // Field name: key_p13_x
  {
    cdr << ros_message->key_p13_x;
  }

  // Field name: key_p13_y
  {
    cdr << ros_message->key_p13_y;
  }

  // Field name: key_p14_x
  {
    cdr << ros_message->key_p14_x;
  }

  // Field name: key_p14_y
  {
    cdr << ros_message->key_p14_y;
  }

  // Field name: key_p15_x
  {
    cdr << ros_message->key_p15_x;
  }

  // Field name: key_p15_y
  {
    cdr << ros_message->key_p15_y;
  }

  // Field name: key_p16_x
  {
    cdr << ros_message->key_p16_x;
  }

  // Field name: key_p16_y
  {
    cdr << ros_message->key_p16_y;
  }

  return true;
}

static bool _Keypoints__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Keypoints__ros_msg_type * ros_message = static_cast<_Keypoints__ros_msg_type *>(untyped_ros_message);
  // Field name: index_person
  {
    cdr >> ros_message->index_person;
  }

  // Field name: average_distance
  {
    cdr >> ros_message->average_distance;
  }

  // Field name: standard_deviation
  {
    cdr >> ros_message->standard_deviation;
  }

  // Field name: key_p0_x
  {
    cdr >> ros_message->key_p0_x;
  }

  // Field name: key_p0_y
  {
    cdr >> ros_message->key_p0_y;
  }

  // Field name: key_p1_x
  {
    cdr >> ros_message->key_p1_x;
  }

  // Field name: key_p1_y
  {
    cdr >> ros_message->key_p1_y;
  }

  // Field name: key_p2_x
  {
    cdr >> ros_message->key_p2_x;
  }

  // Field name: key_p2_y
  {
    cdr >> ros_message->key_p2_y;
  }

  // Field name: key_p3_x
  {
    cdr >> ros_message->key_p3_x;
  }

  // Field name: key_p3_y
  {
    cdr >> ros_message->key_p3_y;
  }

  // Field name: key_p4_x
  {
    cdr >> ros_message->key_p4_x;
  }

  // Field name: key_p4_y
  {
    cdr >> ros_message->key_p4_y;
  }

  // Field name: key_p5_x
  {
    cdr >> ros_message->key_p5_x;
  }

  // Field name: key_p5_y
  {
    cdr >> ros_message->key_p5_y;
  }

  // Field name: key_p6_x
  {
    cdr >> ros_message->key_p6_x;
  }

  // Field name: key_p6_y
  {
    cdr >> ros_message->key_p6_y;
  }

  // Field name: key_p7_x
  {
    cdr >> ros_message->key_p7_x;
  }

  // Field name: key_p7_y
  {
    cdr >> ros_message->key_p7_y;
  }

  // Field name: key_p8_x
  {
    cdr >> ros_message->key_p8_x;
  }

  // Field name: key_p8_y
  {
    cdr >> ros_message->key_p8_y;
  }

  // Field name: key_p9_x
  {
    cdr >> ros_message->key_p9_x;
  }

  // Field name: key_p9_y
  {
    cdr >> ros_message->key_p9_y;
  }

  // Field name: key_p10_x
  {
    cdr >> ros_message->key_p10_x;
  }

  // Field name: key_p10_y
  {
    cdr >> ros_message->key_p10_y;
  }

  // Field name: key_p11_x
  {
    cdr >> ros_message->key_p11_x;
  }

  // Field name: key_p11_y
  {
    cdr >> ros_message->key_p11_y;
  }

  // Field name: key_p12_x
  {
    cdr >> ros_message->key_p12_x;
  }

  // Field name: key_p12_y
  {
    cdr >> ros_message->key_p12_y;
  }

  // Field name: key_p13_x
  {
    cdr >> ros_message->key_p13_x;
  }

  // Field name: key_p13_y
  {
    cdr >> ros_message->key_p13_y;
  }

  // Field name: key_p14_x
  {
    cdr >> ros_message->key_p14_x;
  }

  // Field name: key_p14_y
  {
    cdr >> ros_message->key_p14_y;
  }

  // Field name: key_p15_x
  {
    cdr >> ros_message->key_p15_x;
  }

  // Field name: key_p15_y
  {
    cdr >> ros_message->key_p15_y;
  }

  // Field name: key_p16_x
  {
    cdr >> ros_message->key_p16_x;
  }

  // Field name: key_p16_y
  {
    cdr >> ros_message->key_p16_y;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t get_serialized_size_charmie_interfaces__msg__Keypoints(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Keypoints__ros_msg_type * ros_message = static_cast<const _Keypoints__ros_msg_type *>(untyped_ros_message);
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
  // field.name average_distance
  {
    size_t item_size = sizeof(ros_message->average_distance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name standard_deviation
  {
    size_t item_size = sizeof(ros_message->standard_deviation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p0_x
  {
    size_t item_size = sizeof(ros_message->key_p0_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p0_y
  {
    size_t item_size = sizeof(ros_message->key_p0_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p1_x
  {
    size_t item_size = sizeof(ros_message->key_p1_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p1_y
  {
    size_t item_size = sizeof(ros_message->key_p1_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p2_x
  {
    size_t item_size = sizeof(ros_message->key_p2_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p2_y
  {
    size_t item_size = sizeof(ros_message->key_p2_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p3_x
  {
    size_t item_size = sizeof(ros_message->key_p3_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p3_y
  {
    size_t item_size = sizeof(ros_message->key_p3_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p4_x
  {
    size_t item_size = sizeof(ros_message->key_p4_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p4_y
  {
    size_t item_size = sizeof(ros_message->key_p4_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p5_x
  {
    size_t item_size = sizeof(ros_message->key_p5_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p5_y
  {
    size_t item_size = sizeof(ros_message->key_p5_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p6_x
  {
    size_t item_size = sizeof(ros_message->key_p6_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p6_y
  {
    size_t item_size = sizeof(ros_message->key_p6_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p7_x
  {
    size_t item_size = sizeof(ros_message->key_p7_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p7_y
  {
    size_t item_size = sizeof(ros_message->key_p7_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p8_x
  {
    size_t item_size = sizeof(ros_message->key_p8_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p8_y
  {
    size_t item_size = sizeof(ros_message->key_p8_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p9_x
  {
    size_t item_size = sizeof(ros_message->key_p9_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p9_y
  {
    size_t item_size = sizeof(ros_message->key_p9_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p10_x
  {
    size_t item_size = sizeof(ros_message->key_p10_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p10_y
  {
    size_t item_size = sizeof(ros_message->key_p10_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p11_x
  {
    size_t item_size = sizeof(ros_message->key_p11_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p11_y
  {
    size_t item_size = sizeof(ros_message->key_p11_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p12_x
  {
    size_t item_size = sizeof(ros_message->key_p12_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p12_y
  {
    size_t item_size = sizeof(ros_message->key_p12_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p13_x
  {
    size_t item_size = sizeof(ros_message->key_p13_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p13_y
  {
    size_t item_size = sizeof(ros_message->key_p13_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p14_x
  {
    size_t item_size = sizeof(ros_message->key_p14_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p14_y
  {
    size_t item_size = sizeof(ros_message->key_p14_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p15_x
  {
    size_t item_size = sizeof(ros_message->key_p15_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p15_y
  {
    size_t item_size = sizeof(ros_message->key_p15_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p16_x
  {
    size_t item_size = sizeof(ros_message->key_p16_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name key_p16_y
  {
    size_t item_size = sizeof(ros_message->key_p16_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Keypoints__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_charmie_interfaces__msg__Keypoints(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t max_serialized_size_charmie_interfaces__msg__Keypoints(
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
  // member: average_distance
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: standard_deviation
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p0_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p0_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p1_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p1_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p2_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p2_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p3_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p3_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p4_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p4_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p5_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p5_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p6_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p6_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p7_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p7_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p8_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p8_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p9_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p9_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p10_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p10_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p11_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p11_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p12_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p12_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p13_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p13_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p14_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p14_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p15_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p15_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p16_x
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: key_p16_y
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _Keypoints__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_charmie_interfaces__msg__Keypoints(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Keypoints = {
  "charmie_interfaces::msg",
  "Keypoints",
  _Keypoints__cdr_serialize,
  _Keypoints__cdr_deserialize,
  _Keypoints__get_serialized_size,
  _Keypoints__max_serialized_size
};

static rosidl_message_type_support_t _Keypoints__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Keypoints,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, Keypoints)() {
  return &_Keypoints__type_support;
}

#if defined(__cplusplus)
}
#endif
