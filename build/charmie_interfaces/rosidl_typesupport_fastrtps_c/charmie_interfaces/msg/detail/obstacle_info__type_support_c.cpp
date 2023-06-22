// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from charmie_interfaces:msg/ObstacleInfo.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/obstacle_info__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "charmie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "charmie_interfaces/msg/detail/obstacle_info__struct.h"
#include "charmie_interfaces/msg/detail/obstacle_info__functions.h"
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


using _ObstacleInfo__ros_msg_type = charmie_interfaces__msg__ObstacleInfo;

static bool _ObstacleInfo__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ObstacleInfo__ros_msg_type * ros_message = static_cast<const _ObstacleInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: alfa
  {
    cdr << ros_message->alfa;
  }

  // Field name: dist
  {
    cdr << ros_message->dist;
  }

  // Field name: length_cm
  {
    cdr << ros_message->length_cm;
  }

  // Field name: length_degrees
  {
    cdr << ros_message->length_degrees;
  }

  return true;
}

static bool _ObstacleInfo__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ObstacleInfo__ros_msg_type * ros_message = static_cast<_ObstacleInfo__ros_msg_type *>(untyped_ros_message);
  // Field name: alfa
  {
    cdr >> ros_message->alfa;
  }

  // Field name: dist
  {
    cdr >> ros_message->dist;
  }

  // Field name: length_cm
  {
    cdr >> ros_message->length_cm;
  }

  // Field name: length_degrees
  {
    cdr >> ros_message->length_degrees;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t get_serialized_size_charmie_interfaces__msg__ObstacleInfo(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ObstacleInfo__ros_msg_type * ros_message = static_cast<const _ObstacleInfo__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name alfa
  {
    size_t item_size = sizeof(ros_message->alfa);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dist
  {
    size_t item_size = sizeof(ros_message->dist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name length_cm
  {
    size_t item_size = sizeof(ros_message->length_cm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name length_degrees
  {
    size_t item_size = sizeof(ros_message->length_degrees);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ObstacleInfo__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_charmie_interfaces__msg__ObstacleInfo(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t max_serialized_size_charmie_interfaces__msg__ObstacleInfo(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: alfa
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: dist
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: length_cm
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: length_degrees
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _ObstacleInfo__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_charmie_interfaces__msg__ObstacleInfo(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_ObstacleInfo = {
  "charmie_interfaces::msg",
  "ObstacleInfo",
  _ObstacleInfo__cdr_serialize,
  _ObstacleInfo__cdr_deserialize,
  _ObstacleInfo__get_serialized_size,
  _ObstacleInfo__max_serialized_size
};

static rosidl_message_type_support_t _ObstacleInfo__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ObstacleInfo,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, ObstacleInfo)() {
  return &_ObstacleInfo__type_support;
}

#if defined(__cplusplus)
}
#endif
