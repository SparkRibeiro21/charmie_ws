// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from charmie_interfaces:msg/SpeechType.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/speech_type__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "charmie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "charmie_interfaces/msg/detail/speech_type__struct.h"
#include "charmie_interfaces/msg/detail/speech_type__functions.h"
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


using _SpeechType__ros_msg_type = charmie_interfaces__msg__SpeechType;

static bool _SpeechType__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _SpeechType__ros_msg_type * ros_message = static_cast<const _SpeechType__ros_msg_type *>(untyped_ros_message);
  // Field name: yes_or_no
  {
    cdr << (ros_message->yes_or_no ? true : false);
  }

  // Field name: receptionist
  {
    cdr << (ros_message->receptionist ? true : false);
  }

  // Field name: gpsr
  {
    cdr << (ros_message->gpsr ? true : false);
  }

  // Field name: restaurant
  {
    cdr << (ros_message->restaurant ? true : false);
  }

  return true;
}

static bool _SpeechType__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _SpeechType__ros_msg_type * ros_message = static_cast<_SpeechType__ros_msg_type *>(untyped_ros_message);
  // Field name: yes_or_no
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->yes_or_no = tmp ? true : false;
  }

  // Field name: receptionist
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->receptionist = tmp ? true : false;
  }

  // Field name: gpsr
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->gpsr = tmp ? true : false;
  }

  // Field name: restaurant
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->restaurant = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t get_serialized_size_charmie_interfaces__msg__SpeechType(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SpeechType__ros_msg_type * ros_message = static_cast<const _SpeechType__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name yes_or_no
  {
    size_t item_size = sizeof(ros_message->yes_or_no);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name receptionist
  {
    size_t item_size = sizeof(ros_message->receptionist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gpsr
  {
    size_t item_size = sizeof(ros_message->gpsr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name restaurant
  {
    size_t item_size = sizeof(ros_message->restaurant);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _SpeechType__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_charmie_interfaces__msg__SpeechType(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t max_serialized_size_charmie_interfaces__msg__SpeechType(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: yes_or_no
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: receptionist
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: gpsr
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: restaurant
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _SpeechType__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_charmie_interfaces__msg__SpeechType(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_SpeechType = {
  "charmie_interfaces::msg",
  "SpeechType",
  _SpeechType__cdr_serialize,
  _SpeechType__cdr_deserialize,
  _SpeechType__get_serialized_size,
  _SpeechType__max_serialized_size
};

static rosidl_message_type_support_t _SpeechType__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SpeechType,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, SpeechType)() {
  return &_SpeechType__type_support;
}

#if defined(__cplusplus)
}
#endif
