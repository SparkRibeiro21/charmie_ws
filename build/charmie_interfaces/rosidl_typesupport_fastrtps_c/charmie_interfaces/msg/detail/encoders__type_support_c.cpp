// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from charmie_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/encoders__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "charmie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "charmie_interfaces/msg/detail/encoders__struct.h"
#include "charmie_interfaces/msg/detail/encoders__functions.h"
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


using _Encoders__ros_msg_type = charmie_interfaces__msg__Encoders;

static bool _Encoders__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Encoders__ros_msg_type * ros_message = static_cast<const _Encoders__ros_msg_type *>(untyped_ros_message);
  // Field name: enc_m1
  {
    cdr << ros_message->enc_m1;
  }

  // Field name: enc_m2
  {
    cdr << ros_message->enc_m2;
  }

  // Field name: enc_m3
  {
    cdr << ros_message->enc_m3;
  }

  // Field name: enc_m4
  {
    cdr << ros_message->enc_m4;
  }

  return true;
}

static bool _Encoders__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Encoders__ros_msg_type * ros_message = static_cast<_Encoders__ros_msg_type *>(untyped_ros_message);
  // Field name: enc_m1
  {
    cdr >> ros_message->enc_m1;
  }

  // Field name: enc_m2
  {
    cdr >> ros_message->enc_m2;
  }

  // Field name: enc_m3
  {
    cdr >> ros_message->enc_m3;
  }

  // Field name: enc_m4
  {
    cdr >> ros_message->enc_m4;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t get_serialized_size_charmie_interfaces__msg__Encoders(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Encoders__ros_msg_type * ros_message = static_cast<const _Encoders__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name enc_m1
  {
    size_t item_size = sizeof(ros_message->enc_m1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name enc_m2
  {
    size_t item_size = sizeof(ros_message->enc_m2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name enc_m3
  {
    size_t item_size = sizeof(ros_message->enc_m3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name enc_m4
  {
    size_t item_size = sizeof(ros_message->enc_m4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Encoders__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_charmie_interfaces__msg__Encoders(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t max_serialized_size_charmie_interfaces__msg__Encoders(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: enc_m1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: enc_m2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: enc_m3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: enc_m4
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _Encoders__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_charmie_interfaces__msg__Encoders(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Encoders = {
  "charmie_interfaces::msg",
  "Encoders",
  _Encoders__cdr_serialize,
  _Encoders__cdr_deserialize,
  _Encoders__get_serialized_size,
  _Encoders__max_serialized_size
};

static rosidl_message_type_support_t _Encoders__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Encoders,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, Encoders)() {
  return &_Encoders__type_support;
}

#if defined(__cplusplus)
}
#endif
