// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from charmie_interfaces:msg/PS4Controller.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/ps4_controller__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "charmie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "charmie_interfaces/msg/detail/ps4_controller__struct.h"
#include "charmie_interfaces/msg/detail/ps4_controller__functions.h"
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


using _PS4Controller__ros_msg_type = charmie_interfaces__msg__PS4Controller;

static bool _PS4Controller__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PS4Controller__ros_msg_type * ros_message = static_cast<const _PS4Controller__ros_msg_type *>(untyped_ros_message);
  // Field name: triangle
  {
    cdr << ros_message->triangle;
  }

  // Field name: circle
  {
    cdr << ros_message->circle;
  }

  // Field name: cross
  {
    cdr << ros_message->cross;
  }

  // Field name: square
  {
    cdr << ros_message->square;
  }

  // Field name: arrow_up
  {
    cdr << ros_message->arrow_up;
  }

  // Field name: arrow_right
  {
    cdr << ros_message->arrow_right;
  }

  // Field name: arrow_down
  {
    cdr << ros_message->arrow_down;
  }

  // Field name: arrow_left
  {
    cdr << ros_message->arrow_left;
  }

  // Field name: l1
  {
    cdr << ros_message->l1;
  }

  // Field name: r1
  {
    cdr << ros_message->r1;
  }

  // Field name: l3
  {
    cdr << ros_message->l3;
  }

  // Field name: r3
  {
    cdr << ros_message->r3;
  }

  // Field name: share
  {
    cdr << ros_message->share;
  }

  // Field name: options
  {
    cdr << ros_message->options;
  }

  // Field name: ps
  {
    cdr << ros_message->ps;
  }

  // Field name: l3_ang
  {
    cdr << ros_message->l3_ang;
  }

  // Field name: l3_dist
  {
    cdr << ros_message->l3_dist;
  }

  // Field name: l3_xx
  {
    cdr << ros_message->l3_xx;
  }

  // Field name: l3_yy
  {
    cdr << ros_message->l3_yy;
  }

  // Field name: r3_ang
  {
    cdr << ros_message->r3_ang;
  }

  // Field name: r3_dist
  {
    cdr << ros_message->r3_dist;
  }

  // Field name: r3_xx
  {
    cdr << ros_message->r3_xx;
  }

  // Field name: r3_yy
  {
    cdr << ros_message->r3_yy;
  }

  // Field name: l2
  {
    cdr << ros_message->l2;
  }

  // Field name: r2
  {
    cdr << ros_message->r2;
  }

  return true;
}

static bool _PS4Controller__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PS4Controller__ros_msg_type * ros_message = static_cast<_PS4Controller__ros_msg_type *>(untyped_ros_message);
  // Field name: triangle
  {
    cdr >> ros_message->triangle;
  }

  // Field name: circle
  {
    cdr >> ros_message->circle;
  }

  // Field name: cross
  {
    cdr >> ros_message->cross;
  }

  // Field name: square
  {
    cdr >> ros_message->square;
  }

  // Field name: arrow_up
  {
    cdr >> ros_message->arrow_up;
  }

  // Field name: arrow_right
  {
    cdr >> ros_message->arrow_right;
  }

  // Field name: arrow_down
  {
    cdr >> ros_message->arrow_down;
  }

  // Field name: arrow_left
  {
    cdr >> ros_message->arrow_left;
  }

  // Field name: l1
  {
    cdr >> ros_message->l1;
  }

  // Field name: r1
  {
    cdr >> ros_message->r1;
  }

  // Field name: l3
  {
    cdr >> ros_message->l3;
  }

  // Field name: r3
  {
    cdr >> ros_message->r3;
  }

  // Field name: share
  {
    cdr >> ros_message->share;
  }

  // Field name: options
  {
    cdr >> ros_message->options;
  }

  // Field name: ps
  {
    cdr >> ros_message->ps;
  }

  // Field name: l3_ang
  {
    cdr >> ros_message->l3_ang;
  }

  // Field name: l3_dist
  {
    cdr >> ros_message->l3_dist;
  }

  // Field name: l3_xx
  {
    cdr >> ros_message->l3_xx;
  }

  // Field name: l3_yy
  {
    cdr >> ros_message->l3_yy;
  }

  // Field name: r3_ang
  {
    cdr >> ros_message->r3_ang;
  }

  // Field name: r3_dist
  {
    cdr >> ros_message->r3_dist;
  }

  // Field name: r3_xx
  {
    cdr >> ros_message->r3_xx;
  }

  // Field name: r3_yy
  {
    cdr >> ros_message->r3_yy;
  }

  // Field name: l2
  {
    cdr >> ros_message->l2;
  }

  // Field name: r2
  {
    cdr >> ros_message->r2;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t get_serialized_size_charmie_interfaces__msg__PS4Controller(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PS4Controller__ros_msg_type * ros_message = static_cast<const _PS4Controller__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name triangle
  {
    size_t item_size = sizeof(ros_message->triangle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name circle
  {
    size_t item_size = sizeof(ros_message->circle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cross
  {
    size_t item_size = sizeof(ros_message->cross);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name square
  {
    size_t item_size = sizeof(ros_message->square);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arrow_up
  {
    size_t item_size = sizeof(ros_message->arrow_up);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arrow_right
  {
    size_t item_size = sizeof(ros_message->arrow_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arrow_down
  {
    size_t item_size = sizeof(ros_message->arrow_down);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arrow_left
  {
    size_t item_size = sizeof(ros_message->arrow_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l1
  {
    size_t item_size = sizeof(ros_message->l1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r1
  {
    size_t item_size = sizeof(ros_message->r1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l3
  {
    size_t item_size = sizeof(ros_message->l3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r3
  {
    size_t item_size = sizeof(ros_message->r3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name share
  {
    size_t item_size = sizeof(ros_message->share);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name options
  {
    size_t item_size = sizeof(ros_message->options);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ps
  {
    size_t item_size = sizeof(ros_message->ps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l3_ang
  {
    size_t item_size = sizeof(ros_message->l3_ang);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l3_dist
  {
    size_t item_size = sizeof(ros_message->l3_dist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l3_xx
  {
    size_t item_size = sizeof(ros_message->l3_xx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l3_yy
  {
    size_t item_size = sizeof(ros_message->l3_yy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r3_ang
  {
    size_t item_size = sizeof(ros_message->r3_ang);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r3_dist
  {
    size_t item_size = sizeof(ros_message->r3_dist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r3_xx
  {
    size_t item_size = sizeof(ros_message->r3_xx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r3_yy
  {
    size_t item_size = sizeof(ros_message->r3_yy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name l2
  {
    size_t item_size = sizeof(ros_message->l2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name r2
  {
    size_t item_size = sizeof(ros_message->r2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PS4Controller__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_charmie_interfaces__msg__PS4Controller(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t max_serialized_size_charmie_interfaces__msg__PS4Controller(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: triangle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: circle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cross
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: square
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: arrow_up
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: arrow_right
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: arrow_down
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: arrow_left
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: l1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: r1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: l3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: r3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: share
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: options
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: ps
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: l3_ang
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: l3_dist
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: l3_xx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: l3_yy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: r3_ang
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: r3_dist
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: r3_xx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: r3_yy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: l2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: r2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _PS4Controller__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_charmie_interfaces__msg__PS4Controller(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_PS4Controller = {
  "charmie_interfaces::msg",
  "PS4Controller",
  _PS4Controller__cdr_serialize,
  _PS4Controller__cdr_deserialize,
  _PS4Controller__get_serialized_size,
  _PS4Controller__max_serialized_size
};

static rosidl_message_type_support_t _PS4Controller__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PS4Controller,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, PS4Controller)() {
  return &_PS4Controller__type_support;
}

#if defined(__cplusplus)
}
#endif
