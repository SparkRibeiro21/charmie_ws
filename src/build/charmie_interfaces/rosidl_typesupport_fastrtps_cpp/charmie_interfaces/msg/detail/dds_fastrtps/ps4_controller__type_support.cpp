// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/PS4Controller.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/ps4_controller__rosidl_typesupport_fastrtps_cpp.hpp"
#include "charmie_interfaces/msg/detail/ps4_controller__struct.hpp"

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
  const charmie_interfaces::msg::PS4Controller & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: triangle
  cdr << ros_message.triangle;
  // Member: circle
  cdr << ros_message.circle;
  // Member: cross
  cdr << ros_message.cross;
  // Member: square
  cdr << ros_message.square;
  // Member: arrow_up
  cdr << ros_message.arrow_up;
  // Member: arrow_right
  cdr << ros_message.arrow_right;
  // Member: arrow_down
  cdr << ros_message.arrow_down;
  // Member: arrow_left
  cdr << ros_message.arrow_left;
  // Member: l1
  cdr << ros_message.l1;
  // Member: r1
  cdr << ros_message.r1;
  // Member: l3
  cdr << ros_message.l3;
  // Member: r3
  cdr << ros_message.r3;
  // Member: share
  cdr << ros_message.share;
  // Member: options
  cdr << ros_message.options;
  // Member: ps
  cdr << ros_message.ps;
  // Member: l3_ang
  cdr << ros_message.l3_ang;
  // Member: l3_dist
  cdr << ros_message.l3_dist;
  // Member: l3_xx
  cdr << ros_message.l3_xx;
  // Member: l3_yy
  cdr << ros_message.l3_yy;
  // Member: r3_ang
  cdr << ros_message.r3_ang;
  // Member: r3_dist
  cdr << ros_message.r3_dist;
  // Member: r3_xx
  cdr << ros_message.r3_xx;
  // Member: r3_yy
  cdr << ros_message.r3_yy;
  // Member: l2
  cdr << ros_message.l2;
  // Member: r2
  cdr << ros_message.r2;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  charmie_interfaces::msg::PS4Controller & ros_message)
{
  // Member: triangle
  cdr >> ros_message.triangle;

  // Member: circle
  cdr >> ros_message.circle;

  // Member: cross
  cdr >> ros_message.cross;

  // Member: square
  cdr >> ros_message.square;

  // Member: arrow_up
  cdr >> ros_message.arrow_up;

  // Member: arrow_right
  cdr >> ros_message.arrow_right;

  // Member: arrow_down
  cdr >> ros_message.arrow_down;

  // Member: arrow_left
  cdr >> ros_message.arrow_left;

  // Member: l1
  cdr >> ros_message.l1;

  // Member: r1
  cdr >> ros_message.r1;

  // Member: l3
  cdr >> ros_message.l3;

  // Member: r3
  cdr >> ros_message.r3;

  // Member: share
  cdr >> ros_message.share;

  // Member: options
  cdr >> ros_message.options;

  // Member: ps
  cdr >> ros_message.ps;

  // Member: l3_ang
  cdr >> ros_message.l3_ang;

  // Member: l3_dist
  cdr >> ros_message.l3_dist;

  // Member: l3_xx
  cdr >> ros_message.l3_xx;

  // Member: l3_yy
  cdr >> ros_message.l3_yy;

  // Member: r3_ang
  cdr >> ros_message.r3_ang;

  // Member: r3_dist
  cdr >> ros_message.r3_dist;

  // Member: r3_xx
  cdr >> ros_message.r3_xx;

  // Member: r3_yy
  cdr >> ros_message.r3_yy;

  // Member: l2
  cdr >> ros_message.l2;

  // Member: r2
  cdr >> ros_message.r2;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
get_serialized_size(
  const charmie_interfaces::msg::PS4Controller & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: triangle
  {
    size_t item_size = sizeof(ros_message.triangle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: circle
  {
    size_t item_size = sizeof(ros_message.circle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cross
  {
    size_t item_size = sizeof(ros_message.cross);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: square
  {
    size_t item_size = sizeof(ros_message.square);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: arrow_up
  {
    size_t item_size = sizeof(ros_message.arrow_up);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: arrow_right
  {
    size_t item_size = sizeof(ros_message.arrow_right);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: arrow_down
  {
    size_t item_size = sizeof(ros_message.arrow_down);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: arrow_left
  {
    size_t item_size = sizeof(ros_message.arrow_left);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l1
  {
    size_t item_size = sizeof(ros_message.l1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: r1
  {
    size_t item_size = sizeof(ros_message.r1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l3
  {
    size_t item_size = sizeof(ros_message.l3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: r3
  {
    size_t item_size = sizeof(ros_message.r3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: share
  {
    size_t item_size = sizeof(ros_message.share);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: options
  {
    size_t item_size = sizeof(ros_message.options);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ps
  {
    size_t item_size = sizeof(ros_message.ps);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l3_ang
  {
    size_t item_size = sizeof(ros_message.l3_ang);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l3_dist
  {
    size_t item_size = sizeof(ros_message.l3_dist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l3_xx
  {
    size_t item_size = sizeof(ros_message.l3_xx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l3_yy
  {
    size_t item_size = sizeof(ros_message.l3_yy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: r3_ang
  {
    size_t item_size = sizeof(ros_message.r3_ang);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: r3_dist
  {
    size_t item_size = sizeof(ros_message.r3_dist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: r3_xx
  {
    size_t item_size = sizeof(ros_message.r3_xx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: r3_yy
  {
    size_t item_size = sizeof(ros_message.r3_yy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: l2
  {
    size_t item_size = sizeof(ros_message.l2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: r2
  {
    size_t item_size = sizeof(ros_message.r2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
max_serialized_size_PS4Controller(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: triangle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: circle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cross
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: square
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: arrow_up
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: arrow_right
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: arrow_down
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: arrow_left
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: l1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: r1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: l3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: r3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: share
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: options
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ps
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: l3_ang
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: l3_dist
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: l3_xx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: l3_yy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: r3_ang
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: r3_dist
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: r3_xx
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: r3_yy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: l2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: r2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _PS4Controller__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::PS4Controller *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PS4Controller__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<charmie_interfaces::msg::PS4Controller *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PS4Controller__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::PS4Controller *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PS4Controller__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_PS4Controller(full_bounded, 0);
}

static message_type_support_callbacks_t _PS4Controller__callbacks = {
  "charmie_interfaces::msg",
  "PS4Controller",
  _PS4Controller__cdr_serialize,
  _PS4Controller__cdr_deserialize,
  _PS4Controller__get_serialized_size,
  _PS4Controller__max_serialized_size
};

static rosidl_message_type_support_t _PS4Controller__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PS4Controller__callbacks,
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
get_message_type_support_handle<charmie_interfaces::msg::PS4Controller>()
{
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_PS4Controller__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, charmie_interfaces, msg, PS4Controller)() {
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_PS4Controller__handle;
}

#ifdef __cplusplus
}
#endif
