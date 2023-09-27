// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/MultiObjects.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/multi_objects__rosidl_typesupport_fastrtps_cpp.hpp"
#include "charmie_interfaces/msg/detail/multi_objects__struct.hpp"

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
  const charmie_interfaces::msg::MultiObjects & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: objects
  {
    cdr << ros_message.objects;
  }
  // Member: confidence
  {
    cdr << ros_message.confidence;
  }
  // Member: distance
  {
    cdr << ros_message.distance;
  }
  // Member: position
  {
    cdr << ros_message.position;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  charmie_interfaces::msg::MultiObjects & ros_message)
{
  // Member: objects
  {
    cdr >> ros_message.objects;
  }

  // Member: confidence
  {
    cdr >> ros_message.confidence;
  }

  // Member: distance
  {
    cdr >> ros_message.distance;
  }

  // Member: position
  {
    cdr >> ros_message.position;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
get_serialized_size(
  const charmie_interfaces::msg::MultiObjects & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: objects
  {
    size_t array_size = ros_message.objects.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.objects[index].size() + 1);
    }
  }
  // Member: confidence
  {
    size_t array_size = ros_message.confidence.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.confidence[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: distance
  {
    size_t array_size = ros_message.distance.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.distance[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: position
  {
    size_t array_size = ros_message.position.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
max_serialized_size_MultiObjects(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: objects
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: confidence
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: distance
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: position
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _MultiObjects__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::MultiObjects *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _MultiObjects__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<charmie_interfaces::msg::MultiObjects *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _MultiObjects__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::MultiObjects *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _MultiObjects__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_MultiObjects(full_bounded, 0);
}

static message_type_support_callbacks_t _MultiObjects__callbacks = {
  "charmie_interfaces::msg",
  "MultiObjects",
  _MultiObjects__cdr_serialize,
  _MultiObjects__cdr_deserialize,
  _MultiObjects__get_serialized_size,
  _MultiObjects__max_serialized_size
};

static rosidl_message_type_support_t _MultiObjects__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_MultiObjects__callbacks,
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
get_message_type_support_handle<charmie_interfaces::msg::MultiObjects>()
{
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_MultiObjects__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, charmie_interfaces, msg, MultiObjects)() {
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_MultiObjects__handle;
}

#ifdef __cplusplus
}
#endif
