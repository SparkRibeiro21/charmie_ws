// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/SpeechType.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/speech_type__rosidl_typesupport_fastrtps_cpp.hpp"
#include "charmie_interfaces/msg/detail/speech_type__struct.hpp"

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
  const charmie_interfaces::msg::SpeechType & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: yes_or_no
  cdr << (ros_message.yes_or_no ? true : false);
  // Member: receptionist
  cdr << (ros_message.receptionist ? true : false);
  // Member: gpsr
  cdr << (ros_message.gpsr ? true : false);
  // Member: restaurant
  cdr << (ros_message.restaurant ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  charmie_interfaces::msg::SpeechType & ros_message)
{
  // Member: yes_or_no
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.yes_or_no = tmp ? true : false;
  }

  // Member: receptionist
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.receptionist = tmp ? true : false;
  }

  // Member: gpsr
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.gpsr = tmp ? true : false;
  }

  // Member: restaurant
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.restaurant = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
get_serialized_size(
  const charmie_interfaces::msg::SpeechType & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: yes_or_no
  {
    size_t item_size = sizeof(ros_message.yes_or_no);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: receptionist
  {
    size_t item_size = sizeof(ros_message.receptionist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gpsr
  {
    size_t item_size = sizeof(ros_message.gpsr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: restaurant
  {
    size_t item_size = sizeof(ros_message.restaurant);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
max_serialized_size_SpeechType(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: yes_or_no
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: receptionist
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: gpsr
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: restaurant
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _SpeechType__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::SpeechType *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _SpeechType__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<charmie_interfaces::msg::SpeechType *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _SpeechType__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::SpeechType *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _SpeechType__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_SpeechType(full_bounded, 0);
}

static message_type_support_callbacks_t _SpeechType__callbacks = {
  "charmie_interfaces::msg",
  "SpeechType",
  _SpeechType__cdr_serialize,
  _SpeechType__cdr_deserialize,
  _SpeechType__get_serialized_size,
  _SpeechType__max_serialized_size
};

static rosidl_message_type_support_t _SpeechType__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_SpeechType__callbacks,
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
get_message_type_support_handle<charmie_interfaces::msg::SpeechType>()
{
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_SpeechType__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, charmie_interfaces, msg, SpeechType)() {
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_SpeechType__handle;
}

#ifdef __cplusplus
}
#endif
