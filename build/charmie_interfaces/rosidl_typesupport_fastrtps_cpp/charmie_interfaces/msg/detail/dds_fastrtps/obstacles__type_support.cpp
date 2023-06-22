// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/Obstacles.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/obstacles__rosidl_typesupport_fastrtps_cpp.hpp"
#include "charmie_interfaces/msg/detail/obstacles__struct.hpp"

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
bool cdr_serialize(
  const charmie_interfaces::msg::ObstacleInfo &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  charmie_interfaces::msg::ObstacleInfo &);
size_t get_serialized_size(
  const charmie_interfaces::msg::ObstacleInfo &,
  size_t current_alignment);
size_t
max_serialized_size_ObstacleInfo(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace charmie_interfaces


namespace charmie_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_serialize(
  const charmie_interfaces::msg::Obstacles & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: no_obstacles
  cdr << ros_message.no_obstacles;
  // Member: obstacles
  {
    size_t size = ros_message.obstacles.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      charmie_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.obstacles[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  charmie_interfaces::msg::Obstacles & ros_message)
{
  // Member: no_obstacles
  cdr >> ros_message.no_obstacles;

  // Member: obstacles
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.obstacles.resize(size);
    for (size_t i = 0; i < size; i++) {
      charmie_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.obstacles[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
get_serialized_size(
  const charmie_interfaces::msg::Obstacles & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: no_obstacles
  {
    size_t item_size = sizeof(ros_message.no_obstacles);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: obstacles
  {
    size_t array_size = ros_message.obstacles.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        charmie_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.obstacles[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
max_serialized_size_Obstacles(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: no_obstacles
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: obstacles
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        charmie_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_ObstacleInfo(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _Obstacles__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::Obstacles *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Obstacles__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<charmie_interfaces::msg::Obstacles *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Obstacles__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::Obstacles *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Obstacles__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Obstacles(full_bounded, 0);
}

static message_type_support_callbacks_t _Obstacles__callbacks = {
  "charmie_interfaces::msg",
  "Obstacles",
  _Obstacles__cdr_serialize,
  _Obstacles__cdr_deserialize,
  _Obstacles__get_serialized_size,
  _Obstacles__max_serialized_size
};

static rosidl_message_type_support_t _Obstacles__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Obstacles__callbacks,
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
get_message_type_support_handle<charmie_interfaces::msg::Obstacles>()
{
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_Obstacles__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, charmie_interfaces, msg, Obstacles)() {
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_Obstacles__handle;
}

#ifdef __cplusplus
}
#endif
