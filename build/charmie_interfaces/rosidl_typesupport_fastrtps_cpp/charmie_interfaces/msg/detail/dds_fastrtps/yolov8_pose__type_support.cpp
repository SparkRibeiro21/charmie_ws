// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/Yolov8Pose.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/yolov8_pose__rosidl_typesupport_fastrtps_cpp.hpp"
#include "charmie_interfaces/msg/detail/yolov8_pose__struct.hpp"

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
  const charmie_interfaces::msg::Keypoints &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  charmie_interfaces::msg::Keypoints &);
size_t get_serialized_size(
  const charmie_interfaces::msg::Keypoints &,
  size_t current_alignment);
size_t
max_serialized_size_Keypoints(
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
  const charmie_interfaces::msg::Yolov8Pose & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: num_person
  cdr << ros_message.num_person;
  // Member: keypoints
  {
    size_t size = ros_message.keypoints.size();
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      charmie_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.keypoints[i],
        cdr);
    }
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  charmie_interfaces::msg::Yolov8Pose & ros_message)
{
  // Member: num_person
  cdr >> ros_message.num_person;

  // Member: keypoints
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.keypoints.resize(size);
    for (size_t i = 0; i < size; i++) {
      charmie_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.keypoints[i]);
    }
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
get_serialized_size(
  const charmie_interfaces::msg::Yolov8Pose & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: num_person
  {
    size_t item_size = sizeof(ros_message.num_person);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: keypoints
  {
    size_t array_size = ros_message.keypoints.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        charmie_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.keypoints[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_charmie_interfaces
max_serialized_size_Yolov8Pose(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: num_person
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: keypoints
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        charmie_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Keypoints(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static bool _Yolov8Pose__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::Yolov8Pose *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Yolov8Pose__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<charmie_interfaces::msg::Yolov8Pose *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Yolov8Pose__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const charmie_interfaces::msg::Yolov8Pose *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Yolov8Pose__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Yolov8Pose(full_bounded, 0);
}

static message_type_support_callbacks_t _Yolov8Pose__callbacks = {
  "charmie_interfaces::msg",
  "Yolov8Pose",
  _Yolov8Pose__cdr_serialize,
  _Yolov8Pose__cdr_deserialize,
  _Yolov8Pose__get_serialized_size,
  _Yolov8Pose__max_serialized_size
};

static rosidl_message_type_support_t _Yolov8Pose__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Yolov8Pose__callbacks,
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
get_message_type_support_handle<charmie_interfaces::msg::Yolov8Pose>()
{
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_Yolov8Pose__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, charmie_interfaces, msg, Yolov8Pose)() {
  return &charmie_interfaces::msg::typesupport_fastrtps_cpp::_Yolov8Pose__handle;
}

#ifdef __cplusplus
}
#endif
