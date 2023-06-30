// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from charmie_interfaces:msg/Yolov8PoseArray.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/yolov8_pose_array__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "charmie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "charmie_interfaces/msg/detail/yolov8_pose_array__struct.h"
#include "charmie_interfaces/msg/detail/yolov8_pose_array__functions.h"
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

#include "charmie_interfaces/msg/detail/yolov8_pose__functions.h"  // data

// forward declare type support functions
size_t get_serialized_size_charmie_interfaces__msg__Yolov8Pose(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_charmie_interfaces__msg__Yolov8Pose(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, Yolov8Pose)();


using _Yolov8PoseArray__ros_msg_type = charmie_interfaces__msg__Yolov8PoseArray;

static bool _Yolov8PoseArray__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Yolov8PoseArray__ros_msg_type * ros_message = static_cast<const _Yolov8PoseArray__ros_msg_type *>(untyped_ros_message);
  // Field name: data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, Yolov8Pose
      )()->data);
    size_t size = ros_message->data.size;
    auto array_ptr = ros_message->data.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  return true;
}

static bool _Yolov8PoseArray__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Yolov8PoseArray__ros_msg_type * ros_message = static_cast<_Yolov8PoseArray__ros_msg_type *>(untyped_ros_message);
  // Field name: data
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, Yolov8Pose
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->data.data) {
      charmie_interfaces__msg__Yolov8Pose__Sequence__fini(&ros_message->data);
    }
    if (!charmie_interfaces__msg__Yolov8Pose__Sequence__init(&ros_message->data, size)) {
      return "failed to create array for field 'data'";
    }
    auto array_ptr = ros_message->data.data;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t get_serialized_size_charmie_interfaces__msg__Yolov8PoseArray(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Yolov8PoseArray__ros_msg_type * ros_message = static_cast<const _Yolov8PoseArray__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name data
  {
    size_t array_size = ros_message->data.size;
    auto array_ptr = ros_message->data.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_charmie_interfaces__msg__Yolov8Pose(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Yolov8PoseArray__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_charmie_interfaces__msg__Yolov8PoseArray(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t max_serialized_size_charmie_interfaces__msg__Yolov8PoseArray(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: data
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_charmie_interfaces__msg__Yolov8Pose(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _Yolov8PoseArray__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_charmie_interfaces__msg__Yolov8PoseArray(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Yolov8PoseArray = {
  "charmie_interfaces::msg",
  "Yolov8PoseArray",
  _Yolov8PoseArray__cdr_serialize,
  _Yolov8PoseArray__cdr_deserialize,
  _Yolov8PoseArray__get_serialized_size,
  _Yolov8PoseArray__max_serialized_size
};

static rosidl_message_type_support_t _Yolov8PoseArray__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Yolov8PoseArray,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, Yolov8PoseArray)() {
  return &_Yolov8PoseArray__type_support;
}

#if defined(__cplusplus)
}
#endif
