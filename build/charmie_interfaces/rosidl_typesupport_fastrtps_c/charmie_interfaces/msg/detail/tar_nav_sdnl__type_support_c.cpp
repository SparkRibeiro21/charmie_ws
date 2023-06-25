// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from charmie_interfaces:msg/TarNavSDNL.idl
// generated code does not contain a copyright notice
#include "charmie_interfaces/msg/detail/tar_nav_sdnl__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "charmie_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "charmie_interfaces/msg/detail/tar_nav_sdnl__struct.h"
#include "charmie_interfaces/msg/detail/tar_nav_sdnl__functions.h"
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

#include "geometry_msgs/msg/detail/pose2_d__functions.h"  // move_target_coordinates, rotate_target_coordinates

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_charmie_interfaces
size_t get_serialized_size_geometry_msgs__msg__Pose2D(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_charmie_interfaces
size_t max_serialized_size_geometry_msgs__msg__Pose2D(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_charmie_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose2D)();


using _TarNavSDNL__ros_msg_type = charmie_interfaces__msg__TarNavSDNL;

static bool _TarNavSDNL__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TarNavSDNL__ros_msg_type * ros_message = static_cast<const _TarNavSDNL__ros_msg_type *>(untyped_ros_message);
  // Field name: move_target_coordinates
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose2D
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->move_target_coordinates, cdr))
    {
      return false;
    }
  }

  // Field name: rotate_target_coordinates
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose2D
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->rotate_target_coordinates, cdr))
    {
      return false;
    }
  }

  // Field name: flag_not_obs
  {
    cdr << (ros_message->flag_not_obs ? true : false);
  }

  return true;
}

static bool _TarNavSDNL__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TarNavSDNL__ros_msg_type * ros_message = static_cast<_TarNavSDNL__ros_msg_type *>(untyped_ros_message);
  // Field name: move_target_coordinates
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose2D
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->move_target_coordinates))
    {
      return false;
    }
  }

  // Field name: rotate_target_coordinates
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, geometry_msgs, msg, Pose2D
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->rotate_target_coordinates))
    {
      return false;
    }
  }

  // Field name: flag_not_obs
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->flag_not_obs = tmp ? true : false;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t get_serialized_size_charmie_interfaces__msg__TarNavSDNL(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TarNavSDNL__ros_msg_type * ros_message = static_cast<const _TarNavSDNL__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name move_target_coordinates

  current_alignment += get_serialized_size_geometry_msgs__msg__Pose2D(
    &(ros_message->move_target_coordinates), current_alignment);
  // field.name rotate_target_coordinates

  current_alignment += get_serialized_size_geometry_msgs__msg__Pose2D(
    &(ros_message->rotate_target_coordinates), current_alignment);
  // field.name flag_not_obs
  {
    size_t item_size = sizeof(ros_message->flag_not_obs);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TarNavSDNL__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_charmie_interfaces__msg__TarNavSDNL(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_charmie_interfaces
size_t max_serialized_size_charmie_interfaces__msg__TarNavSDNL(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: move_target_coordinates
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Pose2D(
        full_bounded, current_alignment);
    }
  }
  // member: rotate_target_coordinates
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_geometry_msgs__msg__Pose2D(
        full_bounded, current_alignment);
    }
  }
  // member: flag_not_obs
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _TarNavSDNL__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_charmie_interfaces__msg__TarNavSDNL(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_TarNavSDNL = {
  "charmie_interfaces::msg",
  "TarNavSDNL",
  _TarNavSDNL__cdr_serialize,
  _TarNavSDNL__cdr_deserialize,
  _TarNavSDNL__get_serialized_size,
  _TarNavSDNL__max_serialized_size
};

static rosidl_message_type_support_t _TarNavSDNL__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TarNavSDNL,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, charmie_interfaces, msg, TarNavSDNL)() {
  return &_TarNavSDNL__type_support;
}

#if defined(__cplusplus)
}
#endif
