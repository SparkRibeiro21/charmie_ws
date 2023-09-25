// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from realsense2_camera_msgs:msg/IMUInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "realsense2_camera_msgs/msg/detail/imu_info__rosidl_typesupport_introspection_c.h"
#include "realsense2_camera_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "realsense2_camera_msgs/msg/detail/imu_info__functions.h"
#include "realsense2_camera_msgs/msg/detail/imu_info__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  realsense2_camera_msgs__msg__IMUInfo__init(message_memory);
}

void IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_fini_function(void * message_memory)
{
  realsense2_camera_msgs__msg__IMUInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__msg__IMUInfo, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__msg__IMUInfo, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "noise_variances",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__msg__IMUInfo, noise_variances),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bias_variances",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(realsense2_camera_msgs__msg__IMUInfo, bias_variances),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_message_members = {
  "realsense2_camera_msgs__msg",  // message namespace
  "IMUInfo",  // message name
  4,  // number of fields
  sizeof(realsense2_camera_msgs__msg__IMUInfo),
  IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_message_member_array,  // message members
  IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_message_type_support_handle = {
  0,
  &IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_realsense2_camera_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, realsense2_camera_msgs, msg, IMUInfo)() {
  IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_message_type_support_handle.typesupport_identifier) {
    IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &IMUInfo__rosidl_typesupport_introspection_c__IMUInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
