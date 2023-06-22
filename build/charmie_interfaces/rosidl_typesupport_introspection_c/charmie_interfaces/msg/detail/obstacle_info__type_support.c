// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from charmie_interfaces:msg/ObstacleInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "charmie_interfaces/msg/detail/obstacle_info__rosidl_typesupport_introspection_c.h"
#include "charmie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "charmie_interfaces/msg/detail/obstacle_info__functions.h"
#include "charmie_interfaces/msg/detail/obstacle_info__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  charmie_interfaces__msg__ObstacleInfo__init(message_memory);
}

void ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_fini_function(void * message_memory)
{
  charmie_interfaces__msg__ObstacleInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_message_member_array[4] = {
  {
    "alfa",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__ObstacleInfo, alfa),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__ObstacleInfo, dist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "length_cm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__ObstacleInfo, length_cm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "length_degrees",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__ObstacleInfo, length_degrees),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_message_members = {
  "charmie_interfaces__msg",  // message namespace
  "ObstacleInfo",  // message name
  4,  // number of fields
  sizeof(charmie_interfaces__msg__ObstacleInfo),
  ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_message_member_array,  // message members
  ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_message_type_support_handle = {
  0,
  &ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, charmie_interfaces, msg, ObstacleInfo)() {
  if (!ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_message_type_support_handle.typesupport_identifier) {
    ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ObstacleInfo__rosidl_typesupport_introspection_c__ObstacleInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
