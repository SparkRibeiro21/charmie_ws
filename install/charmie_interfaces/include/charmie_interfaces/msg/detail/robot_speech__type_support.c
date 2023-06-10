// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from charmie_interfaces:msg/RobotSpeech.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "charmie_interfaces/msg/detail/robot_speech__rosidl_typesupport_introspection_c.h"
#include "charmie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "charmie_interfaces/msg/detail/robot_speech__functions.h"
#include "charmie_interfaces/msg/detail/robot_speech__struct.h"


// Include directives for member types
// Member `command`
// Member `language`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  charmie_interfaces__msg__RobotSpeech__init(message_memory);
}

void RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_fini_function(void * message_memory)
{
  charmie_interfaces__msg__RobotSpeech__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_message_member_array[2] = {
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__RobotSpeech, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "language",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__RobotSpeech, language),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_message_members = {
  "charmie_interfaces__msg",  // message namespace
  "RobotSpeech",  // message name
  2,  // number of fields
  sizeof(charmie_interfaces__msg__RobotSpeech),
  RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_message_member_array,  // message members
  RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_message_type_support_handle = {
  0,
  &RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, charmie_interfaces, msg, RobotSpeech)() {
  if (!RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_message_type_support_handle.typesupport_identifier) {
    RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &RobotSpeech__rosidl_typesupport_introspection_c__RobotSpeech_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
