// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from charmie_interfaces:msg/SpeechType.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "charmie_interfaces/msg/detail/speech_type__rosidl_typesupport_introspection_c.h"
#include "charmie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "charmie_interfaces/msg/detail/speech_type__functions.h"
#include "charmie_interfaces/msg/detail/speech_type__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void SpeechType__rosidl_typesupport_introspection_c__SpeechType_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  charmie_interfaces__msg__SpeechType__init(message_memory);
}

void SpeechType__rosidl_typesupport_introspection_c__SpeechType_fini_function(void * message_memory)
{
  charmie_interfaces__msg__SpeechType__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SpeechType__rosidl_typesupport_introspection_c__SpeechType_message_member_array[4] = {
  {
    "yes_or_no",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__SpeechType, yes_or_no),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "receptionist",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__SpeechType, receptionist),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gpsr",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__SpeechType, gpsr),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "restaurant",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__SpeechType, restaurant),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SpeechType__rosidl_typesupport_introspection_c__SpeechType_message_members = {
  "charmie_interfaces__msg",  // message namespace
  "SpeechType",  // message name
  4,  // number of fields
  sizeof(charmie_interfaces__msg__SpeechType),
  SpeechType__rosidl_typesupport_introspection_c__SpeechType_message_member_array,  // message members
  SpeechType__rosidl_typesupport_introspection_c__SpeechType_init_function,  // function to initialize message memory (memory has to be allocated)
  SpeechType__rosidl_typesupport_introspection_c__SpeechType_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SpeechType__rosidl_typesupport_introspection_c__SpeechType_message_type_support_handle = {
  0,
  &SpeechType__rosidl_typesupport_introspection_c__SpeechType_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, charmie_interfaces, msg, SpeechType)() {
  if (!SpeechType__rosidl_typesupport_introspection_c__SpeechType_message_type_support_handle.typesupport_identifier) {
    SpeechType__rosidl_typesupport_introspection_c__SpeechType_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SpeechType__rosidl_typesupport_introspection_c__SpeechType_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
