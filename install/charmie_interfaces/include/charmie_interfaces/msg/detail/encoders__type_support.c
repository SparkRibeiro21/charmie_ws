// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from charmie_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "charmie_interfaces/msg/detail/encoders__rosidl_typesupport_introspection_c.h"
#include "charmie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "charmie_interfaces/msg/detail/encoders__functions.h"
#include "charmie_interfaces/msg/detail/encoders__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void Encoders__rosidl_typesupport_introspection_c__Encoders_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  charmie_interfaces__msg__Encoders__init(message_memory);
}

void Encoders__rosidl_typesupport_introspection_c__Encoders_fini_function(void * message_memory)
{
  charmie_interfaces__msg__Encoders__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Encoders__rosidl_typesupport_introspection_c__Encoders_message_member_array[4] = {
  {
    "enc_m1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__Encoders, enc_m1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enc_m2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__Encoders, enc_m2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enc_m3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__Encoders, enc_m3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "enc_m4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__Encoders, enc_m4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Encoders__rosidl_typesupport_introspection_c__Encoders_message_members = {
  "charmie_interfaces__msg",  // message namespace
  "Encoders",  // message name
  4,  // number of fields
  sizeof(charmie_interfaces__msg__Encoders),
  Encoders__rosidl_typesupport_introspection_c__Encoders_message_member_array,  // message members
  Encoders__rosidl_typesupport_introspection_c__Encoders_init_function,  // function to initialize message memory (memory has to be allocated)
  Encoders__rosidl_typesupport_introspection_c__Encoders_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Encoders__rosidl_typesupport_introspection_c__Encoders_message_type_support_handle = {
  0,
  &Encoders__rosidl_typesupport_introspection_c__Encoders_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, charmie_interfaces, msg, Encoders)() {
  if (!Encoders__rosidl_typesupport_introspection_c__Encoders_message_type_support_handle.typesupport_identifier) {
    Encoders__rosidl_typesupport_introspection_c__Encoders_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Encoders__rosidl_typesupport_introspection_c__Encoders_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
