// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from charmie_interfaces:msg/MultiObjects.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "charmie_interfaces/msg/detail/multi_objects__rosidl_typesupport_introspection_c.h"
#include "charmie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "charmie_interfaces/msg/detail/multi_objects__functions.h"
#include "charmie_interfaces/msg/detail/multi_objects__struct.h"


// Include directives for member types
// Member `objects`
#include "rosidl_runtime_c/string_functions.h"
// Member `confidence`
// Member `distance`
// Member `position`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  charmie_interfaces__msg__MultiObjects__init(message_memory);
}

void MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_fini_function(void * message_memory)
{
  charmie_interfaces__msg__MultiObjects__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_message_member_array[4] = {
  {
    "objects",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__MultiObjects, objects),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "confidence",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__MultiObjects, confidence),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__MultiObjects, distance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__MultiObjects, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_message_members = {
  "charmie_interfaces__msg",  // message namespace
  "MultiObjects",  // message name
  4,  // number of fields
  sizeof(charmie_interfaces__msg__MultiObjects),
  MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_message_member_array,  // message members
  MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_init_function,  // function to initialize message memory (memory has to be allocated)
  MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_message_type_support_handle = {
  0,
  &MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, charmie_interfaces, msg, MultiObjects)() {
  if (!MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_message_type_support_handle.typesupport_identifier) {
    MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &MultiObjects__rosidl_typesupport_introspection_c__MultiObjects_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
