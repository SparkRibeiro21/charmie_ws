// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from charmie_interfaces:msg/ExampleTR.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "charmie_interfaces/msg/detail/example_tr__rosidl_typesupport_introspection_c.h"
#include "charmie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "charmie_interfaces/msg/detail/example_tr__functions.h"
#include "charmie_interfaces/msg/detail/example_tr__struct.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `coordinates`
#include "geometry_msgs/msg/point.h"
// Member `coordinates`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  charmie_interfaces__msg__ExampleTR__init(message_memory);
}

void ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_fini_function(void * message_memory)
{
  charmie_interfaces__msg__ExampleTR__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_message_member_array[2] = {
  {
    "name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__ExampleTR, name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "coordinates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__ExampleTR, coordinates),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_message_members = {
  "charmie_interfaces__msg",  // message namespace
  "ExampleTR",  // message name
  2,  // number of fields
  sizeof(charmie_interfaces__msg__ExampleTR),
  ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_message_member_array,  // message members
  ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_init_function,  // function to initialize message memory (memory has to be allocated)
  ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_message_type_support_handle = {
  0,
  &ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, charmie_interfaces, msg, ExampleTR)() {
  ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_message_type_support_handle.typesupport_identifier) {
    ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ExampleTR__rosidl_typesupport_introspection_c__ExampleTR_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
