// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from charmie_interfaces:msg/TarNavSDNL.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "charmie_interfaces/msg/detail/tar_nav_sdnl__rosidl_typesupport_introspection_c.h"
#include "charmie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "charmie_interfaces/msg/detail/tar_nav_sdnl__functions.h"
#include "charmie_interfaces/msg/detail/tar_nav_sdnl__struct.h"


// Include directives for member types
// Member `move_target_coordinates`
// Member `rotate_target_coordinates`
#include "geometry_msgs/msg/pose2_d.h"
// Member `move_target_coordinates`
// Member `rotate_target_coordinates`
#include "geometry_msgs/msg/detail/pose2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  charmie_interfaces__msg__TarNavSDNL__init(message_memory);
}

void TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_fini_function(void * message_memory)
{
  charmie_interfaces__msg__TarNavSDNL__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_member_array[3] = {
  {
    "move_target_coordinates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__TarNavSDNL, move_target_coordinates),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rotate_target_coordinates",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__TarNavSDNL, rotate_target_coordinates),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "flag_not_obs",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__TarNavSDNL, flag_not_obs),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_members = {
  "charmie_interfaces__msg",  // message namespace
  "TarNavSDNL",  // message name
  3,  // number of fields
  sizeof(charmie_interfaces__msg__TarNavSDNL),
  TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_member_array,  // message members
  TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_init_function,  // function to initialize message memory (memory has to be allocated)
  TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_type_support_handle = {
  0,
  &TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, charmie_interfaces, msg, TarNavSDNL)() {
  TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose2D)();
  TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose2D)();
  if (!TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_type_support_handle.typesupport_identifier) {
    TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TarNavSDNL__rosidl_typesupport_introspection_c__TarNavSDNL_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
