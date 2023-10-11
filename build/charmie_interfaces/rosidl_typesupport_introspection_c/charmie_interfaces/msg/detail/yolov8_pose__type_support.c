// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from charmie_interfaces:msg/Yolov8Pose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "charmie_interfaces/msg/detail/yolov8_pose__rosidl_typesupport_introspection_c.h"
#include "charmie_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "charmie_interfaces/msg/detail/yolov8_pose__functions.h"
#include "charmie_interfaces/msg/detail/yolov8_pose__struct.h"


// Include directives for member types
// Member `persons`
#include "charmie_interfaces/msg/detected_person.h"
// Member `persons`
#include "charmie_interfaces/msg/detail/detected_person__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  charmie_interfaces__msg__Yolov8Pose__init(message_memory);
}

void Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_fini_function(void * message_memory)
{
  charmie_interfaces__msg__Yolov8Pose__fini(message_memory);
}

size_t Yolov8Pose__rosidl_typesupport_introspection_c__size_function__DetectedPerson__persons(
  const void * untyped_member)
{
  const charmie_interfaces__msg__DetectedPerson__Sequence * member =
    (const charmie_interfaces__msg__DetectedPerson__Sequence *)(untyped_member);
  return member->size;
}

const void * Yolov8Pose__rosidl_typesupport_introspection_c__get_const_function__DetectedPerson__persons(
  const void * untyped_member, size_t index)
{
  const charmie_interfaces__msg__DetectedPerson__Sequence * member =
    (const charmie_interfaces__msg__DetectedPerson__Sequence *)(untyped_member);
  return &member->data[index];
}

void * Yolov8Pose__rosidl_typesupport_introspection_c__get_function__DetectedPerson__persons(
  void * untyped_member, size_t index)
{
  charmie_interfaces__msg__DetectedPerson__Sequence * member =
    (charmie_interfaces__msg__DetectedPerson__Sequence *)(untyped_member);
  return &member->data[index];
}

bool Yolov8Pose__rosidl_typesupport_introspection_c__resize_function__DetectedPerson__persons(
  void * untyped_member, size_t size)
{
  charmie_interfaces__msg__DetectedPerson__Sequence * member =
    (charmie_interfaces__msg__DetectedPerson__Sequence *)(untyped_member);
  charmie_interfaces__msg__DetectedPerson__Sequence__fini(member);
  return charmie_interfaces__msg__DetectedPerson__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_message_member_array[2] = {
  {
    "num_person",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__Yolov8Pose, num_person),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "persons",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces__msg__Yolov8Pose, persons),  // bytes offset in struct
    NULL,  // default value
    Yolov8Pose__rosidl_typesupport_introspection_c__size_function__DetectedPerson__persons,  // size() function pointer
    Yolov8Pose__rosidl_typesupport_introspection_c__get_const_function__DetectedPerson__persons,  // get_const(index) function pointer
    Yolov8Pose__rosidl_typesupport_introspection_c__get_function__DetectedPerson__persons,  // get(index) function pointer
    Yolov8Pose__rosidl_typesupport_introspection_c__resize_function__DetectedPerson__persons  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_message_members = {
  "charmie_interfaces__msg",  // message namespace
  "Yolov8Pose",  // message name
  2,  // number of fields
  sizeof(charmie_interfaces__msg__Yolov8Pose),
  Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_message_member_array,  // message members
  Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_init_function,  // function to initialize message memory (memory has to be allocated)
  Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_message_type_support_handle = {
  0,
  &Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_charmie_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, charmie_interfaces, msg, Yolov8Pose)() {
  Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, charmie_interfaces, msg, DetectedPerson)();
  if (!Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_message_type_support_handle.typesupport_identifier) {
    Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Yolov8Pose__rosidl_typesupport_introspection_c__Yolov8Pose_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
