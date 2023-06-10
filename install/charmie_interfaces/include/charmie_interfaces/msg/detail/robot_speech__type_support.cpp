// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/RobotSpeech.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "charmie_interfaces/msg/detail/robot_speech__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace charmie_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RobotSpeech_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) charmie_interfaces::msg::RobotSpeech(_init);
}

void RobotSpeech_fini_function(void * message_memory)
{
  auto typed_message = static_cast<charmie_interfaces::msg::RobotSpeech *>(message_memory);
  typed_message->~RobotSpeech();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RobotSpeech_message_member_array[2] = {
  {
    "command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::RobotSpeech, command),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "language",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::RobotSpeech, language),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RobotSpeech_message_members = {
  "charmie_interfaces::msg",  // message namespace
  "RobotSpeech",  // message name
  2,  // number of fields
  sizeof(charmie_interfaces::msg::RobotSpeech),
  RobotSpeech_message_member_array,  // message members
  RobotSpeech_init_function,  // function to initialize message memory (memory has to be allocated)
  RobotSpeech_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RobotSpeech_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RobotSpeech_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace charmie_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<charmie_interfaces::msg::RobotSpeech>()
{
  return &::charmie_interfaces::msg::rosidl_typesupport_introspection_cpp::RobotSpeech_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, charmie_interfaces, msg, RobotSpeech)() {
  return &::charmie_interfaces::msg::rosidl_typesupport_introspection_cpp::RobotSpeech_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
