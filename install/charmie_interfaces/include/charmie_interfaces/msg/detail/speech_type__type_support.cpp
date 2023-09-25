// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/SpeechType.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "charmie_interfaces/msg/detail/speech_type__struct.hpp"
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

void SpeechType_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) charmie_interfaces::msg::SpeechType(_init);
}

void SpeechType_fini_function(void * message_memory)
{
  auto typed_message = static_cast<charmie_interfaces::msg::SpeechType *>(message_memory);
  typed_message->~SpeechType();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SpeechType_message_member_array[4] = {
  {
    "yes_or_no",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::SpeechType, yes_or_no),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "receptionist",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::SpeechType, receptionist),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gpsr",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::SpeechType, gpsr),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "restaurant",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::SpeechType, restaurant),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SpeechType_message_members = {
  "charmie_interfaces::msg",  // message namespace
  "SpeechType",  // message name
  4,  // number of fields
  sizeof(charmie_interfaces::msg::SpeechType),
  SpeechType_message_member_array,  // message members
  SpeechType_init_function,  // function to initialize message memory (memory has to be allocated)
  SpeechType_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SpeechType_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SpeechType_message_members,
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
get_message_type_support_handle<charmie_interfaces::msg::SpeechType>()
{
  return &::charmie_interfaces::msg::rosidl_typesupport_introspection_cpp::SpeechType_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, charmie_interfaces, msg, SpeechType)() {
  return &::charmie_interfaces::msg::rosidl_typesupport_introspection_cpp::SpeechType_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
