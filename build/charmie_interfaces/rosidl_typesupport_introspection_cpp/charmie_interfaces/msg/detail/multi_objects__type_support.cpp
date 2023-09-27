// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/MultiObjects.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "charmie_interfaces/msg/detail/multi_objects__struct.hpp"
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

void MultiObjects_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) charmie_interfaces::msg::MultiObjects(_init);
}

void MultiObjects_fini_function(void * message_memory)
{
  auto typed_message = static_cast<charmie_interfaces::msg::MultiObjects *>(message_memory);
  typed_message->~MultiObjects();
}

size_t size_function__MultiObjects__objects(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiObjects__objects(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiObjects__objects(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__MultiObjects__objects(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MultiObjects__confidence(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiObjects__confidence(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiObjects__confidence(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MultiObjects__confidence(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MultiObjects__distance(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiObjects__distance(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiObjects__distance(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MultiObjects__distance(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__MultiObjects__position(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__MultiObjects__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__MultiObjects__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__MultiObjects__position(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MultiObjects_message_member_array[4] = {
  {
    "objects",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::MultiObjects, objects),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiObjects__objects,  // size() function pointer
    get_const_function__MultiObjects__objects,  // get_const(index) function pointer
    get_function__MultiObjects__objects,  // get(index) function pointer
    resize_function__MultiObjects__objects  // resize(index) function pointer
  },
  {
    "confidence",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::MultiObjects, confidence),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiObjects__confidence,  // size() function pointer
    get_const_function__MultiObjects__confidence,  // get_const(index) function pointer
    get_function__MultiObjects__confidence,  // get(index) function pointer
    resize_function__MultiObjects__confidence  // resize(index) function pointer
  },
  {
    "distance",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::MultiObjects, distance),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiObjects__distance,  // size() function pointer
    get_const_function__MultiObjects__distance,  // get_const(index) function pointer
    get_function__MultiObjects__distance,  // get(index) function pointer
    resize_function__MultiObjects__distance  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::MultiObjects, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__MultiObjects__position,  // size() function pointer
    get_const_function__MultiObjects__position,  // get_const(index) function pointer
    get_function__MultiObjects__position,  // get(index) function pointer
    resize_function__MultiObjects__position  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MultiObjects_message_members = {
  "charmie_interfaces::msg",  // message namespace
  "MultiObjects",  // message name
  4,  // number of fields
  sizeof(charmie_interfaces::msg::MultiObjects),
  MultiObjects_message_member_array,  // message members
  MultiObjects_init_function,  // function to initialize message memory (memory has to be allocated)
  MultiObjects_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MultiObjects_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MultiObjects_message_members,
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
get_message_type_support_handle<charmie_interfaces::msg::MultiObjects>()
{
  return &::charmie_interfaces::msg::rosidl_typesupport_introspection_cpp::MultiObjects_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, charmie_interfaces, msg, MultiObjects)() {
  return &::charmie_interfaces::msg::rosidl_typesupport_introspection_cpp::MultiObjects_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
