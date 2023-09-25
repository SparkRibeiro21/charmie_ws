// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from charmie_interfaces:msg/Yolov8PoseArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "charmie_interfaces/msg/detail/yolov8_pose_array__struct.hpp"
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

void Yolov8PoseArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) charmie_interfaces::msg::Yolov8PoseArray(_init);
}

void Yolov8PoseArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<charmie_interfaces::msg::Yolov8PoseArray *>(message_memory);
  typed_message->~Yolov8PoseArray();
}

size_t size_function__Yolov8PoseArray__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<charmie_interfaces::msg::Yolov8Pose> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Yolov8PoseArray__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<charmie_interfaces::msg::Yolov8Pose> *>(untyped_member);
  return &member[index];
}

void * get_function__Yolov8PoseArray__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<charmie_interfaces::msg::Yolov8Pose> *>(untyped_member);
  return &member[index];
}

void resize_function__Yolov8PoseArray__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<charmie_interfaces::msg::Yolov8Pose> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Yolov8PoseArray_message_member_array[1] = {
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<charmie_interfaces::msg::Yolov8Pose>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(charmie_interfaces::msg::Yolov8PoseArray, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__Yolov8PoseArray__data,  // size() function pointer
    get_const_function__Yolov8PoseArray__data,  // get_const(index) function pointer
    get_function__Yolov8PoseArray__data,  // get(index) function pointer
    resize_function__Yolov8PoseArray__data  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Yolov8PoseArray_message_members = {
  "charmie_interfaces::msg",  // message namespace
  "Yolov8PoseArray",  // message name
  1,  // number of fields
  sizeof(charmie_interfaces::msg::Yolov8PoseArray),
  Yolov8PoseArray_message_member_array,  // message members
  Yolov8PoseArray_init_function,  // function to initialize message memory (memory has to be allocated)
  Yolov8PoseArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Yolov8PoseArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Yolov8PoseArray_message_members,
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
get_message_type_support_handle<charmie_interfaces::msg::Yolov8PoseArray>()
{
  return &::charmie_interfaces::msg::rosidl_typesupport_introspection_cpp::Yolov8PoseArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, charmie_interfaces, msg, Yolov8PoseArray)() {
  return &::charmie_interfaces::msg::rosidl_typesupport_introspection_cpp::Yolov8PoseArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
