// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/MultiObjects.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__MULTI_OBJECTS__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__MULTI_OBJECTS__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/multi_objects__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_MultiObjects_confidence
{
public:
  explicit Init_MultiObjects_confidence(::charmie_interfaces::msg::MultiObjects & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::MultiObjects confidence(::charmie_interfaces::msg::MultiObjects::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::MultiObjects msg_;
};

class Init_MultiObjects_objects
{
public:
  Init_MultiObjects_objects()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiObjects_confidence objects(::charmie_interfaces::msg::MultiObjects::_objects_type arg)
  {
    msg_.objects = std::move(arg);
    return Init_MultiObjects_confidence(msg_);
  }

private:
  ::charmie_interfaces::msg::MultiObjects msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::MultiObjects>()
{
  return charmie_interfaces::msg::builder::Init_MultiObjects_objects();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__MULTI_OBJECTS__BUILDER_HPP_
