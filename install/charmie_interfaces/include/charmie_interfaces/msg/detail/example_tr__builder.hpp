// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/ExampleTR.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__EXAMPLE_TR__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__EXAMPLE_TR__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/example_tr__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_ExampleTR_coordinates
{
public:
  explicit Init_ExampleTR_coordinates(::charmie_interfaces::msg::ExampleTR & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::ExampleTR coordinates(::charmie_interfaces::msg::ExampleTR::_coordinates_type arg)
  {
    msg_.coordinates = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::ExampleTR msg_;
};

class Init_ExampleTR_name
{
public:
  Init_ExampleTR_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExampleTR_coordinates name(::charmie_interfaces::msg::ExampleTR::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_ExampleTR_coordinates(msg_);
  }

private:
  ::charmie_interfaces::msg::ExampleTR msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::ExampleTR>()
{
  return charmie_interfaces::msg::builder::Init_ExampleTR_name();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__EXAMPLE_TR__BUILDER_HPP_
