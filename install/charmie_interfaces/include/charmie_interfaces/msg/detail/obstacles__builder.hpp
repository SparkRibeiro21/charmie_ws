// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/Obstacles.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLES__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLES__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/obstacles__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_Obstacles_obstacles
{
public:
  explicit Init_Obstacles_obstacles(::charmie_interfaces::msg::Obstacles & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::Obstacles obstacles(::charmie_interfaces::msg::Obstacles::_obstacles_type arg)
  {
    msg_.obstacles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::Obstacles msg_;
};

class Init_Obstacles_no_obstacles
{
public:
  Init_Obstacles_no_obstacles()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Obstacles_obstacles no_obstacles(::charmie_interfaces::msg::Obstacles::_no_obstacles_type arg)
  {
    msg_.no_obstacles = std::move(arg);
    return Init_Obstacles_obstacles(msg_);
  }

private:
  ::charmie_interfaces::msg::Obstacles msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::Obstacles>()
{
  return charmie_interfaces::msg::builder::Init_Obstacles_no_obstacles();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLES__BUILDER_HPP_
