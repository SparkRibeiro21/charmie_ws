// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/RobotSpeech.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__ROBOT_SPEECH__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__ROBOT_SPEECH__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/robot_speech__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotSpeech_language
{
public:
  explicit Init_RobotSpeech_language(::charmie_interfaces::msg::RobotSpeech & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::RobotSpeech language(::charmie_interfaces::msg::RobotSpeech::_language_type arg)
  {
    msg_.language = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::RobotSpeech msg_;
};

class Init_RobotSpeech_command
{
public:
  Init_RobotSpeech_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotSpeech_language command(::charmie_interfaces::msg::RobotSpeech::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_RobotSpeech_language(msg_);
  }

private:
  ::charmie_interfaces::msg::RobotSpeech msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::RobotSpeech>()
{
  return charmie_interfaces::msg::builder::Init_RobotSpeech_command();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__ROBOT_SPEECH__BUILDER_HPP_
