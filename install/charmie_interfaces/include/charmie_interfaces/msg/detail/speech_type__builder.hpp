// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/SpeechType.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__SPEECH_TYPE__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__SPEECH_TYPE__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/speech_type__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_SpeechType_restaurant
{
public:
  explicit Init_SpeechType_restaurant(::charmie_interfaces::msg::SpeechType & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::SpeechType restaurant(::charmie_interfaces::msg::SpeechType::_restaurant_type arg)
  {
    msg_.restaurant = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::SpeechType msg_;
};

class Init_SpeechType_gpsr
{
public:
  explicit Init_SpeechType_gpsr(::charmie_interfaces::msg::SpeechType & msg)
  : msg_(msg)
  {}
  Init_SpeechType_restaurant gpsr(::charmie_interfaces::msg::SpeechType::_gpsr_type arg)
  {
    msg_.gpsr = std::move(arg);
    return Init_SpeechType_restaurant(msg_);
  }

private:
  ::charmie_interfaces::msg::SpeechType msg_;
};

class Init_SpeechType_receptionist
{
public:
  explicit Init_SpeechType_receptionist(::charmie_interfaces::msg::SpeechType & msg)
  : msg_(msg)
  {}
  Init_SpeechType_gpsr receptionist(::charmie_interfaces::msg::SpeechType::_receptionist_type arg)
  {
    msg_.receptionist = std::move(arg);
    return Init_SpeechType_gpsr(msg_);
  }

private:
  ::charmie_interfaces::msg::SpeechType msg_;
};

class Init_SpeechType_yes_or_no
{
public:
  Init_SpeechType_yes_or_no()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SpeechType_receptionist yes_or_no(::charmie_interfaces::msg::SpeechType::_yes_or_no_type arg)
  {
    msg_.yes_or_no = std::move(arg);
    return Init_SpeechType_receptionist(msg_);
  }

private:
  ::charmie_interfaces::msg::SpeechType msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::SpeechType>()
{
  return charmie_interfaces::msg::builder::Init_SpeechType_yes_or_no();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__SPEECH_TYPE__BUILDER_HPP_
