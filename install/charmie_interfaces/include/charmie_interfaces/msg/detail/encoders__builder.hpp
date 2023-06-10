// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__ENCODERS__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__ENCODERS__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/encoders__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_Encoders_enc_m4
{
public:
  explicit Init_Encoders_enc_m4(::charmie_interfaces::msg::Encoders & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::Encoders enc_m4(::charmie_interfaces::msg::Encoders::_enc_m4_type arg)
  {
    msg_.enc_m4 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::Encoders msg_;
};

class Init_Encoders_enc_m3
{
public:
  explicit Init_Encoders_enc_m3(::charmie_interfaces::msg::Encoders & msg)
  : msg_(msg)
  {}
  Init_Encoders_enc_m4 enc_m3(::charmie_interfaces::msg::Encoders::_enc_m3_type arg)
  {
    msg_.enc_m3 = std::move(arg);
    return Init_Encoders_enc_m4(msg_);
  }

private:
  ::charmie_interfaces::msg::Encoders msg_;
};

class Init_Encoders_enc_m2
{
public:
  explicit Init_Encoders_enc_m2(::charmie_interfaces::msg::Encoders & msg)
  : msg_(msg)
  {}
  Init_Encoders_enc_m3 enc_m2(::charmie_interfaces::msg::Encoders::_enc_m2_type arg)
  {
    msg_.enc_m2 = std::move(arg);
    return Init_Encoders_enc_m3(msg_);
  }

private:
  ::charmie_interfaces::msg::Encoders msg_;
};

class Init_Encoders_enc_m1
{
public:
  Init_Encoders_enc_m1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Encoders_enc_m2 enc_m1(::charmie_interfaces::msg::Encoders::_enc_m1_type arg)
  {
    msg_.enc_m1 = std::move(arg);
    return Init_Encoders_enc_m2(msg_);
  }

private:
  ::charmie_interfaces::msg::Encoders msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::Encoders>()
{
  return charmie_interfaces::msg::builder::Init_Encoders_enc_m1();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__ENCODERS__BUILDER_HPP_
