// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/TarNavSDNL.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/tar_nav_sdnl__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_TarNavSDNL_flag_not_obs
{
public:
  explicit Init_TarNavSDNL_flag_not_obs(::charmie_interfaces::msg::TarNavSDNL & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::TarNavSDNL flag_not_obs(::charmie_interfaces::msg::TarNavSDNL::_flag_not_obs_type arg)
  {
    msg_.flag_not_obs = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::TarNavSDNL msg_;
};

class Init_TarNavSDNL_rotate_target_coordinates
{
public:
  explicit Init_TarNavSDNL_rotate_target_coordinates(::charmie_interfaces::msg::TarNavSDNL & msg)
  : msg_(msg)
  {}
  Init_TarNavSDNL_flag_not_obs rotate_target_coordinates(::charmie_interfaces::msg::TarNavSDNL::_rotate_target_coordinates_type arg)
  {
    msg_.rotate_target_coordinates = std::move(arg);
    return Init_TarNavSDNL_flag_not_obs(msg_);
  }

private:
  ::charmie_interfaces::msg::TarNavSDNL msg_;
};

class Init_TarNavSDNL_move_target_coordinates
{
public:
  Init_TarNavSDNL_move_target_coordinates()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TarNavSDNL_rotate_target_coordinates move_target_coordinates(::charmie_interfaces::msg::TarNavSDNL::_move_target_coordinates_type arg)
  {
    msg_.move_target_coordinates = std::move(arg);
    return Init_TarNavSDNL_rotate_target_coordinates(msg_);
  }

private:
  ::charmie_interfaces::msg::TarNavSDNL msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::TarNavSDNL>()
{
  return charmie_interfaces::msg::builder::Init_TarNavSDNL_move_target_coordinates();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__BUILDER_HPP_
