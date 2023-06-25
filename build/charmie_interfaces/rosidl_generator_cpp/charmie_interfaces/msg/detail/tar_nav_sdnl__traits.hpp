// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from charmie_interfaces:msg/TarNavSDNL.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__TRAITS_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__TRAITS_HPP_

#include "charmie_interfaces/msg/detail/tar_nav_sdnl__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'move_target_coordinates'
// Member 'rotate_target_coordinates'
#include "geometry_msgs/msg/detail/pose2_d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<charmie_interfaces::msg::TarNavSDNL>()
{
  return "charmie_interfaces::msg::TarNavSDNL";
}

template<>
inline const char * name<charmie_interfaces::msg::TarNavSDNL>()
{
  return "charmie_interfaces/msg/TarNavSDNL";
}

template<>
struct has_fixed_size<charmie_interfaces::msg::TarNavSDNL>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct has_bounded_size<charmie_interfaces::msg::TarNavSDNL>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct is_message<charmie_interfaces::msg::TarNavSDNL>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__TRAITS_HPP_
