// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from charmie_interfaces:msg/ExampleTR.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__EXAMPLE_TR__TRAITS_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__EXAMPLE_TR__TRAITS_HPP_

#include "charmie_interfaces/msg/detail/example_tr__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'coordinates'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<charmie_interfaces::msg::ExampleTR>()
{
  return "charmie_interfaces::msg::ExampleTR";
}

template<>
inline const char * name<charmie_interfaces::msg::ExampleTR>()
{
  return "charmie_interfaces/msg/ExampleTR";
}

template<>
struct has_fixed_size<charmie_interfaces::msg::ExampleTR>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<charmie_interfaces::msg::ExampleTR>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<charmie_interfaces::msg::ExampleTR>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__EXAMPLE_TR__TRAITS_HPP_
