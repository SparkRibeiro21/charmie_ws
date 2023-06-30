// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from charmie_interfaces:msg/Yolov8Pose.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__TRAITS_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__TRAITS_HPP_

#include "charmie_interfaces/msg/detail/yolov8_pose__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<charmie_interfaces::msg::Yolov8Pose>()
{
  return "charmie_interfaces::msg::Yolov8Pose";
}

template<>
inline const char * name<charmie_interfaces::msg::Yolov8Pose>()
{
  return "charmie_interfaces/msg/Yolov8Pose";
}

template<>
struct has_fixed_size<charmie_interfaces::msg::Yolov8Pose>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<charmie_interfaces::msg::Yolov8Pose>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<charmie_interfaces::msg::Yolov8Pose>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__TRAITS_HPP_
