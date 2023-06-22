// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/ObstacleInfo.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLE_INFO__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLE_INFO__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/obstacle_info__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_ObstacleInfo_length_degrees
{
public:
  explicit Init_ObstacleInfo_length_degrees(::charmie_interfaces::msg::ObstacleInfo & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::ObstacleInfo length_degrees(::charmie_interfaces::msg::ObstacleInfo::_length_degrees_type arg)
  {
    msg_.length_degrees = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::ObstacleInfo msg_;
};

class Init_ObstacleInfo_length_cm
{
public:
  explicit Init_ObstacleInfo_length_cm(::charmie_interfaces::msg::ObstacleInfo & msg)
  : msg_(msg)
  {}
  Init_ObstacleInfo_length_degrees length_cm(::charmie_interfaces::msg::ObstacleInfo::_length_cm_type arg)
  {
    msg_.length_cm = std::move(arg);
    return Init_ObstacleInfo_length_degrees(msg_);
  }

private:
  ::charmie_interfaces::msg::ObstacleInfo msg_;
};

class Init_ObstacleInfo_dist
{
public:
  explicit Init_ObstacleInfo_dist(::charmie_interfaces::msg::ObstacleInfo & msg)
  : msg_(msg)
  {}
  Init_ObstacleInfo_length_cm dist(::charmie_interfaces::msg::ObstacleInfo::_dist_type arg)
  {
    msg_.dist = std::move(arg);
    return Init_ObstacleInfo_length_cm(msg_);
  }

private:
  ::charmie_interfaces::msg::ObstacleInfo msg_;
};

class Init_ObstacleInfo_alfa
{
public:
  Init_ObstacleInfo_alfa()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ObstacleInfo_dist alfa(::charmie_interfaces::msg::ObstacleInfo::_alfa_type arg)
  {
    msg_.alfa = std::move(arg);
    return Init_ObstacleInfo_dist(msg_);
  }

private:
  ::charmie_interfaces::msg::ObstacleInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::ObstacleInfo>()
{
  return charmie_interfaces::msg::builder::Init_ObstacleInfo_alfa();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLE_INFO__BUILDER_HPP_
