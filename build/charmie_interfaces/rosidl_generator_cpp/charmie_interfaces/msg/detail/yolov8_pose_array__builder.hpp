// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/Yolov8PoseArray.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE_ARRAY__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE_ARRAY__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/yolov8_pose_array__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_Yolov8PoseArray_data
{
public:
  Init_Yolov8PoseArray_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::charmie_interfaces::msg::Yolov8PoseArray data(::charmie_interfaces::msg::Yolov8PoseArray::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::Yolov8PoseArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::Yolov8PoseArray>()
{
  return charmie_interfaces::msg::builder::Init_Yolov8PoseArray_data();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE_ARRAY__BUILDER_HPP_
