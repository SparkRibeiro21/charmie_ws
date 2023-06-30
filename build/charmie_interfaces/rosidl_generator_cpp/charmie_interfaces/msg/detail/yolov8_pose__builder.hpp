// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/Yolov8Pose.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/yolov8_pose__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_Yolov8Pose_keypoints
{
public:
  explicit Init_Yolov8Pose_keypoints(::charmie_interfaces::msg::Yolov8Pose & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::Yolov8Pose keypoints(::charmie_interfaces::msg::Yolov8Pose::_keypoints_type arg)
  {
    msg_.keypoints = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::Yolov8Pose msg_;
};

class Init_Yolov8Pose_num_person
{
public:
  Init_Yolov8Pose_num_person()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Yolov8Pose_keypoints num_person(::charmie_interfaces::msg::Yolov8Pose::_num_person_type arg)
  {
    msg_.num_person = std::move(arg);
    return Init_Yolov8Pose_keypoints(msg_);
  }

private:
  ::charmie_interfaces::msg::Yolov8Pose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::Yolov8Pose>()
{
  return charmie_interfaces::msg::builder::Init_Yolov8Pose_num_person();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__BUILDER_HPP_
