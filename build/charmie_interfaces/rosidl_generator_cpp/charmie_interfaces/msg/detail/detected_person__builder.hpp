// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/DetectedPerson.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__DETECTED_PERSON__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__DETECTED_PERSON__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/detected_person__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_DetectedPerson_kp_ankle_right_conf
{
public:
  explicit Init_DetectedPerson_kp_ankle_right_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::DetectedPerson kp_ankle_right_conf(::charmie_interfaces::msg::DetectedPerson::_kp_ankle_right_conf_type arg)
  {
    msg_.kp_ankle_right_conf = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ankle_right_y
{
public:
  explicit Init_DetectedPerson_kp_ankle_right_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ankle_right_conf kp_ankle_right_y(::charmie_interfaces::msg::DetectedPerson::_kp_ankle_right_y_type arg)
  {
    msg_.kp_ankle_right_y = std::move(arg);
    return Init_DetectedPerson_kp_ankle_right_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ankle_right_x
{
public:
  explicit Init_DetectedPerson_kp_ankle_right_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ankle_right_y kp_ankle_right_x(::charmie_interfaces::msg::DetectedPerson::_kp_ankle_right_x_type arg)
  {
    msg_.kp_ankle_right_x = std::move(arg);
    return Init_DetectedPerson_kp_ankle_right_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ankle_left_conf
{
public:
  explicit Init_DetectedPerson_kp_ankle_left_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ankle_right_x kp_ankle_left_conf(::charmie_interfaces::msg::DetectedPerson::_kp_ankle_left_conf_type arg)
  {
    msg_.kp_ankle_left_conf = std::move(arg);
    return Init_DetectedPerson_kp_ankle_right_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ankle_left_y
{
public:
  explicit Init_DetectedPerson_kp_ankle_left_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ankle_left_conf kp_ankle_left_y(::charmie_interfaces::msg::DetectedPerson::_kp_ankle_left_y_type arg)
  {
    msg_.kp_ankle_left_y = std::move(arg);
    return Init_DetectedPerson_kp_ankle_left_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ankle_left_x
{
public:
  explicit Init_DetectedPerson_kp_ankle_left_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ankle_left_y kp_ankle_left_x(::charmie_interfaces::msg::DetectedPerson::_kp_ankle_left_x_type arg)
  {
    msg_.kp_ankle_left_x = std::move(arg);
    return Init_DetectedPerson_kp_ankle_left_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_knee_right_conf
{
public:
  explicit Init_DetectedPerson_kp_knee_right_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ankle_left_x kp_knee_right_conf(::charmie_interfaces::msg::DetectedPerson::_kp_knee_right_conf_type arg)
  {
    msg_.kp_knee_right_conf = std::move(arg);
    return Init_DetectedPerson_kp_ankle_left_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_knee_right_y
{
public:
  explicit Init_DetectedPerson_kp_knee_right_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_knee_right_conf kp_knee_right_y(::charmie_interfaces::msg::DetectedPerson::_kp_knee_right_y_type arg)
  {
    msg_.kp_knee_right_y = std::move(arg);
    return Init_DetectedPerson_kp_knee_right_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_knee_right_x
{
public:
  explicit Init_DetectedPerson_kp_knee_right_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_knee_right_y kp_knee_right_x(::charmie_interfaces::msg::DetectedPerson::_kp_knee_right_x_type arg)
  {
    msg_.kp_knee_right_x = std::move(arg);
    return Init_DetectedPerson_kp_knee_right_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_knee_left_conf
{
public:
  explicit Init_DetectedPerson_kp_knee_left_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_knee_right_x kp_knee_left_conf(::charmie_interfaces::msg::DetectedPerson::_kp_knee_left_conf_type arg)
  {
    msg_.kp_knee_left_conf = std::move(arg);
    return Init_DetectedPerson_kp_knee_right_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_knee_left_y
{
public:
  explicit Init_DetectedPerson_kp_knee_left_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_knee_left_conf kp_knee_left_y(::charmie_interfaces::msg::DetectedPerson::_kp_knee_left_y_type arg)
  {
    msg_.kp_knee_left_y = std::move(arg);
    return Init_DetectedPerson_kp_knee_left_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_knee_left_x
{
public:
  explicit Init_DetectedPerson_kp_knee_left_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_knee_left_y kp_knee_left_x(::charmie_interfaces::msg::DetectedPerson::_kp_knee_left_x_type arg)
  {
    msg_.kp_knee_left_x = std::move(arg);
    return Init_DetectedPerson_kp_knee_left_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_hip_right_conf
{
public:
  explicit Init_DetectedPerson_kp_hip_right_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_knee_left_x kp_hip_right_conf(::charmie_interfaces::msg::DetectedPerson::_kp_hip_right_conf_type arg)
  {
    msg_.kp_hip_right_conf = std::move(arg);
    return Init_DetectedPerson_kp_knee_left_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_hip_right_y
{
public:
  explicit Init_DetectedPerson_kp_hip_right_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_hip_right_conf kp_hip_right_y(::charmie_interfaces::msg::DetectedPerson::_kp_hip_right_y_type arg)
  {
    msg_.kp_hip_right_y = std::move(arg);
    return Init_DetectedPerson_kp_hip_right_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_hip_right_x
{
public:
  explicit Init_DetectedPerson_kp_hip_right_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_hip_right_y kp_hip_right_x(::charmie_interfaces::msg::DetectedPerson::_kp_hip_right_x_type arg)
  {
    msg_.kp_hip_right_x = std::move(arg);
    return Init_DetectedPerson_kp_hip_right_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_hip_left_conf
{
public:
  explicit Init_DetectedPerson_kp_hip_left_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_hip_right_x kp_hip_left_conf(::charmie_interfaces::msg::DetectedPerson::_kp_hip_left_conf_type arg)
  {
    msg_.kp_hip_left_conf = std::move(arg);
    return Init_DetectedPerson_kp_hip_right_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_hip_left_y
{
public:
  explicit Init_DetectedPerson_kp_hip_left_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_hip_left_conf kp_hip_left_y(::charmie_interfaces::msg::DetectedPerson::_kp_hip_left_y_type arg)
  {
    msg_.kp_hip_left_y = std::move(arg);
    return Init_DetectedPerson_kp_hip_left_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_hip_left_x
{
public:
  explicit Init_DetectedPerson_kp_hip_left_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_hip_left_y kp_hip_left_x(::charmie_interfaces::msg::DetectedPerson::_kp_hip_left_x_type arg)
  {
    msg_.kp_hip_left_x = std::move(arg);
    return Init_DetectedPerson_kp_hip_left_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_wrist_right_conf
{
public:
  explicit Init_DetectedPerson_kp_wrist_right_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_hip_left_x kp_wrist_right_conf(::charmie_interfaces::msg::DetectedPerson::_kp_wrist_right_conf_type arg)
  {
    msg_.kp_wrist_right_conf = std::move(arg);
    return Init_DetectedPerson_kp_hip_left_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_wrist_right_y
{
public:
  explicit Init_DetectedPerson_kp_wrist_right_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_wrist_right_conf kp_wrist_right_y(::charmie_interfaces::msg::DetectedPerson::_kp_wrist_right_y_type arg)
  {
    msg_.kp_wrist_right_y = std::move(arg);
    return Init_DetectedPerson_kp_wrist_right_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_wrist_right_x
{
public:
  explicit Init_DetectedPerson_kp_wrist_right_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_wrist_right_y kp_wrist_right_x(::charmie_interfaces::msg::DetectedPerson::_kp_wrist_right_x_type arg)
  {
    msg_.kp_wrist_right_x = std::move(arg);
    return Init_DetectedPerson_kp_wrist_right_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_wrist_left_conf
{
public:
  explicit Init_DetectedPerson_kp_wrist_left_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_wrist_right_x kp_wrist_left_conf(::charmie_interfaces::msg::DetectedPerson::_kp_wrist_left_conf_type arg)
  {
    msg_.kp_wrist_left_conf = std::move(arg);
    return Init_DetectedPerson_kp_wrist_right_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_wrist_left_y
{
public:
  explicit Init_DetectedPerson_kp_wrist_left_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_wrist_left_conf kp_wrist_left_y(::charmie_interfaces::msg::DetectedPerson::_kp_wrist_left_y_type arg)
  {
    msg_.kp_wrist_left_y = std::move(arg);
    return Init_DetectedPerson_kp_wrist_left_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_wrist_left_x
{
public:
  explicit Init_DetectedPerson_kp_wrist_left_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_wrist_left_y kp_wrist_left_x(::charmie_interfaces::msg::DetectedPerson::_kp_wrist_left_x_type arg)
  {
    msg_.kp_wrist_left_x = std::move(arg);
    return Init_DetectedPerson_kp_wrist_left_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_elbow_right_conf
{
public:
  explicit Init_DetectedPerson_kp_elbow_right_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_wrist_left_x kp_elbow_right_conf(::charmie_interfaces::msg::DetectedPerson::_kp_elbow_right_conf_type arg)
  {
    msg_.kp_elbow_right_conf = std::move(arg);
    return Init_DetectedPerson_kp_wrist_left_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_elbow_right_y
{
public:
  explicit Init_DetectedPerson_kp_elbow_right_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_elbow_right_conf kp_elbow_right_y(::charmie_interfaces::msg::DetectedPerson::_kp_elbow_right_y_type arg)
  {
    msg_.kp_elbow_right_y = std::move(arg);
    return Init_DetectedPerson_kp_elbow_right_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_elbow_right_x
{
public:
  explicit Init_DetectedPerson_kp_elbow_right_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_elbow_right_y kp_elbow_right_x(::charmie_interfaces::msg::DetectedPerson::_kp_elbow_right_x_type arg)
  {
    msg_.kp_elbow_right_x = std::move(arg);
    return Init_DetectedPerson_kp_elbow_right_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_elbow_left_conf
{
public:
  explicit Init_DetectedPerson_kp_elbow_left_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_elbow_right_x kp_elbow_left_conf(::charmie_interfaces::msg::DetectedPerson::_kp_elbow_left_conf_type arg)
  {
    msg_.kp_elbow_left_conf = std::move(arg);
    return Init_DetectedPerson_kp_elbow_right_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_elbow_left_y
{
public:
  explicit Init_DetectedPerson_kp_elbow_left_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_elbow_left_conf kp_elbow_left_y(::charmie_interfaces::msg::DetectedPerson::_kp_elbow_left_y_type arg)
  {
    msg_.kp_elbow_left_y = std::move(arg);
    return Init_DetectedPerson_kp_elbow_left_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_elbow_left_x
{
public:
  explicit Init_DetectedPerson_kp_elbow_left_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_elbow_left_y kp_elbow_left_x(::charmie_interfaces::msg::DetectedPerson::_kp_elbow_left_x_type arg)
  {
    msg_.kp_elbow_left_x = std::move(arg);
    return Init_DetectedPerson_kp_elbow_left_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_shoulder_right_conf
{
public:
  explicit Init_DetectedPerson_kp_shoulder_right_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_elbow_left_x kp_shoulder_right_conf(::charmie_interfaces::msg::DetectedPerson::_kp_shoulder_right_conf_type arg)
  {
    msg_.kp_shoulder_right_conf = std::move(arg);
    return Init_DetectedPerson_kp_elbow_left_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_shoulder_right_y
{
public:
  explicit Init_DetectedPerson_kp_shoulder_right_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_shoulder_right_conf kp_shoulder_right_y(::charmie_interfaces::msg::DetectedPerson::_kp_shoulder_right_y_type arg)
  {
    msg_.kp_shoulder_right_y = std::move(arg);
    return Init_DetectedPerson_kp_shoulder_right_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_shoulder_right_x
{
public:
  explicit Init_DetectedPerson_kp_shoulder_right_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_shoulder_right_y kp_shoulder_right_x(::charmie_interfaces::msg::DetectedPerson::_kp_shoulder_right_x_type arg)
  {
    msg_.kp_shoulder_right_x = std::move(arg);
    return Init_DetectedPerson_kp_shoulder_right_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_shoulder_left_conf
{
public:
  explicit Init_DetectedPerson_kp_shoulder_left_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_shoulder_right_x kp_shoulder_left_conf(::charmie_interfaces::msg::DetectedPerson::_kp_shoulder_left_conf_type arg)
  {
    msg_.kp_shoulder_left_conf = std::move(arg);
    return Init_DetectedPerson_kp_shoulder_right_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_shoulder_left_y
{
public:
  explicit Init_DetectedPerson_kp_shoulder_left_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_shoulder_left_conf kp_shoulder_left_y(::charmie_interfaces::msg::DetectedPerson::_kp_shoulder_left_y_type arg)
  {
    msg_.kp_shoulder_left_y = std::move(arg);
    return Init_DetectedPerson_kp_shoulder_left_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_shoulder_left_x
{
public:
  explicit Init_DetectedPerson_kp_shoulder_left_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_shoulder_left_y kp_shoulder_left_x(::charmie_interfaces::msg::DetectedPerson::_kp_shoulder_left_x_type arg)
  {
    msg_.kp_shoulder_left_x = std::move(arg);
    return Init_DetectedPerson_kp_shoulder_left_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ear_right_conf
{
public:
  explicit Init_DetectedPerson_kp_ear_right_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_shoulder_left_x kp_ear_right_conf(::charmie_interfaces::msg::DetectedPerson::_kp_ear_right_conf_type arg)
  {
    msg_.kp_ear_right_conf = std::move(arg);
    return Init_DetectedPerson_kp_shoulder_left_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ear_right_y
{
public:
  explicit Init_DetectedPerson_kp_ear_right_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ear_right_conf kp_ear_right_y(::charmie_interfaces::msg::DetectedPerson::_kp_ear_right_y_type arg)
  {
    msg_.kp_ear_right_y = std::move(arg);
    return Init_DetectedPerson_kp_ear_right_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ear_right_x
{
public:
  explicit Init_DetectedPerson_kp_ear_right_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ear_right_y kp_ear_right_x(::charmie_interfaces::msg::DetectedPerson::_kp_ear_right_x_type arg)
  {
    msg_.kp_ear_right_x = std::move(arg);
    return Init_DetectedPerson_kp_ear_right_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ear_left_conf
{
public:
  explicit Init_DetectedPerson_kp_ear_left_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ear_right_x kp_ear_left_conf(::charmie_interfaces::msg::DetectedPerson::_kp_ear_left_conf_type arg)
  {
    msg_.kp_ear_left_conf = std::move(arg);
    return Init_DetectedPerson_kp_ear_right_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ear_left_y
{
public:
  explicit Init_DetectedPerson_kp_ear_left_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ear_left_conf kp_ear_left_y(::charmie_interfaces::msg::DetectedPerson::_kp_ear_left_y_type arg)
  {
    msg_.kp_ear_left_y = std::move(arg);
    return Init_DetectedPerson_kp_ear_left_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_ear_left_x
{
public:
  explicit Init_DetectedPerson_kp_ear_left_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ear_left_y kp_ear_left_x(::charmie_interfaces::msg::DetectedPerson::_kp_ear_left_x_type arg)
  {
    msg_.kp_ear_left_x = std::move(arg);
    return Init_DetectedPerson_kp_ear_left_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_eye_right_conf
{
public:
  explicit Init_DetectedPerson_kp_eye_right_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_ear_left_x kp_eye_right_conf(::charmie_interfaces::msg::DetectedPerson::_kp_eye_right_conf_type arg)
  {
    msg_.kp_eye_right_conf = std::move(arg);
    return Init_DetectedPerson_kp_ear_left_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_eye_right_y
{
public:
  explicit Init_DetectedPerson_kp_eye_right_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_eye_right_conf kp_eye_right_y(::charmie_interfaces::msg::DetectedPerson::_kp_eye_right_y_type arg)
  {
    msg_.kp_eye_right_y = std::move(arg);
    return Init_DetectedPerson_kp_eye_right_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_eye_right_x
{
public:
  explicit Init_DetectedPerson_kp_eye_right_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_eye_right_y kp_eye_right_x(::charmie_interfaces::msg::DetectedPerson::_kp_eye_right_x_type arg)
  {
    msg_.kp_eye_right_x = std::move(arg);
    return Init_DetectedPerson_kp_eye_right_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_eye_left_conf
{
public:
  explicit Init_DetectedPerson_kp_eye_left_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_eye_right_x kp_eye_left_conf(::charmie_interfaces::msg::DetectedPerson::_kp_eye_left_conf_type arg)
  {
    msg_.kp_eye_left_conf = std::move(arg);
    return Init_DetectedPerson_kp_eye_right_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_eye_left_y
{
public:
  explicit Init_DetectedPerson_kp_eye_left_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_eye_left_conf kp_eye_left_y(::charmie_interfaces::msg::DetectedPerson::_kp_eye_left_y_type arg)
  {
    msg_.kp_eye_left_y = std::move(arg);
    return Init_DetectedPerson_kp_eye_left_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_eye_left_x
{
public:
  explicit Init_DetectedPerson_kp_eye_left_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_eye_left_y kp_eye_left_x(::charmie_interfaces::msg::DetectedPerson::_kp_eye_left_x_type arg)
  {
    msg_.kp_eye_left_x = std::move(arg);
    return Init_DetectedPerson_kp_eye_left_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_nose_conf
{
public:
  explicit Init_DetectedPerson_kp_nose_conf(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_eye_left_x kp_nose_conf(::charmie_interfaces::msg::DetectedPerson::_kp_nose_conf_type arg)
  {
    msg_.kp_nose_conf = std::move(arg);
    return Init_DetectedPerson_kp_eye_left_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_nose_y
{
public:
  explicit Init_DetectedPerson_kp_nose_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_nose_conf kp_nose_y(::charmie_interfaces::msg::DetectedPerson::_kp_nose_y_type arg)
  {
    msg_.kp_nose_y = std::move(arg);
    return Init_DetectedPerson_kp_nose_conf(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_kp_nose_x
{
public:
  explicit Init_DetectedPerson_kp_nose_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_nose_y kp_nose_x(::charmie_interfaces::msg::DetectedPerson::_kp_nose_x_type arg)
  {
    msg_.kp_nose_x = std::move(arg);
    return Init_DetectedPerson_kp_nose_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_box_height
{
public:
  explicit Init_DetectedPerson_box_height(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_kp_nose_x box_height(::charmie_interfaces::msg::DetectedPerson::_box_height_type arg)
  {
    msg_.box_height = std::move(arg);
    return Init_DetectedPerson_kp_nose_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_box_width
{
public:
  explicit Init_DetectedPerson_box_width(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_box_height box_width(::charmie_interfaces::msg::DetectedPerson::_box_width_type arg)
  {
    msg_.box_width = std::move(arg);
    return Init_DetectedPerson_box_height(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_box_top_left_y
{
public:
  explicit Init_DetectedPerson_box_top_left_y(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_box_width box_top_left_y(::charmie_interfaces::msg::DetectedPerson::_box_top_left_y_type arg)
  {
    msg_.box_top_left_y = std::move(arg);
    return Init_DetectedPerson_box_width(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_box_top_left_x
{
public:
  explicit Init_DetectedPerson_box_top_left_x(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_box_top_left_y box_top_left_x(::charmie_interfaces::msg::DetectedPerson::_box_top_left_x_type arg)
  {
    msg_.box_top_left_x = std::move(arg);
    return Init_DetectedPerson_box_top_left_y(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_y_rel
{
public:
  explicit Init_DetectedPerson_y_rel(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_box_top_left_x y_rel(::charmie_interfaces::msg::DetectedPerson::_y_rel_type arg)
  {
    msg_.y_rel = std::move(arg);
    return Init_DetectedPerson_box_top_left_x(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_x_rel
{
public:
  explicit Init_DetectedPerson_x_rel(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_y_rel x_rel(::charmie_interfaces::msg::DetectedPerson::_x_rel_type arg)
  {
    msg_.x_rel = std::move(arg);
    return Init_DetectedPerson_y_rel(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_conf_person
{
public:
  explicit Init_DetectedPerson_conf_person(::charmie_interfaces::msg::DetectedPerson & msg)
  : msg_(msg)
  {}
  Init_DetectedPerson_x_rel conf_person(::charmie_interfaces::msg::DetectedPerson::_conf_person_type arg)
  {
    msg_.conf_person = std::move(arg);
    return Init_DetectedPerson_x_rel(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

class Init_DetectedPerson_index_person
{
public:
  Init_DetectedPerson_index_person()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectedPerson_conf_person index_person(::charmie_interfaces::msg::DetectedPerson::_index_person_type arg)
  {
    msg_.index_person = std::move(arg);
    return Init_DetectedPerson_conf_person(msg_);
  }

private:
  ::charmie_interfaces::msg::DetectedPerson msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::DetectedPerson>()
{
  return charmie_interfaces::msg::builder::Init_DetectedPerson_index_person();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__DETECTED_PERSON__BUILDER_HPP_
