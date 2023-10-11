// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/DetectedPerson.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__DETECTED_PERSON__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__DETECTED_PERSON__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__DetectedPerson __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__DetectedPerson __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectedPerson_
{
  using Type = DetectedPerson_<ContainerAllocator>;

  explicit DetectedPerson_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index_person = 0l;
      this->conf_person = 0.0f;
      this->x_rel = 0.0f;
      this->y_rel = 0.0f;
      this->box_top_left_x = 0l;
      this->box_top_left_y = 0l;
      this->box_width = 0l;
      this->box_height = 0l;
      this->kp_nose_x = 0l;
      this->kp_nose_y = 0l;
      this->kp_nose_conf = 0.0f;
      this->kp_eye_left_x = 0l;
      this->kp_eye_left_y = 0l;
      this->kp_eye_left_conf = 0.0f;
      this->kp_eye_right_x = 0l;
      this->kp_eye_right_y = 0l;
      this->kp_eye_right_conf = 0.0f;
      this->kp_ear_left_x = 0l;
      this->kp_ear_left_y = 0l;
      this->kp_ear_left_conf = 0.0f;
      this->kp_ear_right_x = 0l;
      this->kp_ear_right_y = 0l;
      this->kp_ear_right_conf = 0.0f;
      this->kp_shoulder_left_x = 0l;
      this->kp_shoulder_left_y = 0l;
      this->kp_shoulder_left_conf = 0.0f;
      this->kp_shoulder_right_x = 0l;
      this->kp_shoulder_right_y = 0l;
      this->kp_shoulder_right_conf = 0.0f;
      this->kp_elbow_left_x = 0l;
      this->kp_elbow_left_y = 0l;
      this->kp_elbow_left_conf = 0.0f;
      this->kp_elbow_right_x = 0l;
      this->kp_elbow_right_y = 0l;
      this->kp_elbow_right_conf = 0.0f;
      this->kp_wrist_left_x = 0l;
      this->kp_wrist_left_y = 0l;
      this->kp_wrist_left_conf = 0.0f;
      this->kp_wrist_right_x = 0l;
      this->kp_wrist_right_y = 0l;
      this->kp_wrist_right_conf = 0.0f;
      this->kp_hip_left_x = 0l;
      this->kp_hip_left_y = 0l;
      this->kp_hip_left_conf = 0.0f;
      this->kp_hip_right_x = 0l;
      this->kp_hip_right_y = 0l;
      this->kp_hip_right_conf = 0.0f;
      this->kp_knee_left_x = 0l;
      this->kp_knee_left_y = 0l;
      this->kp_knee_left_conf = 0.0f;
      this->kp_knee_right_x = 0l;
      this->kp_knee_right_y = 0l;
      this->kp_knee_right_conf = 0.0f;
      this->kp_ankle_left_x = 0l;
      this->kp_ankle_left_y = 0l;
      this->kp_ankle_left_conf = 0.0f;
      this->kp_ankle_right_x = 0l;
      this->kp_ankle_right_y = 0l;
      this->kp_ankle_right_conf = 0.0f;
    }
  }

  explicit DetectedPerson_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index_person = 0l;
      this->conf_person = 0.0f;
      this->x_rel = 0.0f;
      this->y_rel = 0.0f;
      this->box_top_left_x = 0l;
      this->box_top_left_y = 0l;
      this->box_width = 0l;
      this->box_height = 0l;
      this->kp_nose_x = 0l;
      this->kp_nose_y = 0l;
      this->kp_nose_conf = 0.0f;
      this->kp_eye_left_x = 0l;
      this->kp_eye_left_y = 0l;
      this->kp_eye_left_conf = 0.0f;
      this->kp_eye_right_x = 0l;
      this->kp_eye_right_y = 0l;
      this->kp_eye_right_conf = 0.0f;
      this->kp_ear_left_x = 0l;
      this->kp_ear_left_y = 0l;
      this->kp_ear_left_conf = 0.0f;
      this->kp_ear_right_x = 0l;
      this->kp_ear_right_y = 0l;
      this->kp_ear_right_conf = 0.0f;
      this->kp_shoulder_left_x = 0l;
      this->kp_shoulder_left_y = 0l;
      this->kp_shoulder_left_conf = 0.0f;
      this->kp_shoulder_right_x = 0l;
      this->kp_shoulder_right_y = 0l;
      this->kp_shoulder_right_conf = 0.0f;
      this->kp_elbow_left_x = 0l;
      this->kp_elbow_left_y = 0l;
      this->kp_elbow_left_conf = 0.0f;
      this->kp_elbow_right_x = 0l;
      this->kp_elbow_right_y = 0l;
      this->kp_elbow_right_conf = 0.0f;
      this->kp_wrist_left_x = 0l;
      this->kp_wrist_left_y = 0l;
      this->kp_wrist_left_conf = 0.0f;
      this->kp_wrist_right_x = 0l;
      this->kp_wrist_right_y = 0l;
      this->kp_wrist_right_conf = 0.0f;
      this->kp_hip_left_x = 0l;
      this->kp_hip_left_y = 0l;
      this->kp_hip_left_conf = 0.0f;
      this->kp_hip_right_x = 0l;
      this->kp_hip_right_y = 0l;
      this->kp_hip_right_conf = 0.0f;
      this->kp_knee_left_x = 0l;
      this->kp_knee_left_y = 0l;
      this->kp_knee_left_conf = 0.0f;
      this->kp_knee_right_x = 0l;
      this->kp_knee_right_y = 0l;
      this->kp_knee_right_conf = 0.0f;
      this->kp_ankle_left_x = 0l;
      this->kp_ankle_left_y = 0l;
      this->kp_ankle_left_conf = 0.0f;
      this->kp_ankle_right_x = 0l;
      this->kp_ankle_right_y = 0l;
      this->kp_ankle_right_conf = 0.0f;
    }
  }

  // field types and members
  using _index_person_type =
    int32_t;
  _index_person_type index_person;
  using _conf_person_type =
    float;
  _conf_person_type conf_person;
  using _x_rel_type =
    float;
  _x_rel_type x_rel;
  using _y_rel_type =
    float;
  _y_rel_type y_rel;
  using _box_top_left_x_type =
    int32_t;
  _box_top_left_x_type box_top_left_x;
  using _box_top_left_y_type =
    int32_t;
  _box_top_left_y_type box_top_left_y;
  using _box_width_type =
    int32_t;
  _box_width_type box_width;
  using _box_height_type =
    int32_t;
  _box_height_type box_height;
  using _kp_nose_x_type =
    int32_t;
  _kp_nose_x_type kp_nose_x;
  using _kp_nose_y_type =
    int32_t;
  _kp_nose_y_type kp_nose_y;
  using _kp_nose_conf_type =
    float;
  _kp_nose_conf_type kp_nose_conf;
  using _kp_eye_left_x_type =
    int32_t;
  _kp_eye_left_x_type kp_eye_left_x;
  using _kp_eye_left_y_type =
    int32_t;
  _kp_eye_left_y_type kp_eye_left_y;
  using _kp_eye_left_conf_type =
    float;
  _kp_eye_left_conf_type kp_eye_left_conf;
  using _kp_eye_right_x_type =
    int32_t;
  _kp_eye_right_x_type kp_eye_right_x;
  using _kp_eye_right_y_type =
    int32_t;
  _kp_eye_right_y_type kp_eye_right_y;
  using _kp_eye_right_conf_type =
    float;
  _kp_eye_right_conf_type kp_eye_right_conf;
  using _kp_ear_left_x_type =
    int32_t;
  _kp_ear_left_x_type kp_ear_left_x;
  using _kp_ear_left_y_type =
    int32_t;
  _kp_ear_left_y_type kp_ear_left_y;
  using _kp_ear_left_conf_type =
    float;
  _kp_ear_left_conf_type kp_ear_left_conf;
  using _kp_ear_right_x_type =
    int32_t;
  _kp_ear_right_x_type kp_ear_right_x;
  using _kp_ear_right_y_type =
    int32_t;
  _kp_ear_right_y_type kp_ear_right_y;
  using _kp_ear_right_conf_type =
    float;
  _kp_ear_right_conf_type kp_ear_right_conf;
  using _kp_shoulder_left_x_type =
    int32_t;
  _kp_shoulder_left_x_type kp_shoulder_left_x;
  using _kp_shoulder_left_y_type =
    int32_t;
  _kp_shoulder_left_y_type kp_shoulder_left_y;
  using _kp_shoulder_left_conf_type =
    float;
  _kp_shoulder_left_conf_type kp_shoulder_left_conf;
  using _kp_shoulder_right_x_type =
    int32_t;
  _kp_shoulder_right_x_type kp_shoulder_right_x;
  using _kp_shoulder_right_y_type =
    int32_t;
  _kp_shoulder_right_y_type kp_shoulder_right_y;
  using _kp_shoulder_right_conf_type =
    float;
  _kp_shoulder_right_conf_type kp_shoulder_right_conf;
  using _kp_elbow_left_x_type =
    int32_t;
  _kp_elbow_left_x_type kp_elbow_left_x;
  using _kp_elbow_left_y_type =
    int32_t;
  _kp_elbow_left_y_type kp_elbow_left_y;
  using _kp_elbow_left_conf_type =
    float;
  _kp_elbow_left_conf_type kp_elbow_left_conf;
  using _kp_elbow_right_x_type =
    int32_t;
  _kp_elbow_right_x_type kp_elbow_right_x;
  using _kp_elbow_right_y_type =
    int32_t;
  _kp_elbow_right_y_type kp_elbow_right_y;
  using _kp_elbow_right_conf_type =
    float;
  _kp_elbow_right_conf_type kp_elbow_right_conf;
  using _kp_wrist_left_x_type =
    int32_t;
  _kp_wrist_left_x_type kp_wrist_left_x;
  using _kp_wrist_left_y_type =
    int32_t;
  _kp_wrist_left_y_type kp_wrist_left_y;
  using _kp_wrist_left_conf_type =
    float;
  _kp_wrist_left_conf_type kp_wrist_left_conf;
  using _kp_wrist_right_x_type =
    int32_t;
  _kp_wrist_right_x_type kp_wrist_right_x;
  using _kp_wrist_right_y_type =
    int32_t;
  _kp_wrist_right_y_type kp_wrist_right_y;
  using _kp_wrist_right_conf_type =
    float;
  _kp_wrist_right_conf_type kp_wrist_right_conf;
  using _kp_hip_left_x_type =
    int32_t;
  _kp_hip_left_x_type kp_hip_left_x;
  using _kp_hip_left_y_type =
    int32_t;
  _kp_hip_left_y_type kp_hip_left_y;
  using _kp_hip_left_conf_type =
    float;
  _kp_hip_left_conf_type kp_hip_left_conf;
  using _kp_hip_right_x_type =
    int32_t;
  _kp_hip_right_x_type kp_hip_right_x;
  using _kp_hip_right_y_type =
    int32_t;
  _kp_hip_right_y_type kp_hip_right_y;
  using _kp_hip_right_conf_type =
    float;
  _kp_hip_right_conf_type kp_hip_right_conf;
  using _kp_knee_left_x_type =
    int32_t;
  _kp_knee_left_x_type kp_knee_left_x;
  using _kp_knee_left_y_type =
    int32_t;
  _kp_knee_left_y_type kp_knee_left_y;
  using _kp_knee_left_conf_type =
    float;
  _kp_knee_left_conf_type kp_knee_left_conf;
  using _kp_knee_right_x_type =
    int32_t;
  _kp_knee_right_x_type kp_knee_right_x;
  using _kp_knee_right_y_type =
    int32_t;
  _kp_knee_right_y_type kp_knee_right_y;
  using _kp_knee_right_conf_type =
    float;
  _kp_knee_right_conf_type kp_knee_right_conf;
  using _kp_ankle_left_x_type =
    int32_t;
  _kp_ankle_left_x_type kp_ankle_left_x;
  using _kp_ankle_left_y_type =
    int32_t;
  _kp_ankle_left_y_type kp_ankle_left_y;
  using _kp_ankle_left_conf_type =
    float;
  _kp_ankle_left_conf_type kp_ankle_left_conf;
  using _kp_ankle_right_x_type =
    int32_t;
  _kp_ankle_right_x_type kp_ankle_right_x;
  using _kp_ankle_right_y_type =
    int32_t;
  _kp_ankle_right_y_type kp_ankle_right_y;
  using _kp_ankle_right_conf_type =
    float;
  _kp_ankle_right_conf_type kp_ankle_right_conf;

  // setters for named parameter idiom
  Type & set__index_person(
    const int32_t & _arg)
  {
    this->index_person = _arg;
    return *this;
  }
  Type & set__conf_person(
    const float & _arg)
  {
    this->conf_person = _arg;
    return *this;
  }
  Type & set__x_rel(
    const float & _arg)
  {
    this->x_rel = _arg;
    return *this;
  }
  Type & set__y_rel(
    const float & _arg)
  {
    this->y_rel = _arg;
    return *this;
  }
  Type & set__box_top_left_x(
    const int32_t & _arg)
  {
    this->box_top_left_x = _arg;
    return *this;
  }
  Type & set__box_top_left_y(
    const int32_t & _arg)
  {
    this->box_top_left_y = _arg;
    return *this;
  }
  Type & set__box_width(
    const int32_t & _arg)
  {
    this->box_width = _arg;
    return *this;
  }
  Type & set__box_height(
    const int32_t & _arg)
  {
    this->box_height = _arg;
    return *this;
  }
  Type & set__kp_nose_x(
    const int32_t & _arg)
  {
    this->kp_nose_x = _arg;
    return *this;
  }
  Type & set__kp_nose_y(
    const int32_t & _arg)
  {
    this->kp_nose_y = _arg;
    return *this;
  }
  Type & set__kp_nose_conf(
    const float & _arg)
  {
    this->kp_nose_conf = _arg;
    return *this;
  }
  Type & set__kp_eye_left_x(
    const int32_t & _arg)
  {
    this->kp_eye_left_x = _arg;
    return *this;
  }
  Type & set__kp_eye_left_y(
    const int32_t & _arg)
  {
    this->kp_eye_left_y = _arg;
    return *this;
  }
  Type & set__kp_eye_left_conf(
    const float & _arg)
  {
    this->kp_eye_left_conf = _arg;
    return *this;
  }
  Type & set__kp_eye_right_x(
    const int32_t & _arg)
  {
    this->kp_eye_right_x = _arg;
    return *this;
  }
  Type & set__kp_eye_right_y(
    const int32_t & _arg)
  {
    this->kp_eye_right_y = _arg;
    return *this;
  }
  Type & set__kp_eye_right_conf(
    const float & _arg)
  {
    this->kp_eye_right_conf = _arg;
    return *this;
  }
  Type & set__kp_ear_left_x(
    const int32_t & _arg)
  {
    this->kp_ear_left_x = _arg;
    return *this;
  }
  Type & set__kp_ear_left_y(
    const int32_t & _arg)
  {
    this->kp_ear_left_y = _arg;
    return *this;
  }
  Type & set__kp_ear_left_conf(
    const float & _arg)
  {
    this->kp_ear_left_conf = _arg;
    return *this;
  }
  Type & set__kp_ear_right_x(
    const int32_t & _arg)
  {
    this->kp_ear_right_x = _arg;
    return *this;
  }
  Type & set__kp_ear_right_y(
    const int32_t & _arg)
  {
    this->kp_ear_right_y = _arg;
    return *this;
  }
  Type & set__kp_ear_right_conf(
    const float & _arg)
  {
    this->kp_ear_right_conf = _arg;
    return *this;
  }
  Type & set__kp_shoulder_left_x(
    const int32_t & _arg)
  {
    this->kp_shoulder_left_x = _arg;
    return *this;
  }
  Type & set__kp_shoulder_left_y(
    const int32_t & _arg)
  {
    this->kp_shoulder_left_y = _arg;
    return *this;
  }
  Type & set__kp_shoulder_left_conf(
    const float & _arg)
  {
    this->kp_shoulder_left_conf = _arg;
    return *this;
  }
  Type & set__kp_shoulder_right_x(
    const int32_t & _arg)
  {
    this->kp_shoulder_right_x = _arg;
    return *this;
  }
  Type & set__kp_shoulder_right_y(
    const int32_t & _arg)
  {
    this->kp_shoulder_right_y = _arg;
    return *this;
  }
  Type & set__kp_shoulder_right_conf(
    const float & _arg)
  {
    this->kp_shoulder_right_conf = _arg;
    return *this;
  }
  Type & set__kp_elbow_left_x(
    const int32_t & _arg)
  {
    this->kp_elbow_left_x = _arg;
    return *this;
  }
  Type & set__kp_elbow_left_y(
    const int32_t & _arg)
  {
    this->kp_elbow_left_y = _arg;
    return *this;
  }
  Type & set__kp_elbow_left_conf(
    const float & _arg)
  {
    this->kp_elbow_left_conf = _arg;
    return *this;
  }
  Type & set__kp_elbow_right_x(
    const int32_t & _arg)
  {
    this->kp_elbow_right_x = _arg;
    return *this;
  }
  Type & set__kp_elbow_right_y(
    const int32_t & _arg)
  {
    this->kp_elbow_right_y = _arg;
    return *this;
  }
  Type & set__kp_elbow_right_conf(
    const float & _arg)
  {
    this->kp_elbow_right_conf = _arg;
    return *this;
  }
  Type & set__kp_wrist_left_x(
    const int32_t & _arg)
  {
    this->kp_wrist_left_x = _arg;
    return *this;
  }
  Type & set__kp_wrist_left_y(
    const int32_t & _arg)
  {
    this->kp_wrist_left_y = _arg;
    return *this;
  }
  Type & set__kp_wrist_left_conf(
    const float & _arg)
  {
    this->kp_wrist_left_conf = _arg;
    return *this;
  }
  Type & set__kp_wrist_right_x(
    const int32_t & _arg)
  {
    this->kp_wrist_right_x = _arg;
    return *this;
  }
  Type & set__kp_wrist_right_y(
    const int32_t & _arg)
  {
    this->kp_wrist_right_y = _arg;
    return *this;
  }
  Type & set__kp_wrist_right_conf(
    const float & _arg)
  {
    this->kp_wrist_right_conf = _arg;
    return *this;
  }
  Type & set__kp_hip_left_x(
    const int32_t & _arg)
  {
    this->kp_hip_left_x = _arg;
    return *this;
  }
  Type & set__kp_hip_left_y(
    const int32_t & _arg)
  {
    this->kp_hip_left_y = _arg;
    return *this;
  }
  Type & set__kp_hip_left_conf(
    const float & _arg)
  {
    this->kp_hip_left_conf = _arg;
    return *this;
  }
  Type & set__kp_hip_right_x(
    const int32_t & _arg)
  {
    this->kp_hip_right_x = _arg;
    return *this;
  }
  Type & set__kp_hip_right_y(
    const int32_t & _arg)
  {
    this->kp_hip_right_y = _arg;
    return *this;
  }
  Type & set__kp_hip_right_conf(
    const float & _arg)
  {
    this->kp_hip_right_conf = _arg;
    return *this;
  }
  Type & set__kp_knee_left_x(
    const int32_t & _arg)
  {
    this->kp_knee_left_x = _arg;
    return *this;
  }
  Type & set__kp_knee_left_y(
    const int32_t & _arg)
  {
    this->kp_knee_left_y = _arg;
    return *this;
  }
  Type & set__kp_knee_left_conf(
    const float & _arg)
  {
    this->kp_knee_left_conf = _arg;
    return *this;
  }
  Type & set__kp_knee_right_x(
    const int32_t & _arg)
  {
    this->kp_knee_right_x = _arg;
    return *this;
  }
  Type & set__kp_knee_right_y(
    const int32_t & _arg)
  {
    this->kp_knee_right_y = _arg;
    return *this;
  }
  Type & set__kp_knee_right_conf(
    const float & _arg)
  {
    this->kp_knee_right_conf = _arg;
    return *this;
  }
  Type & set__kp_ankle_left_x(
    const int32_t & _arg)
  {
    this->kp_ankle_left_x = _arg;
    return *this;
  }
  Type & set__kp_ankle_left_y(
    const int32_t & _arg)
  {
    this->kp_ankle_left_y = _arg;
    return *this;
  }
  Type & set__kp_ankle_left_conf(
    const float & _arg)
  {
    this->kp_ankle_left_conf = _arg;
    return *this;
  }
  Type & set__kp_ankle_right_x(
    const int32_t & _arg)
  {
    this->kp_ankle_right_x = _arg;
    return *this;
  }
  Type & set__kp_ankle_right_y(
    const int32_t & _arg)
  {
    this->kp_ankle_right_y = _arg;
    return *this;
  }
  Type & set__kp_ankle_right_conf(
    const float & _arg)
  {
    this->kp_ankle_right_conf = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::DetectedPerson_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::DetectedPerson_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::DetectedPerson_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::DetectedPerson_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::DetectedPerson_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::DetectedPerson_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::DetectedPerson_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::DetectedPerson_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::DetectedPerson_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::DetectedPerson_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__DetectedPerson
    std::shared_ptr<charmie_interfaces::msg::DetectedPerson_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__DetectedPerson
    std::shared_ptr<charmie_interfaces::msg::DetectedPerson_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectedPerson_ & other) const
  {
    if (this->index_person != other.index_person) {
      return false;
    }
    if (this->conf_person != other.conf_person) {
      return false;
    }
    if (this->x_rel != other.x_rel) {
      return false;
    }
    if (this->y_rel != other.y_rel) {
      return false;
    }
    if (this->box_top_left_x != other.box_top_left_x) {
      return false;
    }
    if (this->box_top_left_y != other.box_top_left_y) {
      return false;
    }
    if (this->box_width != other.box_width) {
      return false;
    }
    if (this->box_height != other.box_height) {
      return false;
    }
    if (this->kp_nose_x != other.kp_nose_x) {
      return false;
    }
    if (this->kp_nose_y != other.kp_nose_y) {
      return false;
    }
    if (this->kp_nose_conf != other.kp_nose_conf) {
      return false;
    }
    if (this->kp_eye_left_x != other.kp_eye_left_x) {
      return false;
    }
    if (this->kp_eye_left_y != other.kp_eye_left_y) {
      return false;
    }
    if (this->kp_eye_left_conf != other.kp_eye_left_conf) {
      return false;
    }
    if (this->kp_eye_right_x != other.kp_eye_right_x) {
      return false;
    }
    if (this->kp_eye_right_y != other.kp_eye_right_y) {
      return false;
    }
    if (this->kp_eye_right_conf != other.kp_eye_right_conf) {
      return false;
    }
    if (this->kp_ear_left_x != other.kp_ear_left_x) {
      return false;
    }
    if (this->kp_ear_left_y != other.kp_ear_left_y) {
      return false;
    }
    if (this->kp_ear_left_conf != other.kp_ear_left_conf) {
      return false;
    }
    if (this->kp_ear_right_x != other.kp_ear_right_x) {
      return false;
    }
    if (this->kp_ear_right_y != other.kp_ear_right_y) {
      return false;
    }
    if (this->kp_ear_right_conf != other.kp_ear_right_conf) {
      return false;
    }
    if (this->kp_shoulder_left_x != other.kp_shoulder_left_x) {
      return false;
    }
    if (this->kp_shoulder_left_y != other.kp_shoulder_left_y) {
      return false;
    }
    if (this->kp_shoulder_left_conf != other.kp_shoulder_left_conf) {
      return false;
    }
    if (this->kp_shoulder_right_x != other.kp_shoulder_right_x) {
      return false;
    }
    if (this->kp_shoulder_right_y != other.kp_shoulder_right_y) {
      return false;
    }
    if (this->kp_shoulder_right_conf != other.kp_shoulder_right_conf) {
      return false;
    }
    if (this->kp_elbow_left_x != other.kp_elbow_left_x) {
      return false;
    }
    if (this->kp_elbow_left_y != other.kp_elbow_left_y) {
      return false;
    }
    if (this->kp_elbow_left_conf != other.kp_elbow_left_conf) {
      return false;
    }
    if (this->kp_elbow_right_x != other.kp_elbow_right_x) {
      return false;
    }
    if (this->kp_elbow_right_y != other.kp_elbow_right_y) {
      return false;
    }
    if (this->kp_elbow_right_conf != other.kp_elbow_right_conf) {
      return false;
    }
    if (this->kp_wrist_left_x != other.kp_wrist_left_x) {
      return false;
    }
    if (this->kp_wrist_left_y != other.kp_wrist_left_y) {
      return false;
    }
    if (this->kp_wrist_left_conf != other.kp_wrist_left_conf) {
      return false;
    }
    if (this->kp_wrist_right_x != other.kp_wrist_right_x) {
      return false;
    }
    if (this->kp_wrist_right_y != other.kp_wrist_right_y) {
      return false;
    }
    if (this->kp_wrist_right_conf != other.kp_wrist_right_conf) {
      return false;
    }
    if (this->kp_hip_left_x != other.kp_hip_left_x) {
      return false;
    }
    if (this->kp_hip_left_y != other.kp_hip_left_y) {
      return false;
    }
    if (this->kp_hip_left_conf != other.kp_hip_left_conf) {
      return false;
    }
    if (this->kp_hip_right_x != other.kp_hip_right_x) {
      return false;
    }
    if (this->kp_hip_right_y != other.kp_hip_right_y) {
      return false;
    }
    if (this->kp_hip_right_conf != other.kp_hip_right_conf) {
      return false;
    }
    if (this->kp_knee_left_x != other.kp_knee_left_x) {
      return false;
    }
    if (this->kp_knee_left_y != other.kp_knee_left_y) {
      return false;
    }
    if (this->kp_knee_left_conf != other.kp_knee_left_conf) {
      return false;
    }
    if (this->kp_knee_right_x != other.kp_knee_right_x) {
      return false;
    }
    if (this->kp_knee_right_y != other.kp_knee_right_y) {
      return false;
    }
    if (this->kp_knee_right_conf != other.kp_knee_right_conf) {
      return false;
    }
    if (this->kp_ankle_left_x != other.kp_ankle_left_x) {
      return false;
    }
    if (this->kp_ankle_left_y != other.kp_ankle_left_y) {
      return false;
    }
    if (this->kp_ankle_left_conf != other.kp_ankle_left_conf) {
      return false;
    }
    if (this->kp_ankle_right_x != other.kp_ankle_right_x) {
      return false;
    }
    if (this->kp_ankle_right_y != other.kp_ankle_right_y) {
      return false;
    }
    if (this->kp_ankle_right_conf != other.kp_ankle_right_conf) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectedPerson_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectedPerson_

// alias to use template instance with default allocator
using DetectedPerson =
  charmie_interfaces::msg::DetectedPerson_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__DETECTED_PERSON__STRUCT_HPP_
