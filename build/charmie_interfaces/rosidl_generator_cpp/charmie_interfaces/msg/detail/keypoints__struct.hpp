// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/Keypoints.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__KEYPOINTS__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__KEYPOINTS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__Keypoints __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__Keypoints __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Keypoints_
{
  using Type = Keypoints_<ContainerAllocator>;

  explicit Keypoints_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index_person = 0l;
      this->average_distance = 0.0f;
      this->standard_deviation = 0.0f;
      this->key_p0_x = 0l;
      this->key_p0_y = 0l;
      this->key_p1_x = 0l;
      this->key_p1_y = 0l;
      this->key_p2_x = 0l;
      this->key_p2_y = 0l;
      this->key_p3_x = 0l;
      this->key_p3_y = 0l;
      this->key_p4_x = 0l;
      this->key_p4_y = 0l;
      this->key_p5_x = 0l;
      this->key_p5_y = 0l;
      this->key_p6_x = 0l;
      this->key_p6_y = 0l;
      this->key_p7_x = 0l;
      this->key_p7_y = 0l;
      this->key_p8_x = 0l;
      this->key_p8_y = 0l;
      this->key_p9_x = 0l;
      this->key_p9_y = 0l;
      this->key_p10_x = 0l;
      this->key_p10_y = 0l;
      this->key_p11_x = 0l;
      this->key_p11_y = 0l;
      this->key_p12_x = 0l;
      this->key_p12_y = 0l;
      this->key_p13_x = 0l;
      this->key_p13_y = 0l;
      this->key_p14_x = 0l;
      this->key_p14_y = 0l;
      this->key_p15_x = 0l;
      this->key_p15_y = 0l;
      this->key_p16_x = 0l;
      this->key_p16_y = 0l;
    }
  }

  explicit Keypoints_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->index_person = 0l;
      this->average_distance = 0.0f;
      this->standard_deviation = 0.0f;
      this->key_p0_x = 0l;
      this->key_p0_y = 0l;
      this->key_p1_x = 0l;
      this->key_p1_y = 0l;
      this->key_p2_x = 0l;
      this->key_p2_y = 0l;
      this->key_p3_x = 0l;
      this->key_p3_y = 0l;
      this->key_p4_x = 0l;
      this->key_p4_y = 0l;
      this->key_p5_x = 0l;
      this->key_p5_y = 0l;
      this->key_p6_x = 0l;
      this->key_p6_y = 0l;
      this->key_p7_x = 0l;
      this->key_p7_y = 0l;
      this->key_p8_x = 0l;
      this->key_p8_y = 0l;
      this->key_p9_x = 0l;
      this->key_p9_y = 0l;
      this->key_p10_x = 0l;
      this->key_p10_y = 0l;
      this->key_p11_x = 0l;
      this->key_p11_y = 0l;
      this->key_p12_x = 0l;
      this->key_p12_y = 0l;
      this->key_p13_x = 0l;
      this->key_p13_y = 0l;
      this->key_p14_x = 0l;
      this->key_p14_y = 0l;
      this->key_p15_x = 0l;
      this->key_p15_y = 0l;
      this->key_p16_x = 0l;
      this->key_p16_y = 0l;
    }
  }

  // field types and members
  using _index_person_type =
    int32_t;
  _index_person_type index_person;
  using _average_distance_type =
    float;
  _average_distance_type average_distance;
  using _standard_deviation_type =
    float;
  _standard_deviation_type standard_deviation;
  using _key_p0_x_type =
    int32_t;
  _key_p0_x_type key_p0_x;
  using _key_p0_y_type =
    int32_t;
  _key_p0_y_type key_p0_y;
  using _key_p1_x_type =
    int32_t;
  _key_p1_x_type key_p1_x;
  using _key_p1_y_type =
    int32_t;
  _key_p1_y_type key_p1_y;
  using _key_p2_x_type =
    int32_t;
  _key_p2_x_type key_p2_x;
  using _key_p2_y_type =
    int32_t;
  _key_p2_y_type key_p2_y;
  using _key_p3_x_type =
    int32_t;
  _key_p3_x_type key_p3_x;
  using _key_p3_y_type =
    int32_t;
  _key_p3_y_type key_p3_y;
  using _key_p4_x_type =
    int32_t;
  _key_p4_x_type key_p4_x;
  using _key_p4_y_type =
    int32_t;
  _key_p4_y_type key_p4_y;
  using _key_p5_x_type =
    int32_t;
  _key_p5_x_type key_p5_x;
  using _key_p5_y_type =
    int32_t;
  _key_p5_y_type key_p5_y;
  using _key_p6_x_type =
    int32_t;
  _key_p6_x_type key_p6_x;
  using _key_p6_y_type =
    int32_t;
  _key_p6_y_type key_p6_y;
  using _key_p7_x_type =
    int32_t;
  _key_p7_x_type key_p7_x;
  using _key_p7_y_type =
    int32_t;
  _key_p7_y_type key_p7_y;
  using _key_p8_x_type =
    int32_t;
  _key_p8_x_type key_p8_x;
  using _key_p8_y_type =
    int32_t;
  _key_p8_y_type key_p8_y;
  using _key_p9_x_type =
    int32_t;
  _key_p9_x_type key_p9_x;
  using _key_p9_y_type =
    int32_t;
  _key_p9_y_type key_p9_y;
  using _key_p10_x_type =
    int32_t;
  _key_p10_x_type key_p10_x;
  using _key_p10_y_type =
    int32_t;
  _key_p10_y_type key_p10_y;
  using _key_p11_x_type =
    int32_t;
  _key_p11_x_type key_p11_x;
  using _key_p11_y_type =
    int32_t;
  _key_p11_y_type key_p11_y;
  using _key_p12_x_type =
    int32_t;
  _key_p12_x_type key_p12_x;
  using _key_p12_y_type =
    int32_t;
  _key_p12_y_type key_p12_y;
  using _key_p13_x_type =
    int32_t;
  _key_p13_x_type key_p13_x;
  using _key_p13_y_type =
    int32_t;
  _key_p13_y_type key_p13_y;
  using _key_p14_x_type =
    int32_t;
  _key_p14_x_type key_p14_x;
  using _key_p14_y_type =
    int32_t;
  _key_p14_y_type key_p14_y;
  using _key_p15_x_type =
    int32_t;
  _key_p15_x_type key_p15_x;
  using _key_p15_y_type =
    int32_t;
  _key_p15_y_type key_p15_y;
  using _key_p16_x_type =
    int32_t;
  _key_p16_x_type key_p16_x;
  using _key_p16_y_type =
    int32_t;
  _key_p16_y_type key_p16_y;

  // setters for named parameter idiom
  Type & set__index_person(
    const int32_t & _arg)
  {
    this->index_person = _arg;
    return *this;
  }
  Type & set__average_distance(
    const float & _arg)
  {
    this->average_distance = _arg;
    return *this;
  }
  Type & set__standard_deviation(
    const float & _arg)
  {
    this->standard_deviation = _arg;
    return *this;
  }
  Type & set__key_p0_x(
    const int32_t & _arg)
  {
    this->key_p0_x = _arg;
    return *this;
  }
  Type & set__key_p0_y(
    const int32_t & _arg)
  {
    this->key_p0_y = _arg;
    return *this;
  }
  Type & set__key_p1_x(
    const int32_t & _arg)
  {
    this->key_p1_x = _arg;
    return *this;
  }
  Type & set__key_p1_y(
    const int32_t & _arg)
  {
    this->key_p1_y = _arg;
    return *this;
  }
  Type & set__key_p2_x(
    const int32_t & _arg)
  {
    this->key_p2_x = _arg;
    return *this;
  }
  Type & set__key_p2_y(
    const int32_t & _arg)
  {
    this->key_p2_y = _arg;
    return *this;
  }
  Type & set__key_p3_x(
    const int32_t & _arg)
  {
    this->key_p3_x = _arg;
    return *this;
  }
  Type & set__key_p3_y(
    const int32_t & _arg)
  {
    this->key_p3_y = _arg;
    return *this;
  }
  Type & set__key_p4_x(
    const int32_t & _arg)
  {
    this->key_p4_x = _arg;
    return *this;
  }
  Type & set__key_p4_y(
    const int32_t & _arg)
  {
    this->key_p4_y = _arg;
    return *this;
  }
  Type & set__key_p5_x(
    const int32_t & _arg)
  {
    this->key_p5_x = _arg;
    return *this;
  }
  Type & set__key_p5_y(
    const int32_t & _arg)
  {
    this->key_p5_y = _arg;
    return *this;
  }
  Type & set__key_p6_x(
    const int32_t & _arg)
  {
    this->key_p6_x = _arg;
    return *this;
  }
  Type & set__key_p6_y(
    const int32_t & _arg)
  {
    this->key_p6_y = _arg;
    return *this;
  }
  Type & set__key_p7_x(
    const int32_t & _arg)
  {
    this->key_p7_x = _arg;
    return *this;
  }
  Type & set__key_p7_y(
    const int32_t & _arg)
  {
    this->key_p7_y = _arg;
    return *this;
  }
  Type & set__key_p8_x(
    const int32_t & _arg)
  {
    this->key_p8_x = _arg;
    return *this;
  }
  Type & set__key_p8_y(
    const int32_t & _arg)
  {
    this->key_p8_y = _arg;
    return *this;
  }
  Type & set__key_p9_x(
    const int32_t & _arg)
  {
    this->key_p9_x = _arg;
    return *this;
  }
  Type & set__key_p9_y(
    const int32_t & _arg)
  {
    this->key_p9_y = _arg;
    return *this;
  }
  Type & set__key_p10_x(
    const int32_t & _arg)
  {
    this->key_p10_x = _arg;
    return *this;
  }
  Type & set__key_p10_y(
    const int32_t & _arg)
  {
    this->key_p10_y = _arg;
    return *this;
  }
  Type & set__key_p11_x(
    const int32_t & _arg)
  {
    this->key_p11_x = _arg;
    return *this;
  }
  Type & set__key_p11_y(
    const int32_t & _arg)
  {
    this->key_p11_y = _arg;
    return *this;
  }
  Type & set__key_p12_x(
    const int32_t & _arg)
  {
    this->key_p12_x = _arg;
    return *this;
  }
  Type & set__key_p12_y(
    const int32_t & _arg)
  {
    this->key_p12_y = _arg;
    return *this;
  }
  Type & set__key_p13_x(
    const int32_t & _arg)
  {
    this->key_p13_x = _arg;
    return *this;
  }
  Type & set__key_p13_y(
    const int32_t & _arg)
  {
    this->key_p13_y = _arg;
    return *this;
  }
  Type & set__key_p14_x(
    const int32_t & _arg)
  {
    this->key_p14_x = _arg;
    return *this;
  }
  Type & set__key_p14_y(
    const int32_t & _arg)
  {
    this->key_p14_y = _arg;
    return *this;
  }
  Type & set__key_p15_x(
    const int32_t & _arg)
  {
    this->key_p15_x = _arg;
    return *this;
  }
  Type & set__key_p15_y(
    const int32_t & _arg)
  {
    this->key_p15_y = _arg;
    return *this;
  }
  Type & set__key_p16_x(
    const int32_t & _arg)
  {
    this->key_p16_x = _arg;
    return *this;
  }
  Type & set__key_p16_y(
    const int32_t & _arg)
  {
    this->key_p16_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::Keypoints_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::Keypoints_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Keypoints_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Keypoints_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Keypoints_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Keypoints_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Keypoints_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Keypoints_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Keypoints_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Keypoints_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__Keypoints
    std::shared_ptr<charmie_interfaces::msg::Keypoints_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__Keypoints
    std::shared_ptr<charmie_interfaces::msg::Keypoints_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Keypoints_ & other) const
  {
    if (this->index_person != other.index_person) {
      return false;
    }
    if (this->average_distance != other.average_distance) {
      return false;
    }
    if (this->standard_deviation != other.standard_deviation) {
      return false;
    }
    if (this->key_p0_x != other.key_p0_x) {
      return false;
    }
    if (this->key_p0_y != other.key_p0_y) {
      return false;
    }
    if (this->key_p1_x != other.key_p1_x) {
      return false;
    }
    if (this->key_p1_y != other.key_p1_y) {
      return false;
    }
    if (this->key_p2_x != other.key_p2_x) {
      return false;
    }
    if (this->key_p2_y != other.key_p2_y) {
      return false;
    }
    if (this->key_p3_x != other.key_p3_x) {
      return false;
    }
    if (this->key_p3_y != other.key_p3_y) {
      return false;
    }
    if (this->key_p4_x != other.key_p4_x) {
      return false;
    }
    if (this->key_p4_y != other.key_p4_y) {
      return false;
    }
    if (this->key_p5_x != other.key_p5_x) {
      return false;
    }
    if (this->key_p5_y != other.key_p5_y) {
      return false;
    }
    if (this->key_p6_x != other.key_p6_x) {
      return false;
    }
    if (this->key_p6_y != other.key_p6_y) {
      return false;
    }
    if (this->key_p7_x != other.key_p7_x) {
      return false;
    }
    if (this->key_p7_y != other.key_p7_y) {
      return false;
    }
    if (this->key_p8_x != other.key_p8_x) {
      return false;
    }
    if (this->key_p8_y != other.key_p8_y) {
      return false;
    }
    if (this->key_p9_x != other.key_p9_x) {
      return false;
    }
    if (this->key_p9_y != other.key_p9_y) {
      return false;
    }
    if (this->key_p10_x != other.key_p10_x) {
      return false;
    }
    if (this->key_p10_y != other.key_p10_y) {
      return false;
    }
    if (this->key_p11_x != other.key_p11_x) {
      return false;
    }
    if (this->key_p11_y != other.key_p11_y) {
      return false;
    }
    if (this->key_p12_x != other.key_p12_x) {
      return false;
    }
    if (this->key_p12_y != other.key_p12_y) {
      return false;
    }
    if (this->key_p13_x != other.key_p13_x) {
      return false;
    }
    if (this->key_p13_y != other.key_p13_y) {
      return false;
    }
    if (this->key_p14_x != other.key_p14_x) {
      return false;
    }
    if (this->key_p14_y != other.key_p14_y) {
      return false;
    }
    if (this->key_p15_x != other.key_p15_x) {
      return false;
    }
    if (this->key_p15_y != other.key_p15_y) {
      return false;
    }
    if (this->key_p16_x != other.key_p16_x) {
      return false;
    }
    if (this->key_p16_y != other.key_p16_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Keypoints_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Keypoints_

// alias to use template instance with default allocator
using Keypoints =
  charmie_interfaces::msg::Keypoints_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__KEYPOINTS__STRUCT_HPP_
