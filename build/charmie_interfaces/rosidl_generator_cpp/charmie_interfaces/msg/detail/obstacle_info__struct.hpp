// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/ObstacleInfo.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLE_INFO__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLE_INFO__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__ObstacleInfo __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__ObstacleInfo __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObstacleInfo_
{
  using Type = ObstacleInfo_<ContainerAllocator>;

  explicit ObstacleInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->alfa = 0.0f;
      this->dist = 0.0f;
      this->length_cm = 0.0f;
      this->length_degrees = 0.0f;
    }
  }

  explicit ObstacleInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->alfa = 0.0f;
      this->dist = 0.0f;
      this->length_cm = 0.0f;
      this->length_degrees = 0.0f;
    }
  }

  // field types and members
  using _alfa_type =
    float;
  _alfa_type alfa;
  using _dist_type =
    float;
  _dist_type dist;
  using _length_cm_type =
    float;
  _length_cm_type length_cm;
  using _length_degrees_type =
    float;
  _length_degrees_type length_degrees;

  // setters for named parameter idiom
  Type & set__alfa(
    const float & _arg)
  {
    this->alfa = _arg;
    return *this;
  }
  Type & set__dist(
    const float & _arg)
  {
    this->dist = _arg;
    return *this;
  }
  Type & set__length_cm(
    const float & _arg)
  {
    this->length_cm = _arg;
    return *this;
  }
  Type & set__length_degrees(
    const float & _arg)
  {
    this->length_degrees = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__ObstacleInfo
    std::shared_ptr<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__ObstacleInfo
    std::shared_ptr<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObstacleInfo_ & other) const
  {
    if (this->alfa != other.alfa) {
      return false;
    }
    if (this->dist != other.dist) {
      return false;
    }
    if (this->length_cm != other.length_cm) {
      return false;
    }
    if (this->length_degrees != other.length_degrees) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObstacleInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObstacleInfo_

// alias to use template instance with default allocator
using ObstacleInfo =
  charmie_interfaces::msg::ObstacleInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLE_INFO__STRUCT_HPP_
