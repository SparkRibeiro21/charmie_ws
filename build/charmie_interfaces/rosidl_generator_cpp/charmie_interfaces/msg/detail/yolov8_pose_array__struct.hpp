// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/Yolov8PoseArray.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE_ARRAY__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE_ARRAY__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'data'
#include "charmie_interfaces/msg/detail/yolov8_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__Yolov8PoseArray __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__Yolov8PoseArray __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Yolov8PoseArray_
{
  using Type = Yolov8PoseArray_<ContainerAllocator>;

  explicit Yolov8PoseArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Yolov8PoseArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>, typename ContainerAllocator::template rebind<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>>::other>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>, typename ContainerAllocator::template rebind<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>>::other> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__Yolov8PoseArray
    std::shared_ptr<charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__Yolov8PoseArray
    std::shared_ptr<charmie_interfaces::msg::Yolov8PoseArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Yolov8PoseArray_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Yolov8PoseArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Yolov8PoseArray_

// alias to use template instance with default allocator
using Yolov8PoseArray =
  charmie_interfaces::msg::Yolov8PoseArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE_ARRAY__STRUCT_HPP_
