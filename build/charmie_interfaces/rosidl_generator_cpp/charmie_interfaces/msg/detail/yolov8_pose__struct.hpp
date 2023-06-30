// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/Yolov8Pose.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'keypoints'
#include "charmie_interfaces/msg/detail/keypoints__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__Yolov8Pose __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__Yolov8Pose __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Yolov8Pose_
{
  using Type = Yolov8Pose_<ContainerAllocator>;

  explicit Yolov8Pose_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_person = 0l;
    }
  }

  explicit Yolov8Pose_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_person = 0l;
    }
  }

  // field types and members
  using _num_person_type =
    int32_t;
  _num_person_type num_person;
  using _keypoints_type =
    std::vector<charmie_interfaces::msg::Keypoints_<ContainerAllocator>, typename ContainerAllocator::template rebind<charmie_interfaces::msg::Keypoints_<ContainerAllocator>>::other>;
  _keypoints_type keypoints;

  // setters for named parameter idiom
  Type & set__num_person(
    const int32_t & _arg)
  {
    this->num_person = _arg;
    return *this;
  }
  Type & set__keypoints(
    const std::vector<charmie_interfaces::msg::Keypoints_<ContainerAllocator>, typename ContainerAllocator::template rebind<charmie_interfaces::msg::Keypoints_<ContainerAllocator>>::other> & _arg)
  {
    this->keypoints = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__Yolov8Pose
    std::shared_ptr<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__Yolov8Pose
    std::shared_ptr<charmie_interfaces::msg::Yolov8Pose_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Yolov8Pose_ & other) const
  {
    if (this->num_person != other.num_person) {
      return false;
    }
    if (this->keypoints != other.keypoints) {
      return false;
    }
    return true;
  }
  bool operator!=(const Yolov8Pose_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Yolov8Pose_

// alias to use template instance with default allocator
using Yolov8Pose =
  charmie_interfaces::msg::Yolov8Pose_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__YOLOV8_POSE__STRUCT_HPP_
