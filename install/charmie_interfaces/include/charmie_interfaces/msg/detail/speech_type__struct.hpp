// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/SpeechType.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__SPEECH_TYPE__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__SPEECH_TYPE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__SpeechType __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__SpeechType __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SpeechType_
{
  using Type = SpeechType_<ContainerAllocator>;

  explicit SpeechType_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yes_or_no = false;
      this->receptionist = false;
      this->gpsr = false;
      this->restaurant = false;
    }
  }

  explicit SpeechType_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yes_or_no = false;
      this->receptionist = false;
      this->gpsr = false;
      this->restaurant = false;
    }
  }

  // field types and members
  using _yes_or_no_type =
    bool;
  _yes_or_no_type yes_or_no;
  using _receptionist_type =
    bool;
  _receptionist_type receptionist;
  using _gpsr_type =
    bool;
  _gpsr_type gpsr;
  using _restaurant_type =
    bool;
  _restaurant_type restaurant;

  // setters for named parameter idiom
  Type & set__yes_or_no(
    const bool & _arg)
  {
    this->yes_or_no = _arg;
    return *this;
  }
  Type & set__receptionist(
    const bool & _arg)
  {
    this->receptionist = _arg;
    return *this;
  }
  Type & set__gpsr(
    const bool & _arg)
  {
    this->gpsr = _arg;
    return *this;
  }
  Type & set__restaurant(
    const bool & _arg)
  {
    this->restaurant = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::SpeechType_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::SpeechType_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::SpeechType_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::SpeechType_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::SpeechType_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::SpeechType_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::SpeechType_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::SpeechType_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::SpeechType_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::SpeechType_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__SpeechType
    std::shared_ptr<charmie_interfaces::msg::SpeechType_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__SpeechType
    std::shared_ptr<charmie_interfaces::msg::SpeechType_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpeechType_ & other) const
  {
    if (this->yes_or_no != other.yes_or_no) {
      return false;
    }
    if (this->receptionist != other.receptionist) {
      return false;
    }
    if (this->gpsr != other.gpsr) {
      return false;
    }
    if (this->restaurant != other.restaurant) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpeechType_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpeechType_

// alias to use template instance with default allocator
using SpeechType =
  charmie_interfaces::msg::SpeechType_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__SPEECH_TYPE__STRUCT_HPP_
