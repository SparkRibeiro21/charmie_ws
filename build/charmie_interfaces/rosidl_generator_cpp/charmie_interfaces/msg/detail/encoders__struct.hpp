// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/Encoders.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__Encoders __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__Encoders __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Encoders_
{
  using Type = Encoders_<ContainerAllocator>;

  explicit Encoders_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enc_m1 = 0l;
      this->enc_m2 = 0l;
      this->enc_m3 = 0l;
      this->enc_m4 = 0l;
    }
  }

  explicit Encoders_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->enc_m1 = 0l;
      this->enc_m2 = 0l;
      this->enc_m3 = 0l;
      this->enc_m4 = 0l;
    }
  }

  // field types and members
  using _enc_m1_type =
    int32_t;
  _enc_m1_type enc_m1;
  using _enc_m2_type =
    int32_t;
  _enc_m2_type enc_m2;
  using _enc_m3_type =
    int32_t;
  _enc_m3_type enc_m3;
  using _enc_m4_type =
    int32_t;
  _enc_m4_type enc_m4;

  // setters for named parameter idiom
  Type & set__enc_m1(
    const int32_t & _arg)
  {
    this->enc_m1 = _arg;
    return *this;
  }
  Type & set__enc_m2(
    const int32_t & _arg)
  {
    this->enc_m2 = _arg;
    return *this;
  }
  Type & set__enc_m3(
    const int32_t & _arg)
  {
    this->enc_m3 = _arg;
    return *this;
  }
  Type & set__enc_m4(
    const int32_t & _arg)
  {
    this->enc_m4 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::Encoders_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::Encoders_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Encoders_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Encoders_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Encoders_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Encoders_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Encoders_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Encoders_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Encoders_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Encoders_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__Encoders
    std::shared_ptr<charmie_interfaces::msg::Encoders_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__Encoders
    std::shared_ptr<charmie_interfaces::msg::Encoders_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Encoders_ & other) const
  {
    if (this->enc_m1 != other.enc_m1) {
      return false;
    }
    if (this->enc_m2 != other.enc_m2) {
      return false;
    }
    if (this->enc_m3 != other.enc_m3) {
      return false;
    }
    if (this->enc_m4 != other.enc_m4) {
      return false;
    }
    return true;
  }
  bool operator!=(const Encoders_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Encoders_

// alias to use template instance with default allocator
using Encoders =
  charmie_interfaces::msg::Encoders_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__ENCODERS__STRUCT_HPP_
