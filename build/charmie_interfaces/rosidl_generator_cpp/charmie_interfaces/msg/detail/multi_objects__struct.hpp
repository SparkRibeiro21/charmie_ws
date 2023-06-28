// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/MultiObjects.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__MULTI_OBJECTS__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__MULTI_OBJECTS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__MultiObjects __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__MultiObjects __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MultiObjects_
{
  using Type = MultiObjects_<ContainerAllocator>;

  explicit MultiObjects_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit MultiObjects_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _objects_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _objects_type objects;
  using _confidence_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _confidence_type confidence;

  // setters for named parameter idiom
  Type & set__objects(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->objects = _arg;
    return *this;
  }
  Type & set__confidence(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->confidence = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::MultiObjects_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::MultiObjects_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::MultiObjects_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::MultiObjects_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::MultiObjects_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::MultiObjects_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::MultiObjects_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::MultiObjects_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::MultiObjects_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::MultiObjects_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__MultiObjects
    std::shared_ptr<charmie_interfaces::msg::MultiObjects_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__MultiObjects
    std::shared_ptr<charmie_interfaces::msg::MultiObjects_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiObjects_ & other) const
  {
    if (this->objects != other.objects) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiObjects_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiObjects_

// alias to use template instance with default allocator
using MultiObjects =
  charmie_interfaces::msg::MultiObjects_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__MULTI_OBJECTS__STRUCT_HPP_
