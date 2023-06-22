// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/Obstacles.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLES__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'obstacles'
#include "charmie_interfaces/msg/detail/obstacle_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__Obstacles __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__Obstacles __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Obstacles_
{
  using Type = Obstacles_<ContainerAllocator>;

  explicit Obstacles_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->no_obstacles = 0;
    }
  }

  explicit Obstacles_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->no_obstacles = 0;
    }
  }

  // field types and members
  using _no_obstacles_type =
    uint16_t;
  _no_obstacles_type no_obstacles;
  using _obstacles_type =
    std::vector<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>, typename ContainerAllocator::template rebind<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>>::other>;
  _obstacles_type obstacles;

  // setters for named parameter idiom
  Type & set__no_obstacles(
    const uint16_t & _arg)
  {
    this->no_obstacles = _arg;
    return *this;
  }
  Type & set__obstacles(
    const std::vector<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>, typename ContainerAllocator::template rebind<charmie_interfaces::msg::ObstacleInfo_<ContainerAllocator>>::other> & _arg)
  {
    this->obstacles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::Obstacles_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::Obstacles_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Obstacles_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::Obstacles_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Obstacles_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Obstacles_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::Obstacles_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::Obstacles_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Obstacles_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::Obstacles_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__Obstacles
    std::shared_ptr<charmie_interfaces::msg::Obstacles_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__Obstacles
    std::shared_ptr<charmie_interfaces::msg::Obstacles_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Obstacles_ & other) const
  {
    if (this->no_obstacles != other.no_obstacles) {
      return false;
    }
    if (this->obstacles != other.obstacles) {
      return false;
    }
    return true;
  }
  bool operator!=(const Obstacles_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Obstacles_

// alias to use template instance with default allocator
using Obstacles =
  charmie_interfaces::msg::Obstacles_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__OBSTACLES__STRUCT_HPP_
