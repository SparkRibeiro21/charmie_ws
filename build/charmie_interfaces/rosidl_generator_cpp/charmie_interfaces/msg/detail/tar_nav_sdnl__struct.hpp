// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/TarNavSDNL.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'move_target_coordinates'
// Member 'rotate_target_coordinates'
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__TarNavSDNL __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__TarNavSDNL __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TarNavSDNL_
{
  using Type = TarNavSDNL_<ContainerAllocator>;

  explicit TarNavSDNL_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : move_target_coordinates(_init),
    rotate_target_coordinates(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flag_not_obs = false;
      this->follow_me = false;
    }
  }

  explicit TarNavSDNL_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : move_target_coordinates(_alloc, _init),
    rotate_target_coordinates(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->flag_not_obs = false;
      this->follow_me = false;
    }
  }

  // field types and members
  using _move_target_coordinates_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _move_target_coordinates_type move_target_coordinates;
  using _rotate_target_coordinates_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _rotate_target_coordinates_type rotate_target_coordinates;
  using _flag_not_obs_type =
    bool;
  _flag_not_obs_type flag_not_obs;
  using _follow_me_type =
    bool;
  _follow_me_type follow_me;

  // setters for named parameter idiom
  Type & set__move_target_coordinates(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->move_target_coordinates = _arg;
    return *this;
  }
  Type & set__rotate_target_coordinates(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->rotate_target_coordinates = _arg;
    return *this;
  }
  Type & set__flag_not_obs(
    const bool & _arg)
  {
    this->flag_not_obs = _arg;
    return *this;
  }
  Type & set__follow_me(
    const bool & _arg)
  {
    this->follow_me = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__TarNavSDNL
    std::shared_ptr<charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__TarNavSDNL
    std::shared_ptr<charmie_interfaces::msg::TarNavSDNL_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TarNavSDNL_ & other) const
  {
    if (this->move_target_coordinates != other.move_target_coordinates) {
      return false;
    }
    if (this->rotate_target_coordinates != other.rotate_target_coordinates) {
      return false;
    }
    if (this->flag_not_obs != other.flag_not_obs) {
      return false;
    }
    if (this->follow_me != other.follow_me) {
      return false;
    }
    return true;
  }
  bool operator!=(const TarNavSDNL_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TarNavSDNL_

// alias to use template instance with default allocator
using TarNavSDNL =
  charmie_interfaces::msg::TarNavSDNL_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__TAR_NAV_SDNL__STRUCT_HPP_
