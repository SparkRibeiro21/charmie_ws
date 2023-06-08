// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from charmie_interfaces:msg/PS4Controller.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__PS4_CONTROLLER__STRUCT_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__PS4_CONTROLLER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__charmie_interfaces__msg__PS4Controller __attribute__((deprecated))
#else
# define DEPRECATED__charmie_interfaces__msg__PS4Controller __declspec(deprecated)
#endif

namespace charmie_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PS4Controller_
{
  using Type = PS4Controller_<ContainerAllocator>;

  explicit PS4Controller_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->triangle = 0;
      this->circle = 0;
      this->cross = 0;
      this->square = 0;
      this->arrow_up = 0;
      this->arrow_right = 0;
      this->arrow_down = 0;
      this->arrow_left = 0;
      this->l1 = 0;
      this->r1 = 0;
      this->l3 = 0;
      this->r3 = 0;
      this->share = 0;
      this->options = 0;
      this->ps = 0;
      this->l3_ang = 0.0f;
      this->l3_dist = 0.0f;
      this->l3_xx = 0.0f;
      this->l3_yy = 0.0f;
      this->r3_ang = 0.0f;
      this->r3_dist = 0.0f;
      this->r3_xx = 0.0f;
      this->r3_yy = 0.0f;
      this->l2 = 0.0f;
      this->r2 = 0.0f;
    }
  }

  explicit PS4Controller_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->triangle = 0;
      this->circle = 0;
      this->cross = 0;
      this->square = 0;
      this->arrow_up = 0;
      this->arrow_right = 0;
      this->arrow_down = 0;
      this->arrow_left = 0;
      this->l1 = 0;
      this->r1 = 0;
      this->l3 = 0;
      this->r3 = 0;
      this->share = 0;
      this->options = 0;
      this->ps = 0;
      this->l3_ang = 0.0f;
      this->l3_dist = 0.0f;
      this->l3_xx = 0.0f;
      this->l3_yy = 0.0f;
      this->r3_ang = 0.0f;
      this->r3_dist = 0.0f;
      this->r3_xx = 0.0f;
      this->r3_yy = 0.0f;
      this->l2 = 0.0f;
      this->r2 = 0.0f;
    }
  }

  // field types and members
  using _triangle_type =
    uint8_t;
  _triangle_type triangle;
  using _circle_type =
    uint8_t;
  _circle_type circle;
  using _cross_type =
    uint8_t;
  _cross_type cross;
  using _square_type =
    uint8_t;
  _square_type square;
  using _arrow_up_type =
    uint8_t;
  _arrow_up_type arrow_up;
  using _arrow_right_type =
    uint8_t;
  _arrow_right_type arrow_right;
  using _arrow_down_type =
    uint8_t;
  _arrow_down_type arrow_down;
  using _arrow_left_type =
    uint8_t;
  _arrow_left_type arrow_left;
  using _l1_type =
    uint8_t;
  _l1_type l1;
  using _r1_type =
    uint8_t;
  _r1_type r1;
  using _l3_type =
    uint8_t;
  _l3_type l3;
  using _r3_type =
    uint8_t;
  _r3_type r3;
  using _share_type =
    uint8_t;
  _share_type share;
  using _options_type =
    uint8_t;
  _options_type options;
  using _ps_type =
    uint8_t;
  _ps_type ps;
  using _l3_ang_type =
    float;
  _l3_ang_type l3_ang;
  using _l3_dist_type =
    float;
  _l3_dist_type l3_dist;
  using _l3_xx_type =
    float;
  _l3_xx_type l3_xx;
  using _l3_yy_type =
    float;
  _l3_yy_type l3_yy;
  using _r3_ang_type =
    float;
  _r3_ang_type r3_ang;
  using _r3_dist_type =
    float;
  _r3_dist_type r3_dist;
  using _r3_xx_type =
    float;
  _r3_xx_type r3_xx;
  using _r3_yy_type =
    float;
  _r3_yy_type r3_yy;
  using _l2_type =
    float;
  _l2_type l2;
  using _r2_type =
    float;
  _r2_type r2;

  // setters for named parameter idiom
  Type & set__triangle(
    const uint8_t & _arg)
  {
    this->triangle = _arg;
    return *this;
  }
  Type & set__circle(
    const uint8_t & _arg)
  {
    this->circle = _arg;
    return *this;
  }
  Type & set__cross(
    const uint8_t & _arg)
  {
    this->cross = _arg;
    return *this;
  }
  Type & set__square(
    const uint8_t & _arg)
  {
    this->square = _arg;
    return *this;
  }
  Type & set__arrow_up(
    const uint8_t & _arg)
  {
    this->arrow_up = _arg;
    return *this;
  }
  Type & set__arrow_right(
    const uint8_t & _arg)
  {
    this->arrow_right = _arg;
    return *this;
  }
  Type & set__arrow_down(
    const uint8_t & _arg)
  {
    this->arrow_down = _arg;
    return *this;
  }
  Type & set__arrow_left(
    const uint8_t & _arg)
  {
    this->arrow_left = _arg;
    return *this;
  }
  Type & set__l1(
    const uint8_t & _arg)
  {
    this->l1 = _arg;
    return *this;
  }
  Type & set__r1(
    const uint8_t & _arg)
  {
    this->r1 = _arg;
    return *this;
  }
  Type & set__l3(
    const uint8_t & _arg)
  {
    this->l3 = _arg;
    return *this;
  }
  Type & set__r3(
    const uint8_t & _arg)
  {
    this->r3 = _arg;
    return *this;
  }
  Type & set__share(
    const uint8_t & _arg)
  {
    this->share = _arg;
    return *this;
  }
  Type & set__options(
    const uint8_t & _arg)
  {
    this->options = _arg;
    return *this;
  }
  Type & set__ps(
    const uint8_t & _arg)
  {
    this->ps = _arg;
    return *this;
  }
  Type & set__l3_ang(
    const float & _arg)
  {
    this->l3_ang = _arg;
    return *this;
  }
  Type & set__l3_dist(
    const float & _arg)
  {
    this->l3_dist = _arg;
    return *this;
  }
  Type & set__l3_xx(
    const float & _arg)
  {
    this->l3_xx = _arg;
    return *this;
  }
  Type & set__l3_yy(
    const float & _arg)
  {
    this->l3_yy = _arg;
    return *this;
  }
  Type & set__r3_ang(
    const float & _arg)
  {
    this->r3_ang = _arg;
    return *this;
  }
  Type & set__r3_dist(
    const float & _arg)
  {
    this->r3_dist = _arg;
    return *this;
  }
  Type & set__r3_xx(
    const float & _arg)
  {
    this->r3_xx = _arg;
    return *this;
  }
  Type & set__r3_yy(
    const float & _arg)
  {
    this->r3_yy = _arg;
    return *this;
  }
  Type & set__l2(
    const float & _arg)
  {
    this->l2 = _arg;
    return *this;
  }
  Type & set__r2(
    const float & _arg)
  {
    this->r2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    charmie_interfaces::msg::PS4Controller_<ContainerAllocator> *;
  using ConstRawPtr =
    const charmie_interfaces::msg::PS4Controller_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<charmie_interfaces::msg::PS4Controller_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<charmie_interfaces::msg::PS4Controller_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::PS4Controller_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::PS4Controller_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      charmie_interfaces::msg::PS4Controller_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<charmie_interfaces::msg::PS4Controller_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<charmie_interfaces::msg::PS4Controller_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<charmie_interfaces::msg::PS4Controller_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__charmie_interfaces__msg__PS4Controller
    std::shared_ptr<charmie_interfaces::msg::PS4Controller_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__charmie_interfaces__msg__PS4Controller
    std::shared_ptr<charmie_interfaces::msg::PS4Controller_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PS4Controller_ & other) const
  {
    if (this->triangle != other.triangle) {
      return false;
    }
    if (this->circle != other.circle) {
      return false;
    }
    if (this->cross != other.cross) {
      return false;
    }
    if (this->square != other.square) {
      return false;
    }
    if (this->arrow_up != other.arrow_up) {
      return false;
    }
    if (this->arrow_right != other.arrow_right) {
      return false;
    }
    if (this->arrow_down != other.arrow_down) {
      return false;
    }
    if (this->arrow_left != other.arrow_left) {
      return false;
    }
    if (this->l1 != other.l1) {
      return false;
    }
    if (this->r1 != other.r1) {
      return false;
    }
    if (this->l3 != other.l3) {
      return false;
    }
    if (this->r3 != other.r3) {
      return false;
    }
    if (this->share != other.share) {
      return false;
    }
    if (this->options != other.options) {
      return false;
    }
    if (this->ps != other.ps) {
      return false;
    }
    if (this->l3_ang != other.l3_ang) {
      return false;
    }
    if (this->l3_dist != other.l3_dist) {
      return false;
    }
    if (this->l3_xx != other.l3_xx) {
      return false;
    }
    if (this->l3_yy != other.l3_yy) {
      return false;
    }
    if (this->r3_ang != other.r3_ang) {
      return false;
    }
    if (this->r3_dist != other.r3_dist) {
      return false;
    }
    if (this->r3_xx != other.r3_xx) {
      return false;
    }
    if (this->r3_yy != other.r3_yy) {
      return false;
    }
    if (this->l2 != other.l2) {
      return false;
    }
    if (this->r2 != other.r2) {
      return false;
    }
    return true;
  }
  bool operator!=(const PS4Controller_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PS4Controller_

// alias to use template instance with default allocator
using PS4Controller =
  charmie_interfaces::msg::PS4Controller_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__PS4_CONTROLLER__STRUCT_HPP_
