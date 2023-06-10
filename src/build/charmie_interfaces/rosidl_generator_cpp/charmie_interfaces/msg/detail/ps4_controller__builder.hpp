// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from charmie_interfaces:msg/PS4Controller.idl
// generated code does not contain a copyright notice

#ifndef CHARMIE_INTERFACES__MSG__DETAIL__PS4_CONTROLLER__BUILDER_HPP_
#define CHARMIE_INTERFACES__MSG__DETAIL__PS4_CONTROLLER__BUILDER_HPP_

#include "charmie_interfaces/msg/detail/ps4_controller__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace charmie_interfaces
{

namespace msg
{

namespace builder
{

class Init_PS4Controller_r2
{
public:
  explicit Init_PS4Controller_r2(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  ::charmie_interfaces::msg::PS4Controller r2(::charmie_interfaces::msg::PS4Controller::_r2_type arg)
  {
    msg_.r2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_l2
{
public:
  explicit Init_PS4Controller_l2(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_r2 l2(::charmie_interfaces::msg::PS4Controller::_l2_type arg)
  {
    msg_.l2 = std::move(arg);
    return Init_PS4Controller_r2(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_r3_yy
{
public:
  explicit Init_PS4Controller_r3_yy(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_l2 r3_yy(::charmie_interfaces::msg::PS4Controller::_r3_yy_type arg)
  {
    msg_.r3_yy = std::move(arg);
    return Init_PS4Controller_l2(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_r3_xx
{
public:
  explicit Init_PS4Controller_r3_xx(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_r3_yy r3_xx(::charmie_interfaces::msg::PS4Controller::_r3_xx_type arg)
  {
    msg_.r3_xx = std::move(arg);
    return Init_PS4Controller_r3_yy(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_r3_dist
{
public:
  explicit Init_PS4Controller_r3_dist(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_r3_xx r3_dist(::charmie_interfaces::msg::PS4Controller::_r3_dist_type arg)
  {
    msg_.r3_dist = std::move(arg);
    return Init_PS4Controller_r3_xx(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_r3_ang
{
public:
  explicit Init_PS4Controller_r3_ang(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_r3_dist r3_ang(::charmie_interfaces::msg::PS4Controller::_r3_ang_type arg)
  {
    msg_.r3_ang = std::move(arg);
    return Init_PS4Controller_r3_dist(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_l3_yy
{
public:
  explicit Init_PS4Controller_l3_yy(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_r3_ang l3_yy(::charmie_interfaces::msg::PS4Controller::_l3_yy_type arg)
  {
    msg_.l3_yy = std::move(arg);
    return Init_PS4Controller_r3_ang(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_l3_xx
{
public:
  explicit Init_PS4Controller_l3_xx(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_l3_yy l3_xx(::charmie_interfaces::msg::PS4Controller::_l3_xx_type arg)
  {
    msg_.l3_xx = std::move(arg);
    return Init_PS4Controller_l3_yy(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_l3_dist
{
public:
  explicit Init_PS4Controller_l3_dist(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_l3_xx l3_dist(::charmie_interfaces::msg::PS4Controller::_l3_dist_type arg)
  {
    msg_.l3_dist = std::move(arg);
    return Init_PS4Controller_l3_xx(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_l3_ang
{
public:
  explicit Init_PS4Controller_l3_ang(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_l3_dist l3_ang(::charmie_interfaces::msg::PS4Controller::_l3_ang_type arg)
  {
    msg_.l3_ang = std::move(arg);
    return Init_PS4Controller_l3_dist(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_ps
{
public:
  explicit Init_PS4Controller_ps(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_l3_ang ps(::charmie_interfaces::msg::PS4Controller::_ps_type arg)
  {
    msg_.ps = std::move(arg);
    return Init_PS4Controller_l3_ang(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_options
{
public:
  explicit Init_PS4Controller_options(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_ps options(::charmie_interfaces::msg::PS4Controller::_options_type arg)
  {
    msg_.options = std::move(arg);
    return Init_PS4Controller_ps(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_share
{
public:
  explicit Init_PS4Controller_share(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_options share(::charmie_interfaces::msg::PS4Controller::_share_type arg)
  {
    msg_.share = std::move(arg);
    return Init_PS4Controller_options(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_r3
{
public:
  explicit Init_PS4Controller_r3(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_share r3(::charmie_interfaces::msg::PS4Controller::_r3_type arg)
  {
    msg_.r3 = std::move(arg);
    return Init_PS4Controller_share(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_l3
{
public:
  explicit Init_PS4Controller_l3(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_r3 l3(::charmie_interfaces::msg::PS4Controller::_l3_type arg)
  {
    msg_.l3 = std::move(arg);
    return Init_PS4Controller_r3(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_r1
{
public:
  explicit Init_PS4Controller_r1(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_l3 r1(::charmie_interfaces::msg::PS4Controller::_r1_type arg)
  {
    msg_.r1 = std::move(arg);
    return Init_PS4Controller_l3(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_l1
{
public:
  explicit Init_PS4Controller_l1(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_r1 l1(::charmie_interfaces::msg::PS4Controller::_l1_type arg)
  {
    msg_.l1 = std::move(arg);
    return Init_PS4Controller_r1(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_arrow_left
{
public:
  explicit Init_PS4Controller_arrow_left(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_l1 arrow_left(::charmie_interfaces::msg::PS4Controller::_arrow_left_type arg)
  {
    msg_.arrow_left = std::move(arg);
    return Init_PS4Controller_l1(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_arrow_down
{
public:
  explicit Init_PS4Controller_arrow_down(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_arrow_left arrow_down(::charmie_interfaces::msg::PS4Controller::_arrow_down_type arg)
  {
    msg_.arrow_down = std::move(arg);
    return Init_PS4Controller_arrow_left(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_arrow_right
{
public:
  explicit Init_PS4Controller_arrow_right(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_arrow_down arrow_right(::charmie_interfaces::msg::PS4Controller::_arrow_right_type arg)
  {
    msg_.arrow_right = std::move(arg);
    return Init_PS4Controller_arrow_down(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_arrow_up
{
public:
  explicit Init_PS4Controller_arrow_up(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_arrow_right arrow_up(::charmie_interfaces::msg::PS4Controller::_arrow_up_type arg)
  {
    msg_.arrow_up = std::move(arg);
    return Init_PS4Controller_arrow_right(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_square
{
public:
  explicit Init_PS4Controller_square(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_arrow_up square(::charmie_interfaces::msg::PS4Controller::_square_type arg)
  {
    msg_.square = std::move(arg);
    return Init_PS4Controller_arrow_up(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_cross
{
public:
  explicit Init_PS4Controller_cross(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_square cross(::charmie_interfaces::msg::PS4Controller::_cross_type arg)
  {
    msg_.cross = std::move(arg);
    return Init_PS4Controller_square(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_circle
{
public:
  explicit Init_PS4Controller_circle(::charmie_interfaces::msg::PS4Controller & msg)
  : msg_(msg)
  {}
  Init_PS4Controller_cross circle(::charmie_interfaces::msg::PS4Controller::_circle_type arg)
  {
    msg_.circle = std::move(arg);
    return Init_PS4Controller_cross(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

class Init_PS4Controller_triangle
{
public:
  Init_PS4Controller_triangle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PS4Controller_circle triangle(::charmie_interfaces::msg::PS4Controller::_triangle_type arg)
  {
    msg_.triangle = std::move(arg);
    return Init_PS4Controller_circle(msg_);
  }

private:
  ::charmie_interfaces::msg::PS4Controller msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::charmie_interfaces::msg::PS4Controller>()
{
  return charmie_interfaces::msg::builder::Init_PS4Controller_triangle();
}

}  // namespace charmie_interfaces

#endif  // CHARMIE_INTERFACES__MSG__DETAIL__PS4_CONTROLLER__BUILDER_HPP_
