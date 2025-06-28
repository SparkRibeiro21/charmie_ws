#ifndef CHARMIE_GAMEPAD__CHARMIE_GAMEPAD_HPP_
#define CHARMIE_GAMEPAD__CHARMIE_GAMEPAD_HPP_

#include <rclcpp/rclcpp.hpp>

namespace charmie_gamepad
{

/**
 * Class implementing a basic Joy -> Twist translation.
 */
class CHARMIEGamepad : public rclcpp::Node
{
public:
  explicit CHARMIEGamepad(const rclcpp::NodeOptions & options);
  virtual ~CHARMIEGamepad();

private:
  struct Impl;
  Impl * pimpl_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle;
};

}  // namespace charmie_gamepad

#endif  // CHARMIE_GAMEPAD__CHARMIE_GAMEPAD_HPP_