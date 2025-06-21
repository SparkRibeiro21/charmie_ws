
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "charmie_gamepad/charmie_gamepad.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<charmie_gamepad::CHARMIEGamepad>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}