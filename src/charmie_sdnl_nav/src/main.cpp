#include "rclcpp/rclcpp.hpp"
#include "charmie_sdnl_nav/sdnl_nav_node.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SDNLNavNode>());
  rclcpp::shutdown();
  return 0;
}
