#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "banyu_vehicle_interface/esp32_bridge_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_vehicle_interface::Esp32BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
