#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "banyu_vehicle_interface/esp32_trigger_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<banyu_vehicle_interface::Esp32TriggerNode>();
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Exception during execution: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
