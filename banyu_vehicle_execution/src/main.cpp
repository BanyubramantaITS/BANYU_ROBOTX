#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "banyu_vehicle_execution/vehicle_execution_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<banyu_vehicle_execution::VehicleExecutionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
