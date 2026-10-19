#include "banyu_teleop/banyu_teleop.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<banyu_teleop::BanyuTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
