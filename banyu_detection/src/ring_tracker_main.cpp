#include "banyu_detection/ring_tracker_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_detection::RingTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
