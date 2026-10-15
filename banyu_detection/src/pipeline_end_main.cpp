#include "banyu_detection/pipeline_end_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_detection::PipelineEndNode>());
  rclcpp::shutdown();
  return 0;
}
