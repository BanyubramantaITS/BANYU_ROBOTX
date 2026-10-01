#include "banyu_mission/task2/send_resource_request.hpp"

namespace banyu_mission::task2 {

BT::NodeStatus SendResourceRequest::tick() {
  banyu_msgs::msg::ResourceDeliveryRequest req;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    if (!ctx_->delivery_circle_colour) {
      RCLCPP_WARN(node_->get_logger(), "SendResourceRequest: no flash colour read, nothing to request");
      return BT::NodeStatus::FAILURE;
    }
    req.delivery_circle_color = *ctx_->delivery_circle_colour;
    req.resource_color = ctx_->resource_colour.value_or(req.COLOR_ANY);
  }
  req.header.stamp = node_->now();
  req.task = req.TASK_INFRA_SURVEY_REPAIR;
  ctx_->resource_pub->publish(req);
  RCLCPP_INFO(node_->get_logger(), "RESOURCE REQUEST: tin colour %u (4 = any) to the circle of colour %u",
              req.resource_color, req.delivery_circle_color);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace banyu_mission::task2
