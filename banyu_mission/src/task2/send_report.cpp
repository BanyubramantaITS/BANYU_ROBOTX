#include "banyu_mission/task2/send_report.hpp"

#include <string>

namespace banyu_mission::task2 {

BT::NodeStatus SendReport::tick() {
  banyu_msgs::msg::PipelineSurveyReport report;
  {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    for (const auto& t : ctx_->tracker.tracks())
      RCLCPP_INFO(node_->get_logger(), "track: n=%d red=%d green=%d", t.n, t.red_votes, t.green_votes);
    for (const uint8_t colour : ctx_->tracker.nodes())
      report.segment_states.push_back(colour == banyu_msgs::msg::LightNode::RED
                                          ? banyu_msgs::msg::PipelineSurveyReport::SEGMENT_DAMAGED
                                          : banyu_msgs::msg::PipelineSurveyReport::SEGMENT_INTACT);
    report.active_buoy_latitude = ctx_->buoy_latitude;
    report.active_buoy_longitude = ctx_->buoy_longitude;
  }
  if (report.segment_states.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "SendReport: nothing surveyed, no report sent");
    return BT::NodeStatus::FAILURE;
  }
  report.header.stamp = node_->now();
  ctx_->report_pub->publish(report);

  std::string text;
  for (const uint8_t s : report.segment_states)
    text += s == banyu_msgs::msg::PipelineSurveyReport::SEGMENT_DAMAGED ? " RED" : " GREEN";
  RCLCPP_INFO(node_->get_logger(), "REPORT from the active buoy end:%s", text.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace banyu_mission::task2
