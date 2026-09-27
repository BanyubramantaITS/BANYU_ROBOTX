#include "banyu_mission/task2/record_buoy_fix.hpp"

#include <limits>

namespace banyu_mission::task2 {

BT::NodeStatus RecordBuoyFix::step(const Snapshot&) {
  sendSticks(*ctx_, 0, 0, 0);
  if (elapsed() < 2.0) return BT::NodeStatus::RUNNING;

  std::lock_guard<std::mutex> lock(ctx_->data_mutex);
  if (ctx_->fix && ctx_->fix->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX) {
    ctx_->buoy_latitude = ctx_->fix->latitude;
    ctx_->buoy_longitude = ctx_->fix->longitude;
    RCLCPP_INFO(logger(), "active buoy GPS fix: %.7f, %.7f", ctx_->buoy_latitude, ctx_->buoy_longitude);
  } else {
    ctx_->buoy_latitude = ctx_->buoy_longitude = std::numeric_limits<double>::quiet_NaN();
    RCLCPP_WARN(logger(), "no GPS fix at the active buoy; reporting its position as unknown");
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace banyu_mission::task2
