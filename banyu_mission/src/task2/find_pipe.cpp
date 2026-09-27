#include "banyu_mission/task2/find_pipe.hpp"

#include <algorithm>

namespace banyu_mission::task2 {

namespace {
constexpr int kSeenFrames = 5;
constexpr double kTimeoutS = 240.0;
}  // namespace

BT::NodeStatus FindPipe::begin() {
  seen_frames_ = 0;
  ping_.reset();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FindPipe::step(const Snapshot& s) {
  if (newFrame(s)) {
    seen_frames_ = s.pipe.ahead.valid ? seen_frames_ + 1 : 0;
    if (seen_frames_ >= kSeenFrames) return BT::NodeStatus::SUCCESS;
  }
  if (elapsed() > kTimeoutS) {
    RCLCPP_ERROR(logger(), "pipeline not found");
    return BT::NodeStatus::FAILURE;
  }
  if (!s.altitude) {
    sendSticks(*ctx_, 0, 0, 0.2);  // no echo yet: keep going down gently
    return BT::NodeStatus::RUNNING;
  }
  const double search = param("task2_search_altitude");
  const double down = std::clamp(0.5 * (*s.altitude - search), -0.5, 0.5);
  if (*s.altitude > search + 0.5) {
    sendSticks(*ctx_, 0, 0, down);  // still descending
    return BT::NodeStatus::RUNNING;
  }
  if (auto p = takePing(*ctx_)) ping_ = p;
  // Face the pinger, sway round it, surge holds the radius; widen ~0.5 m per minute.
  const double radius = std::min(3.0, 1.0 + elapsed() / 120.0);
  const double fwd = ping_ ? std::clamp(0.3 * (ping_->horizontal_range - radius), -0.15, 0.15) : 0.0;
  const double bearing = ping_ ? ping_->bearing : 0.0;
  sendSticks(*ctx_, fwd, 0.15, down, std::clamp(-0.8 * bearing, -0.3, 0.3));
  return BT::NodeStatus::RUNNING;
}

}  // namespace banyu_mission::task2
