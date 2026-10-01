#include "banyu_mission/task2/transit_to_pinger.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission::task2 {

namespace {
constexpr double kCloseElevationCos = 0.985;  // ~10 deg elevation
constexpr double kSlowElevationCos = 0.995;   // ~6 deg: slow in near the hull
constexpr double kTimeoutS = 240.0;
}  // namespace

BT::NodeStatus TransitToPinger::begin() {
  drainPings(*ctx_);
  last_.reset();
  close_pings_ = 0;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TransitToPinger::step(const Snapshot&) {
  if (auto ping = takePing(*ctx_)) {
    last_ = ping;
    close_pings_ = ping->elevation_cos < kCloseElevationCos ? close_pings_ + 1 : 0;
    RCLCPP_INFO(logger(), "transit: bearing %+.0f deg, elevation %.0f deg", ping->bearing * 180 / M_PI,
                std::acos(std::min<double>(ping->elevation_cos, 1.0)) * 180 / M_PI);
    if (close_pings_ >= 2) return BT::NodeStatus::SUCCESS;
  }
  if (last_) {
    const double speed = last_->elevation_cos > kSlowElevationCos ? 0.6 : 0.15;
    sendSticks(*ctx_, speed * std::max(0.0, std::cos(static_cast<double>(last_->bearing))), 0, 0.0,
               std::clamp(-0.8 * last_->bearing, -0.4, 0.4));
  } else {
    sendSticks(*ctx_, 0, 0, 0.0);
    if (elapsed() > 5.0) RCLCPP_WARN_THROTTLE(logger(), *node_->get_clock(), 5000, "transit: no pinger heard yet");
  }
  if (elapsed() > kTimeoutS) {
    RCLCPP_ERROR(logger(), "surface transit to the active buoy timed out");
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace banyu_mission::task2
