#include "banyu_mission/task2/turn_on_pipe.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission::task2 {

BT::NodeStatus TurnOnPipe::begin() {
  const Snapshot s = snapshot(*ctx_);
  heading_ = getInput<bool>("reverse").value_or(false) ? wrapAngle(s.yaw + M_PI)
                                                       : s.yaw - (s.pipe.ahead.valid ? s.pipe.ahead.angle : 0.0);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnOnPipe::step(const Snapshot& s) {
  const double err = wrapAngle(heading_ - s.yaw);
  const auto pull = pullOverPipe(s.pipe).value_or(std::make_pair(0.0, 0.0));
  sendSticks(*ctx_, pull.first, pull.second, depthRate(s.depth_target, s.depth), std::clamp(-0.8 * err, -0.3, 0.3));
  const bool squared = std::fabs(err) < 0.15 && s.pipe.ahead.valid && std::fabs(s.pipe.ahead.angle) < 0.3;
  if (squared || elapsed() > 40.0) {
    RCLCPP_INFO(logger(), "turn to %.0f deg: err %.0f deg", heading_ * 180 / M_PI, err * 180 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace banyu_mission::task2
