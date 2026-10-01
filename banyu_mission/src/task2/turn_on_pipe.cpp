#include "banyu_mission/task2/turn_on_pipe.hpp"

#include <algorithm>
#include <cmath>

namespace banyu_mission::task2 {

namespace {
constexpr double kSettleS = 1.0;
}  // namespace

BT::NodeStatus TurnOnPipe::begin() {
  const Snapshot s = snapshot(*ctx_);
  reverse_ = getInput<bool>("reverse").value_or(false);
  heading_ = reverse_ ? wrapAngle(s.yaw + M_PI) : s.yaw - (s.pipe.ahead.valid ? s.pipe.ahead.angle : 0.0);
  on_heading_since_ = -1.0;
  reaimed_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TurnOnPipe::step(const Snapshot& s) {
  const double err = wrapAngle(heading_ - s.yaw);
  const auto pull = pullOverPipe(s.pipe).value_or(std::make_pair(0.0, 0.0));
  sendSticks(*ctx_, pull.first, pull.second, depthRate(s.depth_target, s.depth), std::clamp(-0.8 * err, -0.3, 0.3));
  // Frames lag the compass: judge the view only once the heading has held for a while.
  if (std::fabs(err) >= 0.15)
    on_heading_since_ = -1.0;
  else if (on_heading_since_ < 0)
    on_heading_since_ = elapsed();
  const bool settled = on_heading_since_ >= 0 && elapsed() - on_heading_since_ > kSettleS;
  // Near an end the pipe may only show on one side of the vehicle: either band will do. With no
  // fit in either, turning on will not find one: the heading is all there is to go on.
  const auto& fit = s.pipe.ahead.valid ? s.pipe.ahead : s.pipe.behind;
  // Reversing, the heading was squared on the way in; after one re-aim, likewise. Near an end the
  // fit can lock onto other members of the structure, so from then on the heading alone decides.
  const bool squared = settled && (reverse_ || reaimed_ || !fit.valid || std::fabs(fit.angle) < 0.3);
  // On the heading but the pipe still crosses the image: the heading came from a frame taken
  // while the vehicle was still turning. Re-aim, once pulled over the pipe: turning off-centre
  // swings it out of view, and with no fit in view the turn counts as done.
  const bool centred = s.pipe.pipe_in_view &&
                       std::fabs(s.pipe.centroid_x - s.pipe.width / 2.0) < s.pipe.width / 8.0 &&
                       std::fabs(s.pipe.centroid_y - s.pipe.reference_row) < s.pipe.height / 8.0;
  if (!squared && settled && fit.valid && centred) {
    heading_ = wrapAngle(s.yaw - fit.angle);
    on_heading_since_ = -1.0;
    reaimed_ = true;
    RCLCPP_INFO(logger(), "re-aim to %.0f deg (pipe at %.0f deg in view)", heading_ * 180 / M_PI, fit.angle * 180 / M_PI);
  }
  if (squared || elapsed() > 40.0) {
    RCLCPP_INFO(logger(), "turn to %.0f deg: err %.0f deg", heading_ * 180 / M_PI, err * 180 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace banyu_mission::task2
