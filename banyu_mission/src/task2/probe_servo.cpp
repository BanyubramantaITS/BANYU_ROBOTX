#include "banyu_mission/task2/probe_servo.hpp"

#include <algorithm>

namespace banyu_mission::task2 {

namespace {
constexpr double kP = 0.6, kD = 1.5, kMaxStick = 0.06;
}  // namespace

std::pair<double, double> ProbeServo::update(const ProbeFix& fix, double now_s) {
  const double dt = now_s - last_s_;
  if (last_s_ < 0 || dt > 0.01) {  // ticks outpace frames: repeats of a sighting change nothing
    if (last_s_ >= 0 && dt < 0.5) {
      rate_fwd_ += 0.3 * ((fix.fwd - last_fwd_) / dt - rate_fwd_);
      rate_right_ += 0.3 * ((fix.right - last_right_) / dt - rate_right_);
    } else {
      rate_fwd_ = rate_right_ = 0;  // restart after a gap: from afar the ring shows in some frames only
    }
    last_s_ = now_s;
    last_fwd_ = fix.fwd;
    last_right_ = fix.right;
  }
  return {std::clamp(kP * fix.fwd + kD * rate_fwd_, -kMaxStick, kMaxStick),
          std::clamp(kP * fix.right + kD * rate_right_, -kMaxStick, kMaxStick)};
}

}  // namespace banyu_mission::task2
