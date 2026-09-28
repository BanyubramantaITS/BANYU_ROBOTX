// Horizontal PD holding a light node's switch under the repair probe tip. The sticks are thrust
// and the hull coasts, so without the damping term it swings past the switch.
#ifndef BANYU_MISSION_TASK2_PROBE_SERVO_HPP_
#define BANYU_MISSION_TASK2_PROBE_SERVO_HPP_

#include <utility>

#include "banyu_mission/task2/vehicle_io.hpp"

namespace banyu_mission::task2 {

class ProbeServo {
 public:
  void reset() { last_s_ = -1; }
  // (fwd, right) sticks for one new sighting at time now_s.
  std::pair<double, double> update(const ProbeFix& fix, double now_s);

 private:
  double last_s_{-1}, last_fwd_{0}, last_right_{0};
  double rate_fwd_{0}, rate_right_{0};  // m/s, low-passed offset rate
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_PROBE_SERVO_HPP_
