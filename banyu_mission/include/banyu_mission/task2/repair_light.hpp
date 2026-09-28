// "Repair" the damaged segment (handbook 3.3.3): with the RED light node in view near the probe
// tip, hold its switch under the tip and lower until the tip rests on it, then hold until the
// light turns GREEN. Runs in STABILIZE (vertical stick = thrust): a small push lowers it and
// then holds the tip on the switch. Down beside the housing instead, it lifts and tries again.
#ifndef BANYU_MISSION_TASK2_REPAIR_LIGHT_HPP_
#define BANYU_MISSION_TASK2_REPAIR_LIGHT_HPP_

#include "banyu_mission/task2/probe_servo.hpp"
#include "banyu_mission/task2/task2_action.hpp"

namespace banyu_mission::task2 {

class RepairLight : public Task2Action {
 public:
  using Task2Action::Task2Action;
  static BT::PortsList providedPorts() { return {}; }

 protected:
  BT::NodeStatus begin() override;
  BT::NodeStatus step(const Snapshot& s) override;

 private:
  double seen_s_{0};
  double aligned_s_{-1};  // last sighting with the switch under the tip
  double aligned_z_{0};
  int green_frames_{0};
  ProbeServo servo_;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_REPAIR_LIGHT_HPP_
