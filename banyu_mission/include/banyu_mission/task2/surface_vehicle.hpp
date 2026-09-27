// Get to the surface - actually floating, not just shallow. Climb in ALT_HOLD to 0.5 m, then
// STABILIZE with neutral throttle: ALT_HOLD would hold that depth, and thrusting up the rest
// makes the sim vehicle fly (thrusters still push out of the water). Succeeds once pressure depth
// has stayed at the float line for 2 s; fails after 120 s.
#ifndef BANYU_MISSION_TASK2_SURFACE_VEHICLE_HPP_
#define BANYU_MISSION_TASK2_SURFACE_VEHICLE_HPP_

#include "banyu_mission/task2/task2_action.hpp"

namespace banyu_mission::task2 {

class SurfaceVehicle : public Task2Action {
 public:
  using Task2Action::Task2Action;
  static BT::PortsList providedPorts() { return {}; }

 protected:
  BT::NodeStatus begin() override;
  BT::NodeStatus step(const Snapshot& s) override;

 private:
  bool stabilized_{false};
  double floating_since_{-1.0};
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_SURFACE_VEHICLE_HPP_
