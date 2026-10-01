// After the repair, stay on the light node and read what it flashes (handbook 3.3.3): Advanced,
// the platform circle the UAV must deliver any tin to; Disruptive (task2_tier), the tin colour
// and then the circle. Succeeds as soon as it is read. Like RepairLight it runs in STABILIZE,
// pressing the tip on the switch.
#ifndef BANYU_MISSION_TASK2_READ_FLASH_HPP_
#define BANYU_MISSION_TASK2_READ_FLASH_HPP_

#include "banyu_mission/task2/flash_decoder.hpp"
#include "banyu_mission/task2/probe_servo.hpp"
#include "banyu_mission/task2/task2_action.hpp"

namespace banyu_mission::task2 {

class ReadFlash : public Task2Action {
 public:
  using Task2Action::Task2Action;
  static BT::PortsList providedPorts() { return {}; }

 protected:
  BT::NodeStatus begin() override;
  BT::NodeStatus step(const Snapshot& s) override;

 private:
  FlashDecoder decoder_;
  ProbeServo servo_;
  bool disruptive_{false};
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_READ_FLASH_HPP_
