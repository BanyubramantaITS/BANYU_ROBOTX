// Switch the ArduSub flight mode (STABILIZE at the surface, ALT_HOLD underwater).
#ifndef BANYU_MISSION_TASK2_SET_FLIGHT_MODE_HPP_
#define BANYU_MISSION_TASK2_SET_FLIGHT_MODE_HPP_

#include <string>

#include "banyu_mission/task2/sync_nodes_base.hpp"

namespace banyu_mission::task2 {

class SetFlightMode : public Task2SyncAction {
 public:
  using Task2SyncAction::Task2SyncAction;
  static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("mode", "ArduSub mode name")}; }
  BT::NodeStatus tick() override;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_SET_FLIGHT_MODE_HPP_
