// Publish the resource request read off the repaired light (Task 2 Advanced tier: any tin to
// the flashed circle colour; Disruptive: the flashed tin colour to the flashed circle). Runs at the surface like SendReport: the OCS relays it to
// RoboCommand and the UAV is re-tasked from the same topic. Fails when no colour was read.
#ifndef BANYU_MISSION_TASK2_SEND_RESOURCE_REQUEST_HPP_
#define BANYU_MISSION_TASK2_SEND_RESOURCE_REQUEST_HPP_

#include "banyu_mission/task2/sync_nodes_base.hpp"

namespace banyu_mission::task2 {

class SendResourceRequest : public Task2SyncAction {
 public:
  using Task2SyncAction::Task2SyncAction;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_SEND_RESOURCE_REQUEST_HPP_
