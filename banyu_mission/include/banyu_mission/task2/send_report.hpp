// Publish the ordered segment states from the survey, with the buoy's GPS fix. Runs after
// SurfaceVehicle: the report link only works at the surface, and the OCS relays it to
// RoboCommand (banyu_robocommand) - vehicles never talk to RoboCommand directly (handbook 3.4).
#ifndef BANYU_MISSION_TASK2_SEND_REPORT_HPP_
#define BANYU_MISSION_TASK2_SEND_REPORT_HPP_

#include "banyu_mission/task2/sync_nodes_base.hpp"

namespace banyu_mission::task2 {

class SendReport : public Task2SyncAction {
 public:
  using Task2SyncAction::Task2SyncAction;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_SEND_REPORT_HPP_
