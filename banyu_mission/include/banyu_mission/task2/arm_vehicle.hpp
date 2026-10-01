// Arm (arm="true") or disarm the vehicle.
#ifndef BANYU_MISSION_TASK2_ARM_VEHICLE_HPP_
#define BANYU_MISSION_TASK2_ARM_VEHICLE_HPP_

#include "banyu_mission/task2/sync_nodes_base.hpp"

namespace banyu_mission::task2 {

class ArmVehicle : public Task2SyncAction {
 public:
  using Task2SyncAction::Task2SyncAction;
  static BT::PortsList providedPorts() { return {BT::InputPort<bool>("arm", true, "true = arm, false = disarm")}; }
  BT::NodeStatus tick() override;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_ARM_VEHICLE_HPP_
