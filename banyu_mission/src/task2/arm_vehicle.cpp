#include "banyu_mission/task2/arm_vehicle.hpp"

#include "banyu_mission/task2/vehicle_io.hpp"

namespace banyu_mission::task2 {

BT::NodeStatus ArmVehicle::tick() {
  sendSticks(*ctx_, 0, 0, 0);  // neutral sticks first: ArduSub refuses to arm on a raised throttle
  return setArmed(*ctx_, node_, getInput<bool>("arm").value_or(true)) ? BT::NodeStatus::SUCCESS
                                                                      : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission::task2
