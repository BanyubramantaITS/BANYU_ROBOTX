#include "banyu_mission/task2/set_flight_mode.hpp"

#include "banyu_mission/task2/vehicle_io.hpp"

namespace banyu_mission::task2 {

BT::NodeStatus SetFlightMode::tick() {
  const auto mode = getInput<std::string>("mode");
  if (!mode) throw BT::RuntimeError("SetFlightMode: missing port [mode]");
  return setMode(*ctx_, node_, *mode) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission::task2
