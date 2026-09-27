// Surface transit to the active buoy on the pinger bearing. At the surface the pinger is barely
// deeper than the hydrophones and its hang depth is unknown, so elevation cannot give a safe
// standoff: stop at the first clear elevation (>= 10 deg, ~1.5 m out for a shallow pinger,
// further for a deep one) and let FindPipe's orbit close the rest, below the buoy's hull.
#ifndef BANYU_MISSION_TASK2_TRANSIT_TO_PINGER_HPP_
#define BANYU_MISSION_TASK2_TRANSIT_TO_PINGER_HPP_

#include <optional>

#include "banyu_mission/task2/task2_action.hpp"

namespace banyu_mission::task2 {

class TransitToPinger : public Task2Action {
 public:
  using Task2Action::Task2Action;
  static BT::PortsList providedPorts() { return {}; }

 protected:
  BT::NodeStatus begin() override;
  BT::NodeStatus step(const Snapshot& s) override;

 private:
  std::optional<banyu_msgs::msg::PingerObservation> last_;
  int close_pings_{0};
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_TRANSIT_TO_PINGER_HPP_
