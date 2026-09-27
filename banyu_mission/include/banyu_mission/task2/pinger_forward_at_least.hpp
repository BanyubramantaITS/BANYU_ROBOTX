// Hold over the pipe for `pings` pings and compare the mean forward component of the unit
// vector to the pinger against min_forward. The start end is the one next to the active buoy,
// so: min_forward=0 asks "is the pinger ahead?", -0.2 asks "is it at least not well behind?".
#ifndef BANYU_MISSION_TASK2_PINGER_FORWARD_AT_LEAST_HPP_
#define BANYU_MISSION_TASK2_PINGER_FORWARD_AT_LEAST_HPP_

#include <vector>

#include "banyu_mission/task2/task2_action.hpp"

namespace banyu_mission::task2 {

class PingerForwardAtLeast : public Task2Action {
 public:
  using Task2Action::Task2Action;
  static BT::PortsList providedPorts() {
    return {BT::InputPort<double>("min_forward", 0.0, "threshold on the mean forward component"),
            BT::InputPort<int>("pings", 3, "pings to average")};
  }

 protected:
  BT::NodeStatus begin() override;
  BT::NodeStatus step(const Snapshot& s) override;

 private:
  std::vector<banyu_msgs::msg::PingerObservation> pings_;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_PINGER_FORWARD_AT_LEAST_HPP_
