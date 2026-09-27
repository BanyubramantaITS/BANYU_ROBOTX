// Descend to search altitude on the echosounder, watching the bottom camera; if the pipe isn't in
// view by then, orbit the pinger at a widening radius (the pipe starts 1.5-2.5 m from the buoy,
// direction unknown). Succeeds on 5 consecutive frames with the pipe in view.
#ifndef BANYU_MISSION_TASK2_FIND_PIPE_HPP_
#define BANYU_MISSION_TASK2_FIND_PIPE_HPP_

#include <optional>

#include "banyu_mission/task2/task2_action.hpp"

namespace banyu_mission::task2 {

class FindPipe : public Task2Action {
 public:
  using Task2Action::Task2Action;
  static BT::PortsList providedPorts() { return {}; }

 protected:
  BT::NodeStatus begin() override;
  BT::NodeStatus step(const Snapshot& s) override;

 private:
  int seen_frames_{0};
  std::optional<banyu_msgs::msg::PingerObservation> ping_;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_FIND_PIPE_HPP_
