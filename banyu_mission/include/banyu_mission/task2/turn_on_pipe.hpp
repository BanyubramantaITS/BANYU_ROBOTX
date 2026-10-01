// Rotate while pivoting on the pipe pixels in view (ALT_HOLD has no position hold). reverse=false
// squares up on the pipe ahead; reverse=true turns 180 deg from the current heading. Gives up
// after 40 s and carries on: a partial turn is still followable.
#ifndef BANYU_MISSION_TASK2_TURN_ON_PIPE_HPP_
#define BANYU_MISSION_TASK2_TURN_ON_PIPE_HPP_

#include "banyu_mission/task2/task2_action.hpp"

namespace banyu_mission::task2 {

class TurnOnPipe : public Task2Action {
 public:
  using Task2Action::Task2Action;
  static BT::PortsList providedPorts() { return {BT::InputPort<bool>("reverse", false, "turn 180 deg instead of squaring up")}; }

 protected:
  BT::NodeStatus begin() override;
  BT::NodeStatus step(const Snapshot& s) override;

 private:
  bool reverse_{false};
  bool reaimed_{false};
  double heading_{0.0};
  double on_heading_since_{-1.0};  // s, elapsed() when the heading was reached; -1 while off it
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_TURN_ON_PIPE_HPP_
