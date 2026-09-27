// Follow the pipe to its end: direction=1 nose first, -1 tail first (the survey leg, flown
// backwards on the same heading because a 180 deg pivot without a position fix drifts along the
// pipe). record=true feeds every light node seen into the context's LedTracker.
// Depth is held survey-altitude above the pipe on the echosounder while centred on it.
#ifndef BANYU_MISSION_TASK2_FOLLOW_PIPE_HPP_
#define BANYU_MISSION_TASK2_FOLLOW_PIPE_HPP_

#include "banyu_mission/task2/task2_action.hpp"

namespace banyu_mission::task2 {

class FollowPipe : public Task2Action {
 public:
  using Task2Action::Task2Action;
  static BT::PortsList providedPorts() {
    return {BT::InputPort<int>("direction", 1, "1 = nose first, -1 = tail first"),
            BT::InputPort<bool>("record", false, "track the light nodes")};
  }

 protected:
  BT::NodeStatus begin() override;
  BT::NodeStatus step(const Snapshot& s) override;

 private:
  int direction_{1};
  bool record_{false};
  bool had_fit_{false};
  int end_frames_{0};
  double lost_since_{-1.0};
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_FOLLOW_PIPE_HPP_
