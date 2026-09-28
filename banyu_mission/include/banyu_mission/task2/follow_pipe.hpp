// Follow the pipe to its end: direction=1 nose first, -1 tail first (the survey leg, flown
// backwards on the same heading because a 180 deg pivot without a position fix drifts along the
// pipe). record=true feeds every light node seen into the context's LedTracker, from scratch
// unless resume=true (which also climbs back to survey depth before judging the pipe's end). stop_at_red=true stops as soon as a RED node reaches the repair probe's
// pixel, and then FAILS if the pipe ends first instead.
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
            BT::InputPort<bool>("record", false, "track the light nodes"),
            BT::InputPort<bool>("resume", false, "keep the tracks recorded so far"),
            BT::InputPort<bool>("stop_at_red", false, "stop with a RED node at the probe")};
  }

 protected:
  BT::NodeStatus begin() override;
  BT::NodeStatus step(const Snapshot& s) override;

 private:
  int direction_{1};
  bool record_{false};
  bool stop_at_red_{false};
  bool climbing_{false};
  bool had_fit_{false};
  int end_frames_{0};
  double lost_since_{-1.0};
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_FOLLOW_PIPE_HPP_
