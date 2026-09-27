// Pipe just came into view: set the depth target to survey altitude above it. The echosounder
// still ranges the seabed here, so the pipe is assumed task2_pipe_height above it; FollowPipe
// then corrects the target against the pipe itself.
#ifndef BANYU_MISSION_TASK2_SET_SURVEY_DEPTH_HPP_
#define BANYU_MISSION_TASK2_SET_SURVEY_DEPTH_HPP_

#include "banyu_mission/task2/sync_nodes_base.hpp"

namespace banyu_mission::task2 {

class SetSurveyDepth : public Task2SyncAction {
 public:
  using Task2SyncAction::Task2SyncAction;
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_SET_SURVEY_DEPTH_HPP_
