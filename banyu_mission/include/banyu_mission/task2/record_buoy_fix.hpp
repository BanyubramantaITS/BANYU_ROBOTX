// Hold at the active buoy for 2 s and keep its GPS position for the report
// (RoboCommand's PipelineSurveyReport.active_buoy_position). NaN without a fix; never fails.
#ifndef BANYU_MISSION_TASK2_RECORD_BUOY_FIX_HPP_
#define BANYU_MISSION_TASK2_RECORD_BUOY_FIX_HPP_

#include "banyu_mission/task2/task2_action.hpp"

namespace banyu_mission::task2 {

class RecordBuoyFix : public Task2Action {
 public:
  using Task2Action::Task2Action;
  static BT::PortsList providedPorts() { return {}; }

 protected:
  BT::NodeStatus step(const Snapshot& s) override;
};

}  // namespace banyu_mission::task2

#endif  // BANYU_MISSION_TASK2_RECORD_BUOY_FIX_HPP_
