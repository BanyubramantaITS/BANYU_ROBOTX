#include "banyu_detection/ring_tracker_adapter.hpp"

namespace banyu_detection
{

RepairAcknowledgementResult RingTrackerAdapter::acknowledgeRingRepair(uint32_t ring_index)
{
  RepairAcknowledgementResult result;
  const auto & records = tracker_.records();
  if (ring_index >= records.size()) {
    result.reason = "ring index is not an accepted record";
    return result;
  }
  if (records[ring_index].color != banyu_msgs::msg::PipelineRingObservation::COLOR_RED) {
    result.reason = "ring index is not the accepted red ring";
    return result;
  }
  if (repair_acknowledged_ || repaired_ring_index_ == ring_index) {
    result.reason = "red ring repair was already acknowledged";
    return result;
  }
  if (!tracker_.redPending()) {
    result.reason = "red ring repair is not pending";
    return result;
  }
  if (!tracker_.markRedRepaired()) {
    result.reason = "red ring repair acknowledgement was rejected";
    return result;
  }

  repair_acknowledged_ = true;
  repaired_ring_index_ = ring_index;
  result.success = true;
  result.reason = "red ring repair acknowledged";
  result.accepted_ring_index = ring_index;
  return result;
}

}  // namespace banyu_detection
