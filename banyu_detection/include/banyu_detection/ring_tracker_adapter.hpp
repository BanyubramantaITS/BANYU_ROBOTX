#ifndef BANYU_DETECTION_RING_TRACKER_ADAPTER_HPP_
#define BANYU_DETECTION_RING_TRACKER_ADAPTER_HPP_

#include <cstdint>
#include <limits>

#include "banyu_detection/ring_tracker.hpp"

namespace banyu_detection
{

constexpr uint32_t kInvalidRingIndex = std::numeric_limits<uint32_t>::max();

struct RepairAcknowledgementResult
{
  bool success{false};
  const char * reason{""};
  uint32_t accepted_ring_index{kInvalidRingIndex};
};

class RingTrackerAdapter
{
public:
  explicit RingTrackerAdapter(RingTrackerConfig config = {})
  : tracker_(config) {}

  RingTrackerUpdate update(const RingTrackerInput & input)
  {
    return tracker_.update(input);
  }

  RepairAcknowledgementResult acknowledgeRingRepair(uint32_t ring_index);
  void reset();

  const RingTracker & tracker() const {return tracker_;}

private:
  RingTracker tracker_;
  bool repair_acknowledged_{false};
  uint32_t repaired_ring_index_{kInvalidRingIndex};
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_RING_TRACKER_ADAPTER_HPP_
