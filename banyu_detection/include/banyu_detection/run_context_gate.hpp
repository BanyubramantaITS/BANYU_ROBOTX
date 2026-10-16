#ifndef BANYU_DETECTION_RUN_CONTEXT_GATE_HPP_
#define BANYU_DETECTION_RUN_CONTEXT_GATE_HPP_

#include <cstdint>

namespace banyu_detection
{

struct ResetContextResult
{
  bool success{false};
  bool changed{false};
  const char * message{""};
};

class RunContextGate
{
public:
  ResetContextResult accept(uint32_t run_id, uint64_t epoch, int64_t reset_stamp_ns)
  {
    if (run_id == 0 || epoch == 0) {
      return {false, false, "run_id and epoch must be nonzero"};
    }
    if (epoch == epoch_ && run_id == run_id_) {
      return {true, false, "run context already initialized"};
    }
    if (epoch <= epoch_) {
      return {false, false, "stale or conflicting run context"};
    }
    run_id_ = run_id;
    epoch_ = epoch;
    reset_stamp_ns_ = reset_stamp_ns;
    return {true, true, "run context initialized"};
  }

  bool acceptsStamp(int64_t stamp_ns, bool require_context) const
  {
    if (epoch_ == 0) {
      return !require_context;
    }
    return stamp_ns > 0 && stamp_ns >= reset_stamp_ns_;
  }

  bool initialized() const {return epoch_ != 0;}
  uint32_t runId() const {return run_id_;}
  uint64_t epoch() const {return epoch_;}

private:
  uint32_t run_id_{0};
  uint64_t epoch_{0};
  int64_t reset_stamp_ns_{0};
};

}  // namespace banyu_detection

#endif  // BANYU_DETECTION_RUN_CONTEXT_GATE_HPP_
