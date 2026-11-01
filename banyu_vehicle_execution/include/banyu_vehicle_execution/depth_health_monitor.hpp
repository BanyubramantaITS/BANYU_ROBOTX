// Bar30 health from the depth samples alone and the commanded heave: silent, stuck, a step, or a drift away from the
// commanded heave. A fault is latched until reset(): once the depth sensor is not trusted the mission works without it.
#ifndef BANYU_VEHICLE_EXECUTION__DEPTH_HEALTH_MONITOR_HPP_
#define BANYU_VEHICLE_EXECUTION__DEPTH_HEALTH_MONITOR_HPP_

#include <cmath>
#include <string>

namespace banyu_vehicle_execution
{

struct DepthHealthConfig
{
  double silent_s{3.0};          // no sample this long
  double stuck_s{3.0};           // value unchanged this long while heave is commanded
  double stuck_epsilon_m{0.002};
  double heave_active_mps{0.05}; // commanded heave above this counts as "moving"
  double max_step_m{0.3};        // jump between two samples
  double max_disagree_m{1.0};    // depth change against the integrated commanded heave
  double disagree_window_s{10.0};
};

class DepthHealthMonitor
{
public:
  using Config = DepthHealthConfig;

  enum class Fault { kNone, kSilent, kStuck, kStep, kDisagree };

  explicit DepthHealthMonitor(Config config = Config()) : config_(config) {}

  void reset()
  {
    fault_ = Fault::kNone;
    have_sample_ = false;
    window_started_ = false;
    heave_mps_ = 0.0;
  }

  // Forget the sample history but keep a latched fault (e.g. while a GPS fix proves the surface).
  void restartSamples()
  {
    have_sample_ = false;
    window_started_ = false;
    moving_since_ = -1.0;
  }

  // Commanded heave, m/s up positive (ENU z), held until the next call.
  void setCommandedHeave(double heave_mps) {heave_mps_ = std::isfinite(heave_mps) ? heave_mps : 0.0;}

  // One depth sample (m, down positive) at time t (s).
  void addSample(double depth_m, double t)
  {
    if (!std::isfinite(depth_m)) {return;}
    if (have_sample_) {
      const double dt = t - last_t_;
      if (std::abs(depth_m - last_depth_) > config_.max_step_m) {latch(Fault::kStep);}
      if (std::abs(depth_m - last_change_depth_) > config_.stuck_epsilon_m) {
        last_change_depth_ = depth_m;
        last_change_t_ = t;
        moving_since_ = -1.0;
      }
      if (dt > 0.0 && dt < 1.0) {expected_depth_ -= heave_mps_ * dt;}
    } else {
      last_change_depth_ = depth_m;
      last_change_t_ = t;
    }
    if (!window_started_ || t - window_start_t_ >= config_.disagree_window_s) {
      window_started_ = true;
      window_start_t_ = t;
      expected_depth_ = depth_m;
    } else if (std::abs(depth_m - expected_depth_) > config_.max_disagree_m) {
      latch(Fault::kDisagree);
    }
    have_sample_ = true;
    last_depth_ = depth_m;
    last_t_ = t;
    check(t);
  }

  // Periodic check at time t (s), also with no new sample.
  void check(double t)
  {
    if (!have_sample_) {return;}
    if (t - last_t_ > config_.silent_s) {latch(Fault::kSilent);}
    if (std::abs(heave_mps_) > config_.heave_active_mps) {
      if (moving_since_ < 0.0) {moving_since_ = t;}
      const double stuck_for = t - std::max(last_change_t_, moving_since_);
      if (stuck_for > config_.stuck_s) {latch(Fault::kStuck);}
    } else {
      moving_since_ = -1.0;
    }
  }

  bool healthy() const {return fault_ == Fault::kNone;}
  Fault fault() const {return fault_;}
  static std::string name(Fault f)
  {
    switch (f) {
      case Fault::kSilent: return "SILENT";
      case Fault::kStuck: return "STUCK";
      case Fault::kStep: return "STEP";
      case Fault::kDisagree: return "DISAGREE";
      default: return "OK";
    }
  }

private:
  void latch(Fault f) {if (fault_ == Fault::kNone) {fault_ = f;}}

  Config config_;
  Fault fault_{Fault::kNone};
  bool have_sample_{false};
  double last_depth_{0.0};
  double last_t_{0.0};
  double last_change_depth_{0.0};
  double last_change_t_{0.0};
  double moving_since_{-1.0};
  double heave_mps_{0.0};
  bool window_started_{false};
  double window_start_t_{0.0};
  double expected_depth_{0.0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__DEPTH_HEALTH_MONITOR_HPP_
