// Heave from the bottom camera when there is no depth sensor: the range to the pipe from its apparent width
// (range = diameter / (width_fraction x 2 tan(hfov / 2))) and a P loop that holds a standoff above it.
#ifndef BANYU_VEHICLE_EXECUTION__CAMERA_HEAVE_HPP_
#define BANYU_VEHICLE_EXECUTION__CAMERA_HEAVE_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <vector>

namespace banyu_vehicle_execution
{

struct CameraHeaveConfig
{
  double pipe_diameter_m{0.0889};  // 3" Schedule 40 PVC
  double hfov_rad{1.047};
  double standoff_m{1.6};
  double kp{0.4};
  double max_rate_mps{0.25};
  double deadband_m{0.1};
  double trim_mps{0.0};        // fixed heave bias (e.g. a push down against positive buoyancy)
  // Integral term: in STABILIZE the heave stick is thrust, and a positively buoyant vehicle rises with zero thrust. The
  // integral learns that trim while the pipe is in view and keeps it when the pipe is lost, so the vehicle does not float
  // up out of camera range.
  double ki{0.03};
  double max_integral_mps{0.15};
  double max_age_s{0.5};
  double min_range_m{0.3};
  double max_range_m{4.0};
  int median_window{7};
};

class CameraHeave
{
public:
  explicit CameraHeave(CameraHeaveConfig config = CameraHeaveConfig()) : config_(config) {}

  const CameraHeaveConfig & config() const {return config_;}

  std::optional<double> rangeFromWidth(double width_fraction) const
  {
    if (!std::isfinite(width_fraction) || width_fraction <= 0.0) {return std::nullopt;}
    const double range = config_.pipe_diameter_m / (width_fraction * 2.0 * std::tan(config_.hfov_rad / 2.0));
    if (range < config_.min_range_m || range > config_.max_range_m) {return std::nullopt;}
    return range;
  }

  // One pipe width sample (fraction of the image width, NaN if none) at time t (s).
  void addWidth(double width_fraction, double t)
  {
    const auto range = rangeFromWidth(width_fraction);
    if (!range) {return;}
    ranges_.push_back(*range);
    while (static_cast<int>(ranges_.size()) > config_.median_window) {ranges_.pop_front();}
    last_t_ = t;
  }

  // Median range of the recent samples, nullopt when stale at time t.
  std::optional<double> range(double t) const
  {
    if (ranges_.empty() || t - last_t_ > config_.max_age_s) {return std::nullopt;}
    std::vector<double> v(ranges_.begin(), ranges_.end());
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }

  // Up-positive heave that holds `standoff` (the configured one if not given). Call once per control tick.
  double command(double t, std::optional<double> standoff = std::nullopt)
  {
    const double dt = last_command_t_ > -1.0e8 ? std::clamp(t - last_command_t_, 0.0, 0.5) : 0.0;
    last_command_t_ = t;
    const auto r = range(t);
    if (!r) {return std::clamp(config_.trim_mps + integral_, -config_.max_rate_mps, config_.max_rate_mps);}
    const double error = *r - standoff.value_or(config_.standoff_m);  // > 0: too high
    integral_ = std::clamp(integral_ - config_.ki * error * dt, -config_.max_integral_mps, config_.max_integral_mps);
    const double p = std::abs(error) <= config_.deadband_m ? 0.0 : -config_.kp * error;
    return std::clamp(p + config_.trim_mps + integral_, -config_.max_rate_mps, config_.max_rate_mps);
  }

  double integral() const {return integral_;}
  // The learned buoyancy compensation, for actions that command their own heave.
  double bias() const {return config_.trim_mps + integral_;}

  void reset() {ranges_.clear();}

private:
  CameraHeaveConfig config_;
  std::deque<double> ranges_;
  double last_t_{-1.0e9};
  double last_command_t_{-1.0e9};
  double integral_{0.0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__CAMERA_HEAVE_HPP_
