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
  double trim_mps{0.0};        // heave with no range (e.g. a small push down against positive buoyancy)
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

  // Up-positive heave that holds `standoff` (the configured one if not given).
  double command(double t, std::optional<double> standoff = std::nullopt) const
  {
    const auto r = range(t);
    if (!r) {return config_.trim_mps;}
    const double error = *r - standoff.value_or(config_.standoff_m);  // > 0: too high
    if (std::abs(error) <= config_.deadband_m) {return config_.trim_mps;}
    return std::clamp(-config_.kp * error + config_.trim_mps, -config_.max_rate_mps, config_.max_rate_mps);
  }

  void reset() {ranges_.clear();}

private:
  CameraHeaveConfig config_;
  std::deque<double> ranges_;
  double last_t_{-1.0e9};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__CAMERA_HEAVE_HPP_
