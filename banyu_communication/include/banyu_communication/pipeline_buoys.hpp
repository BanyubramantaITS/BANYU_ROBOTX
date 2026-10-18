#ifndef BANYU_COMMUNICATION__PIPELINE_BUOYS_HPP_
#define BANYU_COMMUNICATION__PIPELINE_BUOYS_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "banyu_msgs/msg/active_buoy_fix.hpp"
#include "banyu_msgs/msg/task2_buoy_target.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rx_msgs/msg/course.hpp"
#include "rx_msgs/msg/pipeline_buoys.hpp"

namespace banyu_communication
{

// Validates the USV's rx_msgs/PipelineBuoys feed and turns the newest accepted observation into
// the UUV's Task 2 navigation target and run-tagged active buoy evidence.
class PipelineBuoysTracker
{
public:
  struct Config
  {
    std::string source_id{"usv_pipeline_buoys"};
    std::string endpoint_id{"start_buoy"};
    // Used when the USV sends uncertainty_m = 0 (unknown).
    double default_uncertainty_m{3.0};
    bool require_inside_course{true};
    // Buoys closer than this give no usable pipe direction.
    double minimum_buoy_separation_m{1.0};
    // Configured active buoy used for the survey evidence while the USV has not reported one.
    bool fallback_enabled{false};
    double fallback_latitude{0.0};
    double fallback_longitude{0.0};
    double fallback_uncertainty_m{3.0};
    bool fallback_green_confirmed{false};
  };

  PipelineBuoysTracker();
  explicit PipelineBuoysTracker(Config config);

  // A different course_id drops the stored observation.
  void setCourse(const rx_msgs::msg::Course & course);
  // Returns an empty string when the observation was accepted, otherwise the reason.
  std::string offer(const rx_msgs::msg::PipelineBuoys & msg);
  // A new accepted run tags the evidence; the stored observation is kept for the same course.
  void beginRun(uint32_t run_id, uint64_t epoch);
  // Locked while the UUV is submerged or past the survey; USV updates are ignored.
  void lock() {locked_ = true;}
  void unlock() {locked_ = false;}
  bool locked() const {return locked_;}
  bool hasObservation() const {return observation_.has_value();}
  bool activeKnown() const;
  uint64_t revision() const {return revision_;}

  // Navigation target; only once the active buoy is known.
  std::optional<banyu_msgs::msg::Task2BuoyTarget> target(
    const builtin_interfaces::msg::Time & stamp) const;
  // Survey evidence for the current run: USV active buoy, else the configured fallback.
  std::optional<banyu_msgs::msg::ActiveBuoyFix> activeBuoyFix(
    const builtin_interfaces::msg::Time & stamp, uint32_t pinger_freq_hz) const;

  static bool insidePolygon(
    const rx_msgs::msg::LatLng & point, const std::vector<rx_msgs::msg::LatLng> & ring);
  // ENU bearing (0 = east, counter-clockwise) and distance from one WGS-84 point to another.
  static double enuBearing(const rx_msgs::msg::LatLng & from, const rx_msgs::msg::LatLng & to);
  static double distanceM(const rx_msgs::msg::LatLng & from, const rx_msgs::msg::LatLng & to);

private:
  bool validPoint(const rx_msgs::msg::LatLng & point) const;
  const rx_msgs::msg::LatLng & activePoint() const;
  const rx_msgs::msg::LatLng * otherPoint() const;
  double uncertainty() const;

  Config config_;
  std::optional<rx_msgs::msg::Course> course_;
  std::optional<rx_msgs::msg::PipelineBuoys> observation_;
  uint32_t run_id_{0};
  uint64_t epoch_{0};
  uint64_t revision_{0};
  bool locked_{false};
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION__PIPELINE_BUOYS_HPP_
