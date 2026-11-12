#ifndef BANYU_COMMUNICATION__PIPELINE_BUOYS_HPP_
#define BANYU_COMMUNICATION__PIPELINE_BUOYS_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "banyu_msgs/msg/active_buoy_fix.hpp"
#include "banyu_msgs/msg/surface_obstacles.hpp"
#include "banyu_msgs/msg/task2_buoy_target.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "rx_msgs/msg/course.hpp"
#include "rx_msgs/msg/pipeline_buoys.hpp"
#include "rx_msgs/msg/safe_passage.hpp"

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
  // Finite WGS-84 position; (0, 0) means unset.
  static bool validPoint(const rx_msgs::msg::LatLng & point);

private:
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

// The UAV's Task 1 buoys (entry, exit and the whole field, duplicates merged) as surface obstacles of `radius_m`.
// With a course boundary (3+ corners), buoys outside it are dropped: they belong to another course.
banyu_msgs::msg::SurfaceObstacles safePassageObstacles(
  const rx_msgs::msg::SafePassage & msg, const std::vector<rx_msgs::msg::LatLng> & boundary, double radius_m);

// The UAV's buoy field remembered across SafePassage messages. The UAV's map drops buoys for minutes while the buoys
// stay put (trial 2026-11-12: two of six missing for about 9 min), so a buoy is kept until it has not been reported
// for `forget_s` (0 = kept until clear()). A report within `merge_m` of a remembered buoy moves it there.
class UavBuoyField
{
public:
  explicit UavBuoyField(double merge_m = 1.0, double forget_s = 0.0);

  void update(const rx_msgs::msg::SafePassage & msg, const std::vector<rx_msgs::msg::LatLng> & boundary, double now_s);
  void clear() {buoys_.clear();}
  std::size_t size() const {return buoys_.size();}
  // Remembered buoys inside `boundary` (any with fewer than 3 corners) as obstacles of `radius_m`.
  banyu_msgs::msg::SurfaceObstacles obstacles(
    const std::vector<rx_msgs::msg::LatLng> & boundary, double radius_m, double now_s) const;

private:
  struct Buoy
  {
    rx_msgs::msg::LatLng position;
    double last_seen_s{0.0};
  };
  bool expired(const Buoy & buoy, double now_s) const {return forget_s_ > 0.0 && now_s - buoy.last_seen_s > forget_s_;}

  double merge_m_;
  double forget_s_;
  std::vector<Buoy> buoys_;
};

}  // namespace banyu_communication

#endif  // BANYU_COMMUNICATION__PIPELINE_BUOYS_HPP_
