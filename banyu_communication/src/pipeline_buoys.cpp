#include "banyu_communication/pipeline_buoys.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "rx_msgs/msg/vehicle_type.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"

namespace banyu_communication
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEarthRadiusM = 6378137.0;

int64_t stampNs(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.nanosec >= 1000000000U) {return 0;}
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

void enuOffset(
  const rx_msgs::msg::LatLng & from, const rx_msgs::msg::LatLng & to,
  double & east, double & north)
{
  const double mean_latitude = (from.latitude + to.latitude) * 0.5 * kPi / 180.0;
  north = (to.latitude - from.latitude) * kPi / 180.0 * kEarthRadiusM;
  east = (to.longitude - from.longitude) * kPi / 180.0 * kEarthRadiusM * std::cos(mean_latitude);
}
}  // namespace

PipelineBuoysTracker::PipelineBuoysTracker()
: PipelineBuoysTracker(Config{}) {}

PipelineBuoysTracker::PipelineBuoysTracker(Config config)
: config_(std::move(config))
{
  if (config_.source_id.empty() || config_.endpoint_id.empty() ||
    !std::isfinite(config_.default_uncertainty_m) || config_.default_uncertainty_m <= 0.0 ||
    !std::isfinite(config_.minimum_buoy_separation_m) || config_.minimum_buoy_separation_m < 0.0 ||
    !std::isfinite(config_.fallback_uncertainty_m) || config_.fallback_uncertainty_m <= 0.0)
  {
    throw std::invalid_argument("Invalid PipelineBuoys tracker configuration");
  }
  rx_msgs::msg::LatLng fallback;
  fallback.latitude = config_.fallback_latitude;
  fallback.longitude = config_.fallback_longitude;
  if (config_.fallback_enabled && !validPoint(fallback)) {
    throw std::invalid_argument("Fallback active buoy must be a valid WGS-84 position");
  }
}

void PipelineBuoysTracker::setCourse(const rx_msgs::msg::Course & course)
{
  if (course_ && course_->course_id != course.course_id && !locked_ && observation_) {
    observation_.reset();
    ++revision_;
  }
  course_ = course;
}

std::string PipelineBuoysTracker::offer(const rx_msgs::msg::PipelineBuoys & msg)
{
  using rx_msgs::msg::PipelineBuoys;
  if (locked_) {return "LOCKED: the UUV has dived in this run";}
  if (msg.source_vehicle != rx_msgs::msg::VehicleType::TYPE_USV) {return "SOURCE_NOT_USV";}
  if (!course_) {return "COURSE_UNAVAILABLE";}
  if (msg.course_id != course_->course_id) {
    return "COURSE_MISMATCH: '" + msg.course_id + "' is not '" + course_->course_id + "'";
  }
  if (!validPoint(msg.buoy_a)) {return "INVALID_BUOY_A";}
  if (msg.buoy_b_valid && !validPoint(msg.buoy_b)) {return "INVALID_BUOY_B";}
  if (msg.active_buoy > PipelineBuoys::ACTIVE_B ||
    (msg.active_buoy == PipelineBuoys::ACTIVE_B && !msg.buoy_b_valid))
  {
    return "INVALID_ACTIVE_BUOY";
  }
  if (!std::isfinite(msg.uncertainty_m) || msg.uncertainty_m < 0.0F) {
    return "INVALID_UNCERTAINTY";
  }
  if (config_.require_inside_course && course_->boundary.size() >= 3 &&
    (!insidePolygon(msg.buoy_a, course_->boundary) ||
    (msg.buoy_b_valid && !insidePolygon(msg.buoy_b, course_->boundary))))
  {
    return "OUTSIDE_COURSE";
  }
  if (observation_) {
    if (msg.seq == observation_->seq) {return "DUPLICATE";}
    const auto incoming = stampNs(msg.header.stamp);
    const auto stored = stampNs(observation_->header.stamp);
    // A lower seq is only a restarted USV when its observation is provably newer.
    if (msg.seq < observation_->seq && (incoming == 0 || stored == 0 || incoming <= stored)) {
      return "STALE_SEQ";
    }
  }
  observation_ = msg;
  ++revision_;
  return {};
}

void PipelineBuoysTracker::beginRun(uint32_t run_id, uint64_t epoch)
{
  if (epoch == epoch_ && run_id == run_id_) {return;}
  run_id_ = run_id;
  epoch_ = epoch;
  locked_ = false;
  ++revision_;
}

bool PipelineBuoysTracker::activeKnown() const
{
  return observation_ && observation_->active_buoy != rx_msgs::msg::PipelineBuoys::ACTIVE_UNKNOWN;
}

std::optional<banyu_msgs::msg::Task2BuoyTarget> PipelineBuoysTracker::target(
  const builtin_interfaces::msg::Time & stamp) const
{
  if (!activeKnown()) {return std::nullopt;}
  banyu_msgs::msg::Task2BuoyTarget target;
  target.header.stamp = stamp;
  target.header.frame_id = "wgs84";
  target.course_id = observation_->course_id;
  target.source_seq = observation_->seq;
  target.source_vehicle = observation_->source_vehicle;
  const auto & active = activePoint();
  target.latitude = active.latitude;
  target.longitude = active.longitude;
  const auto * other = otherPoint();
  target.heading_valid = other != nullptr &&
    distanceM(active, *other) >= std::max(config_.minimum_buoy_separation_m, 1e-3);
  target.heading_enu_rad = target.heading_valid ? enuBearing(active, *other) : 0.0;
  target.other_valid = target.heading_valid;
  if (target.other_valid) {
    target.other_latitude = other->latitude;
    target.other_longitude = other->longitude;
  }
  target.uncertainty_m = uncertainty();
  target.locked = locked_;
  return target;
}

std::optional<banyu_msgs::msg::ActiveBuoyFix> PipelineBuoysTracker::activeBuoyFix(
  const builtin_interfaces::msg::Time & stamp, uint32_t pinger_freq_hz) const
{
  if (run_id_ == 0 || epoch_ == 0 || !course_) {return std::nullopt;}
  const bool usv = activeKnown();
  if (!usv && !config_.fallback_enabled) {return std::nullopt;}

  banyu_msgs::msg::ActiveBuoyFix buoy;
  buoy.header.stamp = stamp;
  buoy.header.frame_id = "map";
  buoy.run_id = run_id_;
  buoy.epoch = epoch_;
  buoy.revision = std::max<uint64_t>(revision_, 1);
  buoy.source_id = config_.source_id;
  buoy.course_id = course_->course_id;
  buoy.endpoint_id = config_.endpoint_id;
  buoy.fix.header.stamp = stamp;
  buoy.fix.header.frame_id = "wgs84";
  buoy.fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  buoy.fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  if (usv) {
    buoy.fix.latitude = activePoint().latitude;
    buoy.fix.longitude = activePoint().longitude;
    buoy.uncertainty_m = uncertainty();
    buoy.green_confirmed = true;
  } else {
    buoy.fix.latitude = config_.fallback_latitude;
    buoy.fix.longitude = config_.fallback_longitude;
    buoy.uncertainty_m = config_.fallback_uncertainty_m;
    buoy.green_confirmed = config_.fallback_green_confirmed;
  }
  buoy.pinger_confirmed = false;
  buoy.pinger_freq_hz = pinger_freq_hz;
  return buoy;
}

bool PipelineBuoysTracker::insidePolygon(
  const rx_msgs::msg::LatLng & point, const std::vector<rx_msgs::msg::LatLng> & ring)
{
  bool inside = false;
  const auto count = ring.size();
  for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
    const auto & a = ring[i];
    const auto & b = ring[j];
    if ((a.longitude > point.longitude) != (b.longitude > point.longitude)) {
      const double crossing = a.latitude + (point.longitude - a.longitude) *
        (b.latitude - a.latitude) / (b.longitude - a.longitude);
      if (point.latitude < crossing) {inside = !inside;}
    }
  }
  return inside;
}

double PipelineBuoysTracker::enuBearing(
  const rx_msgs::msg::LatLng & from, const rx_msgs::msg::LatLng & to)
{
  double east = 0.0;
  double north = 0.0;
  enuOffset(from, to, east, north);
  return std::atan2(north, east);
}

double PipelineBuoysTracker::distanceM(
  const rx_msgs::msg::LatLng & from, const rx_msgs::msg::LatLng & to)
{
  double east = 0.0;
  double north = 0.0;
  enuOffset(from, to, east, north);
  return std::hypot(east, north);
}

bool PipelineBuoysTracker::validPoint(const rx_msgs::msg::LatLng & point)
{
  return std::isfinite(point.latitude) && std::isfinite(point.longitude) &&
         std::abs(point.latitude) <= 90.0 && std::abs(point.longitude) <= 180.0 &&
         (point.latitude != 0.0 || point.longitude != 0.0);
}

const rx_msgs::msg::LatLng & PipelineBuoysTracker::activePoint() const
{
  return observation_->active_buoy == rx_msgs::msg::PipelineBuoys::ACTIVE_B ?
         observation_->buoy_b : observation_->buoy_a;
}

const rx_msgs::msg::LatLng * PipelineBuoysTracker::otherPoint() const
{
  if (!observation_->buoy_b_valid) {return nullptr;}
  return observation_->active_buoy == rx_msgs::msg::PipelineBuoys::ACTIVE_B ?
         &observation_->buoy_a : &observation_->buoy_b;
}

double PipelineBuoysTracker::uncertainty() const
{
  return observation_->uncertainty_m > 0.0F ?
         static_cast<double>(observation_->uncertainty_m) : config_.default_uncertainty_m;
}

banyu_msgs::msg::SurfaceObstacles safePassageObstacles(
  const rx_msgs::msg::SafePassage & msg, const std::vector<rx_msgs::msg::LatLng> & boundary, double radius_m)
{
  constexpr double kSameBuoyM = 0.5;
  banyu_msgs::msg::SurfaceObstacles obstacles;
  obstacles.header.stamp = msg.header.stamp;
  obstacles.header.frame_id = "wgs84";
  obstacles.source = "uav_safe_passage";
  std::vector<rx_msgs::msg::LatLng> kept;
  const auto add = [&](const rx_msgs::msg::LatLng & p) {
      if (!PipelineBuoysTracker::validPoint(p)) {return;}
      if (boundary.size() >= 3 && !PipelineBuoysTracker::insidePolygon(p, boundary)) {return;}
      for (const auto & k : kept) {
        if (PipelineBuoysTracker::distanceM(k, p) < kSameBuoyM) {return;}
      }
      kept.push_back(p);
      obstacles.latitude_deg.push_back(p.latitude);
      obstacles.longitude_deg.push_back(p.longitude);
      obstacles.radius_m.push_back(radius_m);
    };
  add(msg.entry_position);
  add(msg.exit_position);
  for (const auto & buoy : msg.buoys) {add(buoy.position);}
  return obstacles;
}

}  // namespace banyu_communication
