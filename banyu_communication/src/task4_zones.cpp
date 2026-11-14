#include "banyu_communication/task4_zones.hpp"

#include <cmath>

#include <rx_msgs/msg/vehicle_type.hpp>

namespace banyu_communication
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

bool validPosition(double latitude, double longitude)
{
  return std::isfinite(latitude) && std::isfinite(longitude) && latitude >= -90.0 && latitude <= 90.0 &&
         longitude >= -180.0 && longitude <= 180.0;
}
}  // namespace

Task4ZoneTracker::Task4ZoneTracker(const Config & config)
: config_(config) {}

void Task4ZoneTracker::reset()
{
  statics_.clear();
  moving_.reset();
  seen_.clear();
  last_seq_ = 0;
  last_event_ = "reset";
}

Task4ZoneTracker::Result Task4ZoneTracker::handle(const rx_msgs::msg::Command & command, double now_s)
{
  using rx_msgs::msg::Command;
  using rx_msgs::msg::VehicleType;
  Result result;
  const auto type = command.command_type;
  if (type != Command::CMD_TASK4_KEEP_OUT_ZONE && type != Command::CMD_TASK4_ALL_CLEAR &&
    type != Command::CMD_TASK4_MOVING_OBJECT)
  {
    return result;
  }
  const bool for_uuv = command.target_vehicle == VehicleType::TYPE_UUV ||
    command.target_vehicle == VehicleType::TYPE_UNKNOWN;
  // A UUV on the surface is a USV: zones addressed to the USV apply too (and are avoided without an ack).
  const bool for_usv = command.target_vehicle == VehicleType::TYPE_USV;
  if (!for_uuv && !for_usv) {return result;}
  result.relevant = true;
  result.ack = for_uuv && (type != Command::CMD_TASK4_MOVING_OBJECT || config_.ack_moving_object);

  // The same (command_seq, command_type) arrives once per addressed vehicle type and again on a resend.
  if (!seen_.insert({command.command_seq, type}).second) {
    result.duplicate = true;
    return result;
  }

  if (type == Command::CMD_TASK4_KEEP_OUT_ZONE) {
    const double lat = command.center.latitude;
    const double lon = command.center.longitude;
    const double radius = command.radius_m;
    if (!validPosition(lat, lon) || !std::isfinite(radius) || radius <= 0.0 || radius > config_.max_radius_m) {
      result.rejected = true;
      result.event = "keep_out_zone_rejected";
      return result;
    }
    // The same zone resent under a new seq updates its radius instead of piling up.
    for (auto & zone : statics_) {
      if (std::abs(zone.latitude - lat) < 5e-6 && std::abs(zone.longitude - lon) < 5e-6) {
        result.changed = zone.radius_m != radius;
        zone.radius_m = radius;
        result.event = "keep_out_zone";
        last_event_ = result.event;
        return result;
      }
    }
    if (statics_.size() >= config_.max_static_zones) {
      result.rejected = true;
      result.event = "keep_out_zone_limit";
      return result;
    }
    statics_.push_back({lat, lon, radius});
    result.changed = true;
    result.event = "keep_out_zone";
  } else if (type == Command::CMD_TASK4_ALL_CLEAR) {
    result.changed = !statics_.empty();
    statics_.clear();
    result.event = "all_clear";
  } else {
    const double lat = command.position.latitude;
    const double lon = command.position.longitude;
    const double radius = command.radius_m;
    if (!validPosition(lat, lon) || !std::isfinite(radius) || radius <= 0.0 || radius > config_.max_radius_m ||
      !std::isfinite(command.heading_deg) || !std::isfinite(command.speed_mps) || command.speed_mps < 0.0)
    {
      result.rejected = true;
      result.event = "moving_object_rejected";
      return result;
    }
    // Alerts are updated periodically: an older one arriving late must not replace a newer one.
    if (moving_ && command.command_seq < moving_->seq) {
      result.duplicate = true;
      return result;
    }
    // RoboCommand headings are compass degrees (clockwise from north); the vehicle works in ENU radians.
    const double heading_enu = kPi / 2.0 - static_cast<double>(command.heading_deg) * kPi / 180.0;
    moving_ = Moving{lat, lon, heading_enu, command.speed_mps, radius, now_s, command.command_seq};
    result.changed = true;
    result.event = "moving_object";
  }
  last_seq_ = command.command_seq;
  last_event_ = result.event;
  return result;
}

bool Task4ZoneTracker::expire(double now_s)
{
  if (moving_ && now_s - moving_->received_s > config_.moving_expiry_s) {
    moving_.reset();
    last_event_ = "moving_object_expired";
    return true;
  }
  return false;
}

void Task4ZoneTracker::fill(banyu_msgs::msg::Task4Zones & msg) const
{
  msg.static_latitude_deg.clear();
  msg.static_longitude_deg.clear();
  msg.static_radius_m.clear();
  for (const auto & zone : statics_) {
    msg.static_latitude_deg.push_back(zone.latitude);
    msg.static_longitude_deg.push_back(zone.longitude);
    msg.static_radius_m.push_back(zone.radius_m);
  }
  msg.moving_active = moving_.has_value();
  if (moving_) {
    msg.moving_latitude_deg = moving_->latitude;
    msg.moving_longitude_deg = moving_->longitude;
    msg.moving_heading_enu_rad = moving_->heading_enu_rad;
    msg.moving_speed_mps = moving_->speed_mps;
    msg.moving_radius_m = moving_->radius_m;
  }
  msg.last_event = last_event_;
  msg.last_command_seq = last_seq_;
}

}  // namespace banyu_communication
