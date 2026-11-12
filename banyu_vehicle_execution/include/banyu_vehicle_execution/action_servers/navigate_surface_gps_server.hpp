#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__NAVIGATE_SURFACE_GPS_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__NAVIGATE_SURFACE_GPS_SERVER_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "banyu_msgs/action/navigate_surface_gps.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"
#include "banyu_vehicle_execution/task4_zones.hpp"

namespace banyu_vehicle_execution
{

// Where to steer, in ENU metres relative to the vehicle, to reach `goal` without passing within `radius` of any
// obstacle: the goal itself when the straight line is clear, else a point beside the first obstacle in the way
// (1.5 x radius from it, on the side the line already passes); straight away from an obstacle the vehicle is inside.
std::pair<double, double> keepOutAim(
  std::pair<double, double> goal, const std::vector<std::pair<double, double>> & obstacles, double radius);

// Latitude and longitude of the point `offset` (ENU metres) from (ref_lat, ref_lon); the inverse of enuOffset.
std::pair<double, double> latLonAt(double ref_lat, double ref_lon, Vec2 offset);

// Velocity out of a Task 4 zone along `exit`, faster the deeper inside, at most max_speed_mps.
MotionIntent zoneExitIntent(const Exit & exit, double max_speed_mps, double yaw_rate);

class NavigateSurfaceGpsServer final
  : public TypedActionServer<banyu_msgs::action::NavigateSurfaceGps>
{
public:
  NavigateSurfaceGpsServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator,
    double nav_kp = 0.5,
    double nav_max_speed = 1.0,
    double yaw_kp = 1.0,
    double max_yaw_rate = 0.5);

  // A dive point shifted out of a zone stays at least this far from the buoys (the start standoff).
  void setZoneBuoyClear(double metres) {zone_buoy_clear_m_ = metres;}

  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

protected:
  void onGoalStarted() override;
  bool validateGoal(const Goal & goal, std::string & reason) override;

private:
  double nav_kp_{0.5};
  double nav_max_speed_{1.0};
  double yaw_kp_{1.0};
  double max_yaw_rate_{0.5};
  double zone_buoy_clear_m_{1.5};
  // Detour point being steered at (latitude, longitude), kept by keepDetourPoint.
  std::optional<std::pair<double, double>> detour_point_;
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__NAVIGATE_SURFACE_GPS_SERVER_HPP_
