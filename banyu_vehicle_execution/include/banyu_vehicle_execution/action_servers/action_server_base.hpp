#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ACTION_SERVER_BASE_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ACTION_SERVER_BASE_HPP_

#include <string>

#include "banyu_vehicle_execution/goal_arbiter.hpp"
#include "banyu_vehicle_execution/motion_intent.hpp"
#include "banyu_vehicle_execution/vehicle_telemetry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace banyu_vehicle_execution
{

class ActionServerBase
{
public:
  virtual ~ActionServerBase() = default;
  virtual std::string name() const = 0;
  virtual bool hasActiveGoal() const = 0;
  virtual bool hasPendingGoal() const = 0;
  virtual GoalIdentity activeIdentity() const = 0;
  virtual GoalIdentity pendingIdentity() const = 0;
  virtual void activatePending() = 0;
  virtual void cancelActive() = 0;
  virtual void abortActive(const std::string & reason) = 0;
  virtual void cancelPending() = 0;
  virtual void abortPending(const std::string & reason) = 0;
  virtual MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) = 0;
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ACTION_SERVER_BASE_HPP_
