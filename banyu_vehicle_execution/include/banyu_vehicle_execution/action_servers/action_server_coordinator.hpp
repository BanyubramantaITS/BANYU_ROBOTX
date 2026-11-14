#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ACTION_SERVER_COORDINATOR_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ACTION_SERVER_COORDINATOR_HPP_

#include <string>
#include <vector>

#include "banyu_vehicle_execution/goal_arbiter.hpp"

namespace banyu_vehicle_execution
{

class ActionServerCoordinator
{
public:
  virtual ~ActionServerCoordinator() = default;
  virtual GoalRequest requestGoal(const GoalIdentity & identity) = 0;
  virtual bool cancelGoal(const banyu_vehicle_execution::GoalHandle & handle) = 0;
  virtual void completeGoal(const banyu_vehicle_execution::GoalHandle & handle) = 0;
  virtual void faultGoal(const banyu_vehicle_execution::GoalHandle & handle) = 0;
  virtual void notifyDisplaced(
    const GoalIdentity & displaced_identity,
    const std::string & new_owner) = 0;
  virtual void setBuoyKeepOut(
    const std::vector<double> & /*latitude_deg*/,
    const std::vector<double> & /*longitude_deg*/,
    double /*radius_m*/) {}
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__ACTION_SERVER_COORDINATOR_HPP_
