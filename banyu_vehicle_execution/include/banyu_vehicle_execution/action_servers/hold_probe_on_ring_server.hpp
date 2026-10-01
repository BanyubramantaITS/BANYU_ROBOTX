#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__HOLD_PROBE_ON_RING_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__HOLD_PROBE_ON_RING_SERVER_HPP_

#include <string>

#include "banyu_msgs/action/hold_probe_on_ring.hpp"
#include "banyu_vehicle_execution/action_servers/typed_action_server.hpp"

namespace banyu_vehicle_execution
{

class HoldProbeOnRingServer final
  : public TypedActionServer<banyu_msgs::action::HoldProbeOnRing>
{
public:
  HoldProbeOnRingServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator);

  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

protected:
  void onGoalStarted() override;
  bool validateGoal(const Goal & goal, std::string & reason) override;

private:
  double elapsed_aligned_s_{0.0};
  double max_alignment_error_{0.0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__HOLD_PROBE_ON_RING_SERVER_HPP_
