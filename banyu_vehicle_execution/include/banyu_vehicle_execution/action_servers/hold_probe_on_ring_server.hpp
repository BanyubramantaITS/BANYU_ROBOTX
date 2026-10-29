#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__HOLD_PROBE_ON_RING_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__HOLD_PROBE_ON_RING_SERVER_HPP_

#include <limits>
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
    ActionServerCoordinator * coordinator,
    double ring_kp_sway = 0.8,
    double ring_kp_surge = 0.5);

  MotionIntent tick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now,
    double dt) override;

protected:
  void onGoalStarted() override;
  bool validateGoal(const Goal & goal, std::string & reason) override;

private:
  enum class Phase {Descend, Hold, Climb};

  // Approach mode (goal.descent_rate_mps > 0): sink onto the ring with the camera, hold on contact until the
  // ring turns GREEN, climb back to the starting depth.
  MotionIntent approachTick(
    const VehicleTelemetry & telemetry, const rclcpp::Time & now, double dt);
  void finish(bool success, const std::string & reason);

  Phase phase_{Phase::Descend};
  double start_depth_m_{0.0};
  double origin_depth_m_{std::numeric_limits<double>::quiet_NaN()};   // where the last approach started
  bool depth_captured_{false};
  bool saw_red_{false};
  bool repaired_{false};
  std::string failure_;
  double hold_s_{0.0};
  double lost_s_{0.0};
  double green_s_{0.0};
  // The lid is dark for most of a flash code; the last offset keeps the correction going for a few seconds.
  double last_offset_x_{0.0};
  double last_offset_y_{0.0};
  double since_lit_s_{1.0e9};
  double ring_kp_sway_{0.8};
  double ring_kp_surge_{0.5};
  double elapsed_aligned_s_{0.0};
  double max_alignment_error_{0.0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__HOLD_PROBE_ON_RING_SERVER_HPP_
