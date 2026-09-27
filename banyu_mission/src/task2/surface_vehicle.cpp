#include "banyu_mission/task2/surface_vehicle.hpp"

namespace banyu_mission::task2 {

namespace {
constexpr double kStabilizeDepth = 0.5;  // m: switch to STABILIZE and let buoyancy finish
constexpr double kFloatDepth = 0.15;     // m: pressure depth when floating at the waterline
constexpr double kSteadyS = 2.0;
constexpr double kTimeoutS = 120.0;
}  // namespace

BT::NodeStatus SurfaceVehicle::begin() {
  stabilized_ = false;
  floating_since_ = -1.0;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SurfaceVehicle::step(const Snapshot& s) {
  if (s.depth <= kStabilizeDepth && !stabilized_) stabilized_ = setMode(*ctx_, node_, "STABILIZE");
  sendSticks(*ctx_, 0, 0, s.depth > kStabilizeDepth ? -0.5 : 0.0);

  if (s.depth >= kFloatDepth) {
    floating_since_ = -1.0;
  } else if (floating_since_ < 0) {
    floating_since_ = elapsed();
  } else if (elapsed() - floating_since_ >= kSteadyS) {
    RCLCPP_INFO(logger(), "surfaced (depth %.2f m)", s.depth);
    return BT::NodeStatus::SUCCESS;
  }
  if (elapsed() > kTimeoutS) {
    RCLCPP_ERROR(logger(), "not surfaced after %.0f s (depth %.2f m)", kTimeoutS, s.depth);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace banyu_mission::task2
