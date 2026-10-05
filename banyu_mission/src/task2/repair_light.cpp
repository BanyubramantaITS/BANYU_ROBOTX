#include "banyu_mission/task2/repair_light.hpp"

#include <tuple>
#include <cmath>

namespace banyu_mission::task2 {

namespace {
constexpr double kGateM = 0.3;
// Lower only while the switch is inside a cone down to the tip: kAlignedM at contact, wider by
// kConeSlope per metre of height still to go.
constexpr double kAlignedM = 0.015;
constexpr double kConeSlope = 0.15;
// Keep lowering this long after the last aligned sighting: long while far (the ring is seen in
// some frames only), short near contact so the tip never comes down on a stale fix.
constexpr double kAlignedHoldS = 0.3;
constexpr double kAlignedHoldPerM = 1.0;
// STABILIZE: the vertical stick is thrust. The same small push lowers the vehicle and, once the
// tip is on the switch, holds it there; nothing winds up while it is blocked.
constexpr double kPress = 0.1;
constexpr double kLiftBelowM = 0.05;  // this close to contact height and off the switch: lift
constexpr int kGreenFrames = 10;
// The switch only works under the tip: GREEN anywhere else is some other light.
constexpr double kOnSwitchM = 0.08, kOnSwitchAboveM = 0.1;
constexpr double kLostS = 15.0;
constexpr double kTimeoutS = 120.0;
}  // namespace

BT::NodeStatus RepairLight::begin() {
  seen_s_ = 0;
  aligned_s_ = -1;
  aligned_z_ = 0;
  green_frames_ = 0;
  servo_.reset();
  if (!repair_pub_) {
    repair_pub_ = node_->create_publisher<std_msgs::msg::String>("/robotx/pipeline/repair_damaged", 10);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RepairLight::step(const Snapshot& s) {
  if (elapsed() > kTimeoutS) {
    RCLCPP_ERROR(logger(), "repair: no GREEN after %.0f s", kTimeoutS);
    return BT::NodeStatus::FAILURE;
  }
  // Seen from more than ~1 m the ring is a few single-pixel dots, detected in some frames only.
  const auto fix = lightUnderProbe(s, kGateM);
  const double drop = param("task2_probe_drop");
  double fwd = 0.0, right = 0.0, down = 0.0;
  bool lift = false;
  if (fix) {
    const double off = std::hypot(fix->fwd, fix->right);
    if (newFrame(s)) {
      const bool on_switch = off < kOnSwitchM && fix->z < drop + kOnSwitchAboveM;
      if (on_switch && repair_pub_) {
        std_msgs::msg::String msg;
        msg.data = "2.5";
        repair_pub_->publish(msg);
      }
      green_frames_ = on_switch && fix->light.colour == banyu_msgs::msg::LightNode::GREEN ? green_frames_ + 1 : 0;
      if (green_frames_ >= kGreenFrames) {
        RCLCPP_INFO(logger(), "repair: light is GREEN after %.0f s", elapsed());
        return BT::NodeStatus::SUCCESS;
      }
    }
    seen_s_ = elapsed();
    if (off < kAlignedM + kConeSlope * (fix->z - drop)) {
      aligned_s_ = elapsed();
      aligned_z_ = fix->z;
    }
    std::tie(fwd, right) = servo_.update(*fix, elapsed());
    // Down beside the node instead of on it (the rod then jams on the housing): back up and retry.
    lift = fix->z < drop + kLiftBelowM && off > 2 * kAlignedM;
  } else if (elapsed() - seen_s_ > kLostS) {
    RCLCPP_ERROR(logger(), "repair: light lost");
    return BT::NodeStatus::FAILURE;
  }

  if (lift)
    down = -2 * kPress;
  else if (aligned_s_ >= 0 && elapsed() - aligned_s_ < kAlignedHoldS + kAlignedHoldPerM * (aligned_z_ - drop))
    down = kPress;
  if (fix)
    RCLCPP_INFO_THROTTLE(logger(), *node_->get_clock(), 1000,
                         "repair: switch %+.2f fwd %+.2f right of the tip, camera %.2f m above, depth %.2f%s", fix->fwd,
                         fix->right, fix->z, s.depth, down > 0 ? ", lowering" : down < 0 ? ", lifting" : "");
  sendSticks(*ctx_, fwd, right, down);
  return BT::NodeStatus::RUNNING;
}

}  // namespace banyu_mission::task2
