#include "banyu_mission/task2/read_flash.hpp"

#include <string>
#include <tuple>

#include <banyu_msgs/msg/resource_delivery_request.hpp>

namespace banyu_mission::task2 {

namespace {
constexpr double kGateM = 0.15;
constexpr double kTimeoutS = 40.0;  // 5 s GREEN + 30 s of flashing, with margin
constexpr double kPress = 0.1;      // as RepairLight: keeps the tip down on the switch

uint8_t toRequestColour(uint8_t light_colour) {
  using banyu_msgs::msg::LightNode;
  using Req = banyu_msgs::msg::ResourceDeliveryRequest;
  return light_colour == LightNode::RED ? Req::COLOR_RED : light_colour == LightNode::BLUE ? Req::COLOR_BLUE : Req::COLOR_GREEN;
}

const char* colourName(uint8_t light_colour) {
  using banyu_msgs::msg::LightNode;
  return light_colour == LightNode::RED ? "RED" : light_colour == LightNode::BLUE ? "BLUE" : "GREEN";
}
}  // namespace

BT::NodeStatus ReadFlash::begin() {
  decoder_ = FlashDecoder();
  servo_.reset();
  disruptive_ = node_->get_parameter("task2_tier").as_string() == "disruptive";
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReadFlash::step(const Snapshot& s) {
  const auto fix = lightUnderProbe(s, kGateM);
  double fwd = 0.0, right = 0.0;
  if (fix) std::tie(fwd, right) = servo_.update(*fix, elapsed());
  sendSticks(*ctx_, fwd, right, kPress);
  if (newFrame(s)) decoder_.update(fix ? std::optional<uint8_t>(fix->light.colour) : std::nullopt, elapsed());

  // Done as soon as it is read: nothing asks to wait out the 30 s of flashing, and the survey leg
  // after this never counts a node that was in view when it starts (LedTracker::resume).
  using Req = banyu_msgs::msg::ResourceDeliveryRequest;
  std::optional<std::pair<uint8_t, uint8_t>> read;  // (resource, circle) as Req colours
  if (disruptive_) {
    if (const auto p = decoder_.pair()) {
      read = std::make_pair(toRequestColour(p->first), toRequestColour(p->second));
      RCLCPP_INFO(logger(), "flash: %s tin to the %s circle after %.0f s", colourName(p->first), colourName(p->second), elapsed());
    }
  } else if (const auto c = decoder_.colour()) {
    read = std::make_pair(Req::COLOR_ANY, toRequestColour(*c));
    RCLCPP_INFO(logger(), "flash: any tin to the %s circle after %.0f s", colourName(*c), elapsed());
  }
  if (read) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->resource_colour = read->first;
    ctx_->delivery_circle_colour = read->second;
    return BT::NodeStatus::SUCCESS;
  }
  if (elapsed() > kTimeoutS) {
    RCLCPP_ERROR(logger(), "flash: nothing read in %.0f s", kTimeoutS);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

}  // namespace banyu_mission::task2
