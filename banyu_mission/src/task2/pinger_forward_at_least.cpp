#include "banyu_mission/task2/pinger_forward_at_least.hpp"

namespace banyu_mission::task2 {

BT::NodeStatus PingerForwardAtLeast::begin() {
  drainPings(*ctx_);
  pings_.clear();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PingerForwardAtLeast::step(const Snapshot& s) {
  const auto pull = pullOverPipe(s.pipe).value_or(std::make_pair(0.0, 0.0));
  sendSticks(*ctx_, pull.first, pull.second, depthRate(s.depth_target, s.depth));
  for (auto& p : drainPings(*ctx_)) pings_.push_back(p);

  const int wanted = getInput<int>("pings").value_or(3);
  if (static_cast<int>(pings_.size()) < wanted && elapsed() < 4.0 * wanted) return BT::NodeStatus::RUNNING;

  double forward = 0, left = 0;
  for (const auto& p : pings_) {
    forward += p.forward / pings_.size();
    left += p.left / pings_.size();
  }
  RCLCPP_INFO(logger(), "pinger: %zu pings, forward=%+.2f left=%+.2f", pings_.size(), forward, left);
  return forward >= getInput<double>("min_forward").value_or(0.0) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace banyu_mission::task2
