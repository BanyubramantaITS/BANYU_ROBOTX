#include <gtest/gtest.h>

#include "banyu_mission/task2/led_tracker.hpp"

using banyu_mission::task2::LedTracker;
using banyu_msgs::msg::LightNode;

namespace {
LightNode light(uint8_t colour, float x, float y) {
  LightNode l;
  l.colour = colour;
  l.x = x;
  l.y = y;
  return l;
}
}  // namespace

TEST(LedTracker, NodesInPipeOrderAndFlickerDropped) {
  LedTracker tracker;
  double t = 0;
  for (const auto& [colour, x] : {std::pair<uint8_t, float>{LightNode::GREEN, 50}, {LightNode::RED, 150}, {LightNode::GREEN, 250}}) {
    for (int i = 0; i < 40; ++i, t += 0.05) tracker.update({light(colour, x, 120)}, t);
    t += 2.0;  // each track expires before the next node arrives
  }
  tracker.update({light(LightNode::RED, 300, 10)}, t);  // one-frame noise blob
  EXPECT_EQ(tracker.nodes(), (std::vector<uint8_t>{LightNode::GREEN, LightNode::RED, LightNode::GREEN}));
}

TEST(LedTracker, RingFragmentsJoinOneTrack) {
  LedTracker tracker;
  for (int i = 0; i < 30; ++i) tracker.update({light(LightNode::GREEN, 160, 100), light(LightNode::GREEN, 185, 105)}, i * 0.05);
  EXPECT_EQ(tracker.tracks().size(), 1u);
}
