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
    for (int i = 0; i < 40; ++i, t += 0.05) tracker.update({light(colour, x, 200)}, t);
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

TEST(LedTracker, ConfirmedNodeKeptThoughBrief) {
  LedTracker tracker;
  double t = 0;
  for (int i = 0; i < 5; ++i, t += 0.05) tracker.update({light(LightNode::RED, 160, 80)}, t);  // stopped on it
  tracker.confirm(160, 80);
  t += 40.0;  // repair
  for (int i = 0; i < 100; ++i, t += 0.05) tracker.update({light(LightNode::GREEN, 160, 200)}, t);
  EXPECT_EQ(tracker.nodes(), (std::vector<uint8_t>{LightNode::RED, LightNode::GREEN}));
}

TEST(LedTracker, BriefNodeThatSweepsKeptFragmentDropped) {
  LedTracker tracker;
  double t = 0;
  for (int i = 0; i < 12; ++i, t += 0.05) tracker.update({light(LightNode::GREEN, 160, 170 - 8.0f * i)}, t);  // start node
  t += 2.0;
  for (int i = 0; i < 60; ++i, t += 0.05) tracker.update({light(LightNode::GREEN, 160, 235 - 3.5f * i)}, t);
  t += 1.5;
  for (int i = 0; i < 14; ++i, t += 0.05) tracker.update({light(LightNode::GREEN, 160, 35 - 2.0f * i)}, t);  // edge fragment
  EXPECT_EQ(tracker.nodes(), (std::vector<uint8_t>{LightNode::GREEN, LightNode::GREEN}));
}

TEST(LedTracker, NodeSeenAgainFromTheTopNotCounted) {
  LedTracker tracker;
  double t = 0;
  for (int i = 0; i < 60; ++i, t += 0.05) tracker.update({light(LightNode::GREEN, 160, 235 - 3.5f * i)}, t);
  t += 30.0;  // repair; the vehicle drifts forward while it climbs back
  for (int i = 0; i < 60; ++i, t += 0.05) tracker.update({light(LightNode::GREEN, 160, 2 + 2.0f * i)}, t);
  EXPECT_EQ(tracker.nodes(), (std::vector<uint8_t>{LightNode::GREEN}));
}

TEST(LedTracker, InViewAtResumeNotCountedAtStartCounted) {
  LedTracker tracker;
  tracker.reset(100.0);
  double t = 100.0;
  for (int i = 0; i < 6; ++i, t += 0.05) tracker.update({light(LightNode::GREEN, 160, 60 - 8.0f * i)}, t);  // start node, leaving
  for (int i = 0; i < 60; ++i, t += 0.05) tracker.update({light(LightNode::RED, 160, 235 - 3.5f * i)}, t);
  t += 40.0;
  tracker.resume(t);
  for (int i = 0; i < 60; ++i, t += 0.05) tracker.update({light(LightNode::GREEN, 160, 150 - 2.0f * i)}, t);  // seen before
  EXPECT_EQ(tracker.nodes(), (std::vector<uint8_t>{LightNode::GREEN, LightNode::RED}));
}
