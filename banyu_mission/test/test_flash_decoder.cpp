#include <gtest/gtest.h>

#include <banyu_msgs/msg/light_node.hpp>

#include "banyu_mission/task2/flash_decoder.hpp"

using banyu_mission::task2::FlashDecoder;
using banyu_msgs::msg::LightNode;

namespace {
// The repaired light at 30 fps: GREEN 5 s, then the tier's pattern for 30 s, frames dropped every
// 7th. second < 0: Advanced (first, 1 s on / 1 s off); else Disruptive (first, off, second, off off).
FlashDecoder watch(uint8_t first, int second, double seconds) {
  FlashDecoder d;
  for (int f = 0; f < seconds * 30; ++f) {
    const double t = f / 30.0, since = t - 5.0;
    std::optional<uint8_t> seen;
    if (since < 0) {
      seen = LightNode::GREEN;
    } else if (since < 30) {
      const int s = static_cast<int>(since);
      if (second < 0 && s % 2 == 0) seen = first;
      if (second >= 0 && s % 5 == 0) seen = first;
      if (second >= 0 && s % 5 == 2) seen = static_cast<uint8_t>(second);
    }
    if (f % 7 == 3) seen.reset();
    d.update(seen, t);
  }
  return d;
}
}  // namespace

TEST(FlashDecoder, ReadsEachColour) {
  for (const uint8_t c : {LightNode::RED, LightNode::GREEN, LightNode::BLUE}) {
    EXPECT_EQ(watch(c, -1, 12.0).colour(), c);
  }
}

TEST(FlashDecoder, NeedsTwoFlashes) {
  EXPECT_FALSE(watch(LightNode::BLUE, -1, 6.5).colour());  // steady green + one flash
}

TEST(FlashDecoder, ReadsEachDisruptivePair) {
  for (const uint8_t a : {LightNode::RED, LightNode::GREEN, LightNode::BLUE})
    for (const uint8_t b : {LightNode::RED, LightNode::GREEN, LightNode::BLUE}) {
      const auto p = watch(a, b, 22.0).pair();
      ASSERT_TRUE(p) << int(a) << " " << int(b);
      EXPECT_EQ(*p, std::make_pair(a, b));
    }
}

TEST(FlashDecoder, DisruptiveNeedsTwoCycles) {
  EXPECT_FALSE(watch(LightNode::RED, LightNode::BLUE, 12.5).pair());
}
