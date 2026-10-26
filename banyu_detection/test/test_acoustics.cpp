#include <cmath>

#include <gtest/gtest.h>

#include "banyu_detection/acoustics.hpp"

using namespace banyu_detection;

namespace
{
const std::vector<Vec3> kArray = {{0.15, 0, -0.1}, {-0.15, 0, -0.1}, {0, 0.15, -0.1},
  {0, -0.15, -0.1}};

std::vector<double> arrivals(const Vec3 & src)
{
  std::vector<double> t;
  for (const auto & h : kArray) {
    t.push_back(std::hypot(h[0] - src[0], h[1] - src[1], h[2] - src[2]) / kSoundSpeed);
  }
  return t;
}
}  // namespace

TEST(Acoustics, BearingFromArrivalTimes) {
  // Pinger 3 m ahead, 1 m left, 4 m above a level vehicle.
  const auto u = pingerVector(arrivals({3.0, 1.0, 4.0}), kArray);
  ASSERT_TRUE(u);
  EXPECT_NEAR(u->bearing(), std::atan2(1, 3), 0.02);
  EXPECT_NEAR(std::hypot(u->forward, u->left), 1.0, 1e-9);  // unit vector: no elevation/range
}

TEST(Acoustics, PingerOverheadHasNoBearing) {
  EXPECT_FALSE(pingerVector({0.0, 0.0, 0.0, 0.0}, kArray));
}

TEST(Acoustics, PingerAheadIsAhead) {
  const auto u = pingerVector(arrivals({5.0, 0.0, 10.0}), kArray);
  ASSERT_TRUE(u);
  EXPECT_NEAR(u->forward, 1.0, 1e-6);
  EXPECT_NEAR(u->left, 0.0, 1e-6);
}

TEST(Acoustics, RejectsMismatchedSizes) {
  EXPECT_FALSE(pingerVector({0.0, 0.0}, kArray));
}
