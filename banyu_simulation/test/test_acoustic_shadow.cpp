#include <cmath>

#include <gtest/gtest.h>

#include "../src/acoustic_shadow.hpp"

using gz::math::Vector3d;

TEST(AcousticShadow, ClearPathIsStraight) {
  const banyu::ShadowPaths paths({{{-0.1, -1, 0}, {0.1, 1, 0.02}}}, {{0.5, 0, -0.05}});
  bool shadowed = true;
  EXPECT_NEAR(paths.Length({0.5, 0, 10}, 0, &shadowed), 10.05, 1e-9);
  EXPECT_FALSE(shadowed);
}

TEST(AcousticShadow, PlateAboveReceiverDetoursOverItsEdge) {
  // Long thin plate: the shortest way round is over one long edge, straight up its 2 cm side.
  const banyu::ShadowPaths paths({{{-0.1, -1, 0}, {0.1, 1, 0.02}}}, {{0, 0, -0.05}});
  bool shadowed = false;
  const double len = paths.Length({0, 0, 10}, 0, &shadowed);
  EXPECT_TRUE(shadowed);
  EXPECT_NEAR(len, std::hypot(0.1, 0.05) + 0.02 + std::hypot(0.1, 9.98), 1e-9);
}

TEST(AcousticShadow, GrazingAFaceIsNotBlocked) {
  const banyu::Box b{{0, 0, 0}, {1, 1, 1}};
  EXPECT_FALSE(banyu::SegmentHits({-1, 0.5, 1}, {2, 0.5, 1}, b));
  EXPECT_TRUE(banyu::SegmentHits({-1, 0.5, 0.9}, {2, 0.5, 0.9}, b));
}
