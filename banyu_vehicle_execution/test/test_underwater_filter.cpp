#include <gtest/gtest.h>

#include <cmath>

#include "banyu_vehicle_execution/underwater_filter.hpp"

using banyu_vehicle_execution::UnderwaterFilter;

TEST(UnderwaterFilter, DeadReckonsWithGain)
{
  UnderwaterFilter::Params p;
  p.velocity_gain = 0.9;
  UnderwaterFilter f(p);
  f.reset(1.0);
  for (int i = 0; i < 100; ++i) {f.predict(1.0, 0.0, 0.1);}
  EXPECT_NEAR(f.x(), 9.0, 1e-9);
  EXPECT_NEAR(f.y(), 0.0, 1e-9);
}

TEST(UnderwaterFilter, UncertaintyGrowsWhileBlind)
{
  UnderwaterFilter f(UnderwaterFilter::Params{});
  f.reset(1.0);
  const double before = f.covXX();
  for (int i = 0; i < 100; ++i) {f.predict(0.5, 0.0, 0.1);}
  EXPECT_GT(f.covXX(), before);
}

TEST(UnderwaterFilter, BearingsPullEstimateTowardTruth)
{
  UnderwaterFilter::Params p;
  p.bearing_sigma_rad = 0.05;
  UnderwaterFilter f(p);
  f.reset(1.0);
  // Believes it moved 6 m east, true position is (4, 3). Pinger at (10, 0).
  for (int i = 0; i < 60; ++i) {f.predict(1.0, 0.0, 0.1);}
  const double px = 10.0, py = 0.0, tx = 4.0, ty = 3.0;
  const double err_before = std::hypot(f.x() - tx, f.y() - ty);
  for (int i = 0; i < 20; ++i) {f.updateBearing(std::atan2(py - ty, px - tx), px, py);}
  EXPECT_LT(std::hypot(f.x() - tx, f.y() - ty), err_before);
}

TEST(UnderwaterFilter, WrapsInnovationAcrossPi)
{
  UnderwaterFilter f(UnderwaterFilter::Params{});
  f.reset(1.0);
  // Pinger due west; measured bearing -pi+0.01 vs predicted +pi: innovation is ~0.01, not 2pi.
  EXPECT_TRUE(f.updateBearing(-M_PI + 0.01, -10.0, 0.0));
  EXPECT_NEAR(f.x(), 0.0, 0.5);
}

TEST(UnderwaterFilter, RejectsOutlierAndTooClose)
{
  UnderwaterFilter f(UnderwaterFilter::Params{});
  f.reset(0.5);
  EXPECT_FALSE(f.updateBearing(M_PI / 2.0, 10.0, 0.0));  // 90 deg off, gated
  EXPECT_FALSE(f.updateBearing(0.0, 0.5, 0.0));          // inside min_range
}
