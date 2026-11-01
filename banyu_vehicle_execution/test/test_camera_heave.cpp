#include <gtest/gtest.h>

#include <cmath>

#include "banyu_vehicle_execution/camera_heave.hpp"

using banyu_vehicle_execution::CameraHeave;
using banyu_vehicle_execution::CameraHeaveConfig;

namespace
{
// Width fraction the pipe has from `range` metres.
double widthAt(const CameraHeaveConfig & c, double range)
{
  return c.pipe_diameter_m / (range * 2.0 * std::tan(c.hfov_rad / 2.0));
}
}  // namespace

TEST(CameraHeave, RangeFromTheApparentWidth)
{
  CameraHeave h;
  EXPECT_NEAR(*h.rangeFromWidth(widthAt(h.config(), 1.5)), 1.5, 1e-9);
  EXPECT_FALSE(h.rangeFromWidth(std::nan("")));
  EXPECT_FALSE(h.rangeFromWidth(0.0));
  EXPECT_FALSE(h.rangeFromWidth(widthAt(h.config(), 10.0)));   // beyond what the detector can see
}

TEST(CameraHeave, SinksWhenHighRisesWhenLowHoldsInTheDeadband)
{
  CameraHeave h;
  const auto & c = h.config();
  h.addWidth(widthAt(c, 2.6), 0.0);
  EXPECT_LT(h.command(0.1), 0.0);
  EXPECT_GE(h.command(0.1), -c.max_rate_mps);
  h.reset();
  h.addWidth(widthAt(c, 0.8), 0.0);
  EXPECT_GT(h.command(0.1), 0.0);
  h.reset();
  h.addWidth(widthAt(c, c.standoff_m + 0.05), 0.0);
  EXPECT_NEAR(h.command(0.1), c.trim_mps, 1e-9);   // first tick: no integral yet
}

TEST(CameraHeave, MedianRejectsAWrongWidth)
{
  CameraHeave h;
  const auto & c = h.config();
  for (int i = 0; i < 6; ++i) {h.addWidth(widthAt(c, 1.6), i * 0.04);}
  h.addWidth(widthAt(c, 0.4), 0.25);   // a node frame read as the pipe
  EXPECT_NEAR(*h.range(0.3), 1.6, 1e-9);
}

TEST(CameraHeave, NoRangeWhenTheWidthIsStale)
{
  CameraHeave h;
  h.addWidth(widthAt(h.config(), 2.0), 0.0);
  EXPECT_TRUE(h.range(0.4));
  EXPECT_FALSE(h.range(0.6));
  EXPECT_NEAR(h.command(0.6), h.config().trim_mps, 1e-9);
}

TEST(CameraHeave, IntegralLearnsTheBuoyancyAndKeepsItWhenThePipeIsLost)
{
  // A vehicle that floats up at 0.04 m/s with zero heave: simulate the height and feed the widths it would see.
  CameraHeave h;
  const auto & c = h.config();
  double height = c.standoff_m;
  for (int i = 0; i < 1200; ++i) {   // 60 s at 20 Hz
    const double t = i * 0.05;
    h.addWidth(widthAt(c, height), t);
    height += (h.command(t) + 0.04) * 0.05;
  }
  EXPECT_NEAR(height, c.standoff_m, 0.15);
  EXPECT_NEAR(h.integral(), -0.04, 0.015);
  // Pipe lost: the learned push down stays.
  EXPECT_NEAR(h.command(61.0), -0.04, 0.015);
}
