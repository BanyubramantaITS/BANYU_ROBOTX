#include <gtest/gtest.h>

#include <cmath>
#include <cstdlib>
#include <vector>

#include "banyu_detection/resource_code_decoder.hpp"

using banyu_detection::ResourceCodeConfig;
using banyu_detection::ResourceCodeDecoder;

namespace
{
constexpr int kRed = 1, kGreen = 2, kBlue = 3;
}  // namespace

namespace
{

// Post-repair trace like the sim light controller: 5 s steady green, then the code for 30 s.
// `two_colour` selects the Disruptive 5 s cycle, otherwise the Advanced 2 s flash.
struct Trace
{
  double fps {30.0};
  double dropout {0.0};     // probability that a frame is lost (reads as dark)
  double start {7.3};       // arbitrary clock offset
  unsigned seed {1};
  double red_lead_s {2.0};  // the ring is red before the repair (fed before `from` when from is 0)
};

int colourAt(double since, bool two_colour, int tin, int circle)
{
  if (since < 5.0) {return kGreen;}
  const double s = since - 5.0;
  if (s >= 30.0) {return 0;}
  if (two_colour) {
    const double c = std::fmod(s, 5.0);
    if (c < 1.0) {return tin;}
    if (c >= 2.0 && c < 3.0) {return circle;}
    return 0;
  }
  return std::fmod(s, 2.0) < 1.0 ? circle : 0;
}

void run(ResourceCodeDecoder & d, const Trace & tr, bool two_colour, int tin, int circle,
  double from = 0.0, double to = 37.0)
{
  std::srand(tr.seed);
  const double dt = 1.0 / tr.fps;
  if (from <= 0.0) {
    for (double since = -tr.red_lead_s; since < 0.0; since += dt) {d.add(tr.start + since, kRed);}
  }
  for (double since = from; since < to; since += dt) {
    int colour = colourAt(since, two_colour, tin, circle);
    if (tr.dropout > 0.0 && std::rand() / static_cast<double>(RAND_MAX) < tr.dropout) {colour = 0;}
    d.add(tr.start + since, colour);
  }
}

}  // namespace

TEST(ResourceCodeDecoder, DisruptiveAllNinePairs)
{
  for (int tin = kRed; tin <= kBlue; ++tin) {
    for (int circle = kRed; circle <= kBlue; ++circle) {
      ResourceCodeDecoder d;
      run(d, Trace{}, true, tin, circle);
      const auto r = d.decode();
      EXPECT_TRUE(r.complete) << tin << "," << circle;
      EXPECT_EQ(r.tin, tin) << tin << "," << circle;
      EXPECT_EQ(r.circle, circle) << tin << "," << circle;
      EXPECT_GE(r.cycles, 5);
      EXPECT_GT(r.confidence, 0.8f);
    }
  }
}

TEST(ResourceCodeDecoder, DisruptiveSurvivesDroppedFrames)
{
  for (unsigned seed = 1; seed <= 8; ++seed) {
    ResourceCodeDecoder d;
    Trace tr;
    tr.dropout = 0.10;
    tr.seed = seed;
    run(d, tr, true, kBlue, kRed);
    const auto r = d.decode();
    EXPECT_TRUE(r.complete) << seed;
    EXPECT_EQ(r.tin, kBlue);
    EXPECT_EQ(r.circle, kRed);
  }
}

TEST(ResourceCodeDecoder, DisruptiveLowFrameRate)
{
  ResourceCodeDecoder d;
  Trace tr;
  tr.fps = 10.0;
  run(d, tr, true, kRed, kGreen);
  const auto r = d.decode();
  EXPECT_TRUE(r.complete);
  EXPECT_EQ(r.tin, kRed);
  EXPECT_EQ(r.circle, kGreen);
}

TEST(ResourceCodeDecoder, GreenTinMergesWithSteadyGreenButStillDecodes)
{
  // The first cycle is lost in the 5 s steady green; later cycles carry the code.
  ResourceCodeDecoder d;
  run(d, Trace{}, true, kGreen, kBlue);
  const auto r = d.decode();
  EXPECT_TRUE(r.complete);
  EXPECT_EQ(r.tin, kGreen);
  EXPECT_EQ(r.circle, kBlue);
}

TEST(ResourceCodeDecoder, ObservationStartingMidCodeDoesNotInventTin)
{
  // Starting in the middle of a cycle (no steady green, no known cycle start) must not swap tin and
  // circle: the first half cycle has no long gap before it and is skipped.
  ResourceCodeConfig cfg;
  cfg.require_repair_verified = false;   // the window opened after the repair
  ResourceCodeDecoder d(cfg);
  run(d, Trace{}, true, kRed, kBlue, 7.5, 37.0);   // starts on the circle flash of cycle 1
  const auto r = d.decode();
  EXPECT_TRUE(r.complete);
  EXPECT_EQ(r.tin, kRed);
  EXPECT_EQ(r.circle, kBlue);
}

TEST(ResourceCodeDecoder, AdvancedReadsOnlyTheCircle)
{
  ResourceCodeConfig cfg;
  cfg.code_length = 1;
  for (int circle = kRed; circle <= kBlue; ++circle) {
    ResourceCodeDecoder d(cfg);
    run(d, Trace{}, false, 0, circle);
    const auto r = d.decode();
    EXPECT_TRUE(r.complete) << circle;
    EXPECT_EQ(r.circle, circle);
    EXPECT_EQ(r.tin, 0);
    EXPECT_GE(r.cycles, 10);
  }
}

TEST(ResourceCodeDecoder, SteadyGreenVerifiesTheRepair)
{
  ResourceCodeDecoder d;
  run(d, Trace{}, true, kRed, kBlue);
  EXPECT_TRUE(d.decode().repair_verified);

  // Only flashes (no steady green) is not proof of a repair.
  ResourceCodeDecoder e;
  run(e, Trace{}, true, kRed, kBlue, 5.0, 37.0);
  EXPECT_FALSE(e.decode().repair_verified);

  // Steady green without an earlier red (a neighbouring node) is not a repair.
  ResourceCodeDecoder g;
  Trace no_red;
  no_red.red_lead_s = 0.0;
  run(g, no_red, true, kRed, kBlue);
  EXPECT_FALSE(g.decode().repair_verified);

  // A ring that stays red never verifies.
  ResourceCodeDecoder f;
  for (double t = 0.0; t < 10.0; t += 1.0 / 30.0) {f.add(t, kRed);}
  EXPECT_FALSE(f.decode().repair_verified);
}

TEST(ResourceCodeDecoder, TooShortIsIncomplete)
{
  ResourceCodeDecoder d;
  run(d, Trace{}, true, kRed, kGreen, 0.0, 11.0);   // steady green and one and a bit cycle
  const auto r = d.decode();
  EXPECT_FALSE(r.complete);
}

TEST(ResourceCodeDecoder, SteadyGreenAndDarkGiveNothing)
{
  ResourceCodeDecoder d;
  for (double t = 0.0; t < 10.0; t += 1.0 / 30.0) {d.add(t, kGreen);}
  EXPECT_FALSE(d.decode().complete);
  d.reset();
  for (double t = 0.0; t < 10.0; t += 1.0 / 30.0) {d.add(t, 0);}
  const auto r = d.decode();
  EXPECT_FALSE(r.complete);
  EXPECT_EQ(r.cycles_seen, 0);
}

TEST(ResourceCodeDecoder, OutOfOrderAndEmptyInputAreSafe)
{
  ResourceCodeDecoder d;
  EXPECT_FALSE(d.decode().complete);
  d.add(1.0, kRed);
  EXPECT_FALSE(d.decode().complete);

  // The same samples fed back to front decode to the same code.
  ResourceCodeDecoder forward;
  run(forward, Trace{}, true, kRed, kBlue);
  std::vector<banyu_detection::CodeSample> all;
  {
    ResourceCodeDecoder probe;
    const Trace tr;
    const double dt = 1.0 / tr.fps;
    for (double since = 0.0; since < 37.0; since += dt) {
      all.push_back({tr.start + since, colourAt(since, true, kRed, kBlue)});
    }
  }
  ResourceCodeConfig lenient;
  lenient.require_repair_verified = false;   // the reversed samples have no red lead
  ResourceCodeDecoder reversed(lenient);
  for (auto it = all.rbegin(); it != all.rend(); ++it) {reversed.add(it->t, it->colour);}
  const auto a = forward.decode();
  const auto b = reversed.decode();
  EXPECT_TRUE(b.complete);
  EXPECT_EQ(a.tin, b.tin);
  EXPECT_EQ(a.circle, b.circle);
}

TEST(ResourceCodeDecoder, SteadyRedWithFlickerAndNoRepairIsNotACode)
{
  // A damaged ring that is never repaired: long red, with short red/dark flicker that looks like flashes.
  ResourceCodeConfig cfg;
  cfg.code_length = 1;
  ResourceCodeDecoder d(cfg);
  double t = 1.0;
  for (int i = 0; i < 180; ++i, t += 1.0 / 30.0) {d.add(t, kRed);}           // steady red, 6 s
  for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, 0);}
  for (int cycle = 0; cycle < 12; ++cycle) {                                   // flicker
    for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, kRed);}
    for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, 0);}
  }
  const auto r = d.decode();
  EXPECT_FALSE(r.complete);
  EXPECT_FALSE(r.repair_verified);
}

TEST(ResourceCodeDecoder, ShortRedRunsAtTheStartOfTheWindowAreNotACode)
{
  // The window has just opened on a damaged ring that flickers in and out of view: 1 s of red, a gap, 1 s of red ...
  ResourceCodeConfig cfg;
  cfg.code_length = 1;
  ResourceCodeDecoder d(cfg);
  double t = 1.0;
  for (int cycle = 0; cycle < 4; ++cycle) {
    for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, kRed);}
    for (int i = 0; i < 20; ++i, t += 1.0 / 30.0) {d.add(t, 0);}
  }
  const auto r = d.decode();
  EXPECT_FALSE(r.complete);
}

TEST(ResourceCodeDecoder, NeighbouringGreenNodeFlickerIsNotACode)
{
  // The window looks at a node that was never repaired (the detector reports a neighbouring GREEN node):
  // only green, flickering in and out of view in 1 s runs.
  ResourceCodeConfig cfg;
  cfg.code_length = 1;
  ResourceCodeDecoder d(cfg);
  double t = 1.0;
  for (int cycle = 0; cycle < 6; ++cycle) {
    for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, kGreen);}
    for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, 0);}
  }
  EXPECT_FALSE(d.decode().complete);
}

TEST(ResourceCodeDecoder, AdvancedBlueFlashWithGreenInTheDarkHalfIsBlue)
{
  // The dark half of each flash reads GREEN (the lid's centre): blue, green, blue, green ...
  ResourceCodeConfig cfg;
  cfg.code_length = 1;
  ResourceCodeDecoder d(cfg);
  double t = 1.0;
  for (int i = 0; i < 60; ++i, t += 1.0 / 30.0) {d.add(t, kRed);}
  for (int i = 0; i < 150; ++i, t += 1.0 / 30.0) {d.add(t, kGreen);}      // steady green 5 s
  for (int cycle = 0; cycle < 8; ++cycle) {
    for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, kBlue);}
    for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, kGreen);}
  }
  const auto r = d.decode();
  EXPECT_TRUE(r.complete);
  EXPECT_EQ(r.circle, kBlue);
}

TEST(ResourceCodeDecoder, FlashesLongAfterTheRepairAreNotTheCode)
{
  // Advanced seed 5: the flash was missed (the vehicle drifted off the lid), and 90 s later the camera saw the
  // neighbouring node's steady lid flicker GREEN in and out of view. That is not the code.
  ResourceCodeConfig cfg;
  cfg.code_length = 1;
  ResourceCodeDecoder d(cfg);
  double t = 1.0;
  for (int i = 0; i < 90; ++i, t += 1.0 / 30.0) {d.add(t, kRed);}         // the damaged ring, 3 s
  for (int i = 0; i < 150; ++i, t += 1.0 / 30.0) {d.add(t, kGreen);}      // steady green 5 s: repaired
  for (int i = 0; i < 30 * 90; ++i, t += 1.0 / 30.0) {d.add(t, 0);}       // 90 s of nothing
  for (int cycle = 0; cycle < 6; ++cycle) {
    for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, kGreen);}
    for (int i = 0; i < 30; ++i, t += 1.0 / 30.0) {d.add(t, 0);}
  }
  const auto r = d.decode();
  EXPECT_TRUE(r.repair_verified);
  EXPECT_FALSE(r.complete);
}
