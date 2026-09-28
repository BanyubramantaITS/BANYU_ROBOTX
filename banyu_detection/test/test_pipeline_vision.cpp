#include <cmath>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "banyu_detection/pipeline_vision.hpp"

using namespace banyu_detection;

namespace {
// Camera noise: flat synthetic frames have no texture to find edges in, a real sensor always has.
cv::Mat noisy(cv::Mat im) {
  cv::RNG rng(1);
  cv::Mat noise(im.size(), CV_16SC3);
  rng.fill(noise, cv::RNG::NORMAL, 0, 1.5);
  cv::Mat out;
  im.convertTo(out, CV_16SC3);
  out += noise;
  out.convertTo(out, CV_8UC3);
  return out;
}

// Water veil, a faint pipe straight up the frame, one GREEN light node on it at row 100.
cv::Mat syntheticFrame(cv::Scalar led = cv::Scalar(22, 89, 12)) {
  cv::Mat im(240, 320, CV_8UC3, cv::Scalar(2, 24, 1));
  im.colRange(150, 170).setTo(cv::Scalar(2, 32, 1));
  for (int k = 0; k < 8; ++k) {
    const double a = 2 * M_PI * k / 8;
    cv::circle(im, cv::Point(160 + static_cast<int>(3 * std::cos(a)), 100 + static_cast<int>(3 * std::sin(a))), 1,
               led, -1);
  }
  return noisy(im);
}
}  // namespace

TEST(PipelineVision, FitsPipeStraightAhead) {
  const auto fit = fitPipe(pipeMask(syntheticFrame()), aheadBand(240));
  ASSERT_TRUE(fit);
  EXPECT_NEAR(fit->angle, 0.0, 0.05);
  EXPECT_NEAR(fit->lateral, 0.0, 5.0);
}

TEST(PipelineVision, OneGreenNodeOnThePipe) {
  const cv::Mat im = syntheticFrame();
  const auto lights = detectLights(im, pipeMask(im));
  ASSERT_EQ(lights.size(), 1u);
  EXPECT_EQ(lights[0].colour, banyu_detection::Light::kGreen);
}

TEST(PipelineVision, BlueFlashOnThePipe) {
  const cv::Mat im = syntheticFrame(cv::Scalar(90, 45, 8));
  const auto lights = detectLights(im, pipeMask(im));
  ASSERT_EQ(lights.size(), 1u);
  EXPECT_EQ(lights[0].colour, banyu_detection::Light::kBlue);
}

TEST(PipelineVision, NoLightBesideTheProbeRod) {
  // Black rod covering most of the lit pipe: the sliver left beside it is no light.
  cv::Mat im(240, 320, CV_8UC3, cv::Scalar(2, 24, 1));
  im.colRange(150, 170).setTo(cv::Scalar(10, 60, 5));
  im(cv::Rect(146, 0, 20, 75)).setTo(cv::Scalar(1, 3, 1));
  cv::circle(im, cv::Point(156, 75), 10, cv::Scalar(1, 3, 1), -1);  // the magnet at its tip
  EXPECT_TRUE(detectLights(im, pipeMask(im)).empty());
}

TEST(PipelineVision, FitsDarkPipe) {
  // Grey pipe as a silhouette against the lit water, and the probe rod above it ignored.
  cv::Mat im(240, 320, CV_8UC3, cv::Scalar(2, 24, 1));
  im.colRange(150, 170).setTo(cv::Scalar(1, 14, 1));
  im(cv::Rect(152, 0, 16, 78)).setTo(cv::Scalar(1, 3, 1));
  const auto fit = fitPipe(pipeMask(noisy(im), cv::Rect(148, 0, 24, 82)), behindBand(240));
  ASSERT_TRUE(fit);
  EXPECT_NEAR(fit->angle, 0.0, 0.05);
  EXPECT_NEAR(fit->lateral, 0.0, 5.0);
}

TEST(PipelineVision, StandBaseBelowIsNotPipe) {
  // Dark pipe, and a stand's base on the seabed below it: same colour, but thinner (further away).
  cv::Mat im(240, 320, CV_8UC3, cv::Scalar(2, 24, 1));
  cv::line(im, cv::Point(40, 60), cv::Point(120, 200), cv::Scalar(1, 16, 1), 6);
  im.colRange(150, 170).setTo(cv::Scalar(1, 14, 1));
  const cv::Mat mask = pipeMask(noisy(im));
  EXPECT_EQ(cv::countNonZero(mask.colRange(0, 130)), 0);
  EXPECT_GT(cv::countNonZero(mask.colRange(150, 170)), 1000);
}

TEST(PipelineVision, NoPipeInPlainVeil) {
  const cv::Mat veil = noisy(cv::Mat(240, 320, CV_8UC3, cv::Scalar(2, 24, 1)));
  EXPECT_FALSE(fitPipe(pipeMask(veil), aheadBand(240)));
  EXPECT_FALSE(pipeCentroid(pipeMask(veil)));
}

TEST(PipelineVision, PipeEndingMidFrameEndsGoingForward) {
  cv::Mat im(240, 320, CV_8UC3, cv::Scalar(2, 24, 1));
  im(cv::Rect(150, 130, 20, 110)).setTo(cv::Scalar(2, 32, 1));  // pipe only in the lower half
  const cv::Mat mask = pipeMask(noisy(im));
  EXPECT_TRUE(pipeEndsHere(mask, 1));
  EXPECT_FALSE(pipeEndsHere(mask, -1));
}

TEST(PipelineVision, NodeFrameBarsAreNotPipe) {
  // Two thin dark bars of a node frame a pipe's width apart: water between them, not a body.
  cv::Mat im(240, 320, CV_8UC3, cv::Scalar(2, 24, 1));
  im(cv::Rect(100, 40, 5, 160)).setTo(cv::Scalar(1, 14, 1));
  im(cv::Rect(122, 40, 5, 160)).setTo(cv::Scalar(1, 14, 1));
  EXPECT_EQ(cv::countNonZero(pipeMask(noisy(im))), 0);
}
