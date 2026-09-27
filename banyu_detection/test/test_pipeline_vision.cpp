#include <cmath>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "banyu_detection/pipeline_vision.hpp"

using namespace banyu_detection;

namespace {
// Water veil, a faint pipe straight up the frame, one GREEN light node on it at row 100.
cv::Mat syntheticFrame() {
  cv::Mat im(240, 320, CV_8UC3, cv::Scalar(2, 24, 1));
  im.colRange(150, 170).setTo(cv::Scalar(2, 32, 1));
  for (int k = 0; k < 8; ++k) {
    const double a = 2 * M_PI * k / 8;
    cv::circle(im, cv::Point(160 + static_cast<int>(3 * std::cos(a)), 100 + static_cast<int>(3 * std::sin(a))), 1,
               cv::Scalar(22, 89, 12), -1);
  }
  return im;
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
  EXPECT_FALSE(lights[0].red);
}

TEST(PipelineVision, NoPipeInPlainVeil) {
  const cv::Mat veil(240, 320, CV_8UC3, cv::Scalar(2, 24, 1));
  EXPECT_FALSE(fitPipe(pipeMask(veil), aheadBand(240)));
  EXPECT_FALSE(pipeCentroid(pipeMask(veil)));
}

TEST(PipelineVision, PipeEndingMidFrameEndsGoingForward) {
  cv::Mat im(240, 320, CV_8UC3, cv::Scalar(2, 24, 1));
  im(cv::Rect(150, 130, 20, 110)).setTo(cv::Scalar(2, 32, 1));  // pipe only in the lower half
  const cv::Mat mask = pipeMask(im);
  EXPECT_TRUE(pipeEndsHere(mask, 1));
  EXPECT_FALSE(pipeEndsHere(mask, -1));
}
