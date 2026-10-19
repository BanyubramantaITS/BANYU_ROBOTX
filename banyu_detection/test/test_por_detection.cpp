#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>

#include "banyu_detection/por_detection.hpp"

namespace por_detection
{
namespace test
{

class PorDetectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    detector_ = std::make_shared<PorDetection>();
    detector_->setFocalLengths(500.0, 500.0);
    detector_->setFallbackFocalLength(500.0);
    detector_->setBuoyDimensions(1.5, 0.3);
    detector_->setGateDimensions(2.0, 2.0);
    detector_->setEdgeMargin(5);
    detector_->setMinDimensions(10, 10, 100);
  }

  void TearDown() override
  {
    detector_.reset();
  }

  std::shared_ptr<PorDetection> detector_;
};

TEST_F(PorDetectionTest, UnclippedBuoyUsesHeightBasedDistance)
{
  DetectionCandidate cand;
  cand.objectId = 1;
  cand.x = 290;
  cand.y = 90;
  cand.width = 60;
  cand.height = 300;
  cand.touchesLeft = false;
  cand.touchesRight = false;
  cand.touchesTop = false;
  cand.touchesBottom = false;

  const float dist = detector_->calculateDistance(cand);
  // fy * H / h = 500.0 * 1.5 / 300.0 = 2.5 m
  EXPECT_NEAR(dist, 2.5f, 0.05f);
}

TEST_F(PorDetectionTest, VerticallyClippedBuoyUsesWidthBasedDistance)
{
  DetectionCandidate cand;
  cand.objectId = 1;
  cand.x = 280;
  cand.y = 0;  // Touching top edge
  cand.width = 75;
  cand.height = 200;  // Truncated height
  cand.touchesLeft = false;
  cand.touchesRight = false;
  cand.touchesTop = true;
  cand.touchesBottom = false;

  const float dist = detector_->calculateDistance(cand);
  // fx * W / w = 500.0 * 0.3 / 75.0 = 2.0 m (NOT 500 * 1.5 / 200 = 3.75 m)
  EXPECT_NEAR(dist, 2.0f, 0.05f);
}

TEST_F(PorDetectionTest, HorizontallyClippedGateUsesHeightBasedDistance)
{
  DetectionCandidate cand;
  cand.objectId = 0;
  cand.x = 0;  // Touching left edge
  cand.y = 40;
  cand.width = 300;  // Truncated width
  cand.height = 400;  // Intact height
  cand.touchesLeft = true;
  cand.touchesRight = false;
  cand.touchesTop = false;
  cand.touchesBottom = false;

  const float dist = detector_->calculateDistance(cand);
  // fy * H / h = 500.0 * 2.0 / 400.0 = 2.5 m
  EXPECT_NEAR(dist, 2.5f, 0.05f);
}

TEST_F(PorDetectionTest, BothEdgesClippedReturnsTighterUpperBound)
{
  DetectionCandidate cand;
  cand.objectId = 1;
  cand.x = 0;
  cand.y = 0;
  cand.width = 100;   // dw = 500 * 0.3 / 100 = 1.5 m
  cand.height = 300;  // dh = 500 * 1.5 / 300 = 2.5 m
  cand.touchesLeft = true;
  cand.touchesRight = false;
  cand.touchesTop = true;
  cand.touchesBottom = false;

  const float dist = detector_->calculateDistance(cand);
  EXPECT_NEAR(dist, 1.5f, 0.05f);
}

TEST_F(PorDetectionTest, DetectsVerticallyClippedBuoyWithoutDisappearing)
{
  // 640x480 image
  cv::Mat origImage = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::Mat mask = cv::Mat::zeros(480, 640, CV_8UC1);

  // Draw a tall, narrow yellow buoy pole spanning y=0 to y=479 (touching both top and bottom edges)
  // x from 300 to 360 (width = 60 px)
  cv::Rect buoyRect(300, 0, 60, 480);
  mask(buoyRect).setTo(255);

  cv::Mat labels, stats, centroids;
  cv::connectedComponentsWithStats(mask, labels, stats, centroids);

  cv::Mat annotated = origImage.clone();
  DetectionCandidate best;
  const bool found = detector_->detectObjectFromContours(
    origImage, annotated, stats, centroids, best);

  ASSERT_TRUE(found);
  EXPECT_EQ(best.objectId, 1);  // Correctly classified as yellow buoy
  EXPECT_TRUE(best.touchesTop);
  EXPECT_TRUE(best.touchesBottom);
  EXPECT_FALSE(best.touchesLeft);
  EXPECT_FALSE(best.touchesRight);
  EXPECT_EQ(best.distanceSource, "width");
  // Distance from width: 500 * 0.3 / 60 = 2.5 m
  EXPECT_NEAR(best.distance, 2.5f, 0.1f);
  // Centering: targetCx is 330, center of 640 is 320 -> offsetX ~ (330-320)/320 = 0.03125
  EXPECT_NEAR(best.offsetX, 0.03125f, 0.02f);
}

TEST_F(PorDetectionTest, DetectsHorizontallyClippedGateWithoutDisappearing)
{
  cv::Mat origImage = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::Mat mask = cv::Mat::zeros(480, 640, CV_8UC1);

  // Draw an inverted U-gate touching left and right edges (x=0 to x=639)
  // Top bar: y=50 to y=70, x=0 to 639
  mask(cv::Rect(0, 50, 640, 20)).setTo(255);
  // Left post: y=50 to y=450, x=0 to 30
  mask(cv::Rect(0, 50, 30, 400)).setTo(255);
  // Right post: y=50 to y=450, x=610 to 639
  mask(cv::Rect(610, 50, 30, 400)).setTo(255);

  cv::Mat labels, stats, centroids;
  cv::connectedComponentsWithStats(mask, labels, stats, centroids);

  cv::Mat annotated = origImage.clone();
  DetectionCandidate best;
  const bool found = detector_->detectObjectFromContours(
    origImage, annotated, stats, centroids, best);

  ASSERT_TRUE(found);
  EXPECT_EQ(best.objectId, 0);  // Correctly classified as gate
  EXPECT_TRUE(best.touchesLeft);
  EXPECT_TRUE(best.touchesRight);
  EXPECT_FALSE(best.touchesTop);
  EXPECT_FALSE(best.touchesBottom);
  EXPECT_EQ(best.distanceSource, "height");
  // Height is 400 px. Distance from height: 500 * 2.0 / 400 = 2.5 m
  EXPECT_NEAR(best.distance, 2.5f, 0.1f);
  // Centering: gate touches both left and right, so targetCx is cols/2 -> offsetX = 0.0
  EXPECT_NEAR(best.offsetX, 0.0f, 0.01f);
}

TEST_F(PorDetectionTest, DiscardsTinyEdgeSlivers)
{
  cv::Mat origImage = cv::Mat::zeros(480, 640, CV_8UC3);
  cv::Mat mask = cv::Mat::zeros(480, 640, CV_8UC1);

  // Draw a 3x3 pixel sliver touching top-left corner
  mask(cv::Rect(0, 0, 3, 3)).setTo(255);

  cv::Mat labels, stats, centroids;
  cv::connectedComponentsWithStats(mask, labels, stats, centroids);

  cv::Mat annotated = origImage.clone();
  DetectionCandidate best;
  const bool found = detector_->detectObjectFromContours(
    origImage, annotated, stats, centroids, best);

  EXPECT_FALSE(found);
}

TEST_F(PorDetectionTest, PerObjectColourSeparatesYellowGateFromOrangeMarker)
{
  // Blue pool background, yellow gate (two posts + top bar), orange marker pole.
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(140, 90, 20));
  const cv::Scalar yellow(0, 230, 255);   // BGR
  const cv::Scalar orange(0, 120, 255);
  cv::rectangle(image, cv::Rect(150, 100, 20, 250), yellow, cv::FILLED);
  cv::rectangle(image, cv::Rect(330, 100, 20, 250), yellow, cv::FILLED);
  cv::rectangle(image, cv::Rect(150, 100, 200, 15), yellow, cv::FILLED);
  cv::rectangle(image, cv::Rect(500, 120, 18, 200), orange, cv::FILLED);

  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  const HsvRange gateRange {20, 35, 80, 255, 80, 255};
  const HsvRange markerRange {5, 18, 120, 255, 100, 255};
  const cv::Mat gateMask = PorDetection::hsvMask(hsv, gateRange);
  const cv::Mat markerMask = PorDetection::hsvMask(hsv, markerRange);

  cv::Mat annotated = image.clone();
  DetectionCandidate gate;
  DetectionCandidate marker;
  ASSERT_TRUE(detector_->detectColorTarget(image, annotated, gateMask, 0, gate));
  ASSERT_TRUE(detector_->detectColorTarget(image, annotated, markerMask, 1, marker));

  // Gate box is the union of both posts and the bar; its centre is x = 250.
  EXPECT_EQ(gate.objectId, 0);
  EXPECT_EQ(gate.width, 200);
  EXPECT_NEAR(gate.offsetX, (250.0 - 320.0) / 320.0, 0.01);
  EXPECT_EQ(marker.objectId, 1);
  EXPECT_EQ(marker.width, 18);
  EXPECT_GT(marker.offsetX, 0.5f);
}

TEST_F(PorDetectionTest, GateUnionKeepsCentreWithoutTopBar)
{
  cv::Mat mask = cv::Mat::zeros(480, 640, CV_8UC1);
  mask(cv::Rect(100, 150, 20, 200)).setTo(255);
  mask(cv::Rect(400, 150, 20, 200)).setTo(255);
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat annotated = image.clone();
  DetectionCandidate gate;
  ASSERT_TRUE(detector_->detectColorTarget(image, annotated, mask, 0, gate));
  EXPECT_NEAR(gate.offsetX, (260.0 - 320.0) / 320.0, 0.01);
}

TEST_F(PorDetectionTest, MarkerRejectsWideBlobOfMarkerColour)
{
  detector_->setAspectLimits(65, 85);
  cv::Mat mask = cv::Mat::zeros(480, 640, CV_8UC1);
  mask(cv::Rect(100, 200, 200, 40)).setTo(255);  // wide, not a pole
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat annotated = image.clone();
  DetectionCandidate marker;
  EXPECT_FALSE(detector_->detectColorTarget(image, annotated, mask, 1, marker));
}

TEST_F(PorDetectionTest, HueRangeWrapsForRedTargets)
{
  cv::Mat image(10, 20, CV_8UC3, cv::Scalar(0, 0, 0));
  image(cv::Rect(0, 0, 10, 10)).setTo(cv::Scalar(0, 0, 255));    // red, hue 0
  image(cv::Rect(10, 0, 10, 10)).setTo(cv::Scalar(255, 0, 0));   // blue, hue 120
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  const cv::Mat mask = PorDetection::hsvMask(hsv, HsvRange {170, 10, 100, 255, 100, 255});
  EXPECT_EQ(cv::countNonZero(mask(cv::Rect(0, 0, 10, 10))), 100);
  EXPECT_EQ(cv::countNonZero(mask(cv::Rect(10, 0, 10, 10))), 0);
}

}  // namespace test
}  // namespace por_detection
