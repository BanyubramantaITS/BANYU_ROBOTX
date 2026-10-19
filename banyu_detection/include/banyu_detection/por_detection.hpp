#pragma once

#include <banyu_msgs/msg/detection.hpp>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <vector>

namespace por_detection
{

const cv::Scalar CV_YELLOW(0, 215, 255);
const cv::Scalar CV_GREEN(0, 255, 0);
const cv::Scalar CV_GRAY(180, 180, 180);
const cv::Scalar CV_WHITE(255, 255, 255);

struct DetectionCandidate
{
  int objectId {-1};
  int x {0};
  int y {0};
  int width {0};
  int height {0};
  int area {0};

  bool touchesLeft {false};
  bool touchesRight {false};
  bool touchesTop {false};
  bool touchesBottom {false};

  float offsetX {0.0f};
  float offsetY {0.0f};
  float apparentSize {0.0f};
  float confidence {0.0f};
  float distance {0.0f};
  std::string distanceSource {"none"};
};

class PorDetection : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _frontCameraSub;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr _frontCameraCompressedSub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraInfoSub;
  rclcpp::Publisher<banyu_msgs::msg::Detection>::SharedPtr _detectionPub;

  std::vector<DetectionCandidate> candidates;

  std::string priority = "largest";

  double focalX {0.0};
  double focalY {0.0};
  double fallbackFocalLengthPx {0.0};
  double buoyReferenceHeightM {1.5};
  double buoyReferenceWidthM {0.3};
  double gateReferenceHeightM {2.0};
  double gateReferenceWidthM {2.0};
  int edgeMarginPx {5};
  int minHeightPx {10};
  int minWidthPx {10};
  double minApparentSize {0.001};
  double maxDistanceM {20.0};

  bool useGui {};

  int hMin {25};
  int sMin {75};
  int vMin {50};
  int hMax {35};
  int sMax {255};
  int vMax {255};

  int minArea {300};

  int kernelSize {5};

  // Slider values are aspect ratio multiplied by 100.
  int buoyAspectMax {65};
  int gateAspectMin {85};

  void frontCameraCb(sensor_msgs::msg::Image::SharedPtr msg);
  void frontCameraCompressedCb(sensor_msgs::msg::CompressedImage::SharedPtr msg);
  void cameraInfoCb(sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void createSlider();
  void getSliderValue();
  void declareParameters();

public:
  explicit PorDetection(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  float calculateDistance(const DetectionCandidate & candidate) const;
  bool detectObjectFromContours(
    const cv::Mat & origImage, cv::Mat & annotated, const cv::Mat & stats,
    const cv::Mat & centroids, DetectionCandidate & bestCandidate);

  // Setters for unit testing
  void setFocalLengths(double fx, double fy) {focalX = fx; focalY = fy;}
  void setFallbackFocalLength(double f) {fallbackFocalLengthPx = f;}
  void setBuoyDimensions(double h, double w) {buoyReferenceHeightM = h; buoyReferenceWidthM = w;}
  void setGateDimensions(double h, double w) {gateReferenceHeightM = h; gateReferenceWidthM = w;}
  void setEdgeMargin(int margin) {edgeMarginPx = margin;}
  void setMinDimensions(int h, int w, int area) {minHeightPx = h; minWidthPx = w; minArea = area;}
  void setTargetPriority(const std::string & prio) {priority = prio;}
};

}  // namespace por_detection
