#pragma once

#include <banyu_msgs/msg/detection.hpp>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
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
  int objectId;
  int x;
  int y;
  int width;
  int height;
  int area;

  float offsetX;
  float offsetY;
  float apparentSize;
  float confidence;
  float distance;
};

class PorDetection : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _frontCameraSub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraInfoSub;
  rclcpp::Publisher<banyu_msgs::msg::Detection>::SharedPtr _detectionPub;

  std::vector<DetectionCandidate> candidates;

  std::string priority = "largest";

  double focalX {0.0};
  double focalY {0.0};
  double fallbackFocalLengthPx {0.0};
  double buoyReferenceHeightM {1.5};
  double gateReferenceHeightM {2.0};
  int edgeMarginPx {5};
  int minHeightPx {10};
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
  void cameraInfoCb(sensor_msgs::msg::CameraInfo::SharedPtr msg);
  float calculateDistance(const DetectionCandidate & candidate) const;
  bool detectObjectFromContours(
    const cv::Mat & origImage, cv::Mat & annotated, const cv::Mat & stats,
    const cv::Mat & centroids, DetectionCandidate & bestCandidate);
  void createSlider();
  void getSliderValue();
  void declareParameters();

public:
  PorDetection();
};

}  // namespace por_detection
