#pragma once

#include <banyu_msgs/msg/detection.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv4/opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace por_detection
{

const cv::Scalar CV_YELLOW(0, 215, 255);
const cv::Scalar CV_GREEN(0, 255, 0);
const cv::Scalar CV_GRAY(180, 180, 180);
const cv::Scalar CV_WHITE(255, 255, 255);

// HSV threshold for one target colour. When hMin > hMax the hue range wraps
// around 0/180 (needed for red/orange targets).
struct HsvRange
{
  int hMin {0};
  int hMax {179};
  int sMin {0};
  int sMax {255};
  int vMin {0};
  int vMax {255};
};

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
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _cameraInfoSub;
  rclcpp::Publisher<banyu_msgs::msg::Detection>::SharedPtr _detectionPub;
  rclcpp::Publisher<banyu_msgs::msg::Detection>::SharedPtr _gatePub;
  rclcpp::Publisher<banyu_msgs::msg::Detection>::SharedPtr _markerPub;
  image_transport::Publisher _debugAnnotatedPub;
  image_transport::Publisher _debugGateMaskPub;
  image_transport::Publisher _debugMarkerMaskPub;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _saveService;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _paramCallback;
  rclcpp::Time _lastDebugPublish {0, 0, RCL_ROS_TIME};

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

  // "per_object": separate gate/marker colours (gate = yellow, marker = orange by
  // default). "single": legacy one-colour mask classified by shape.
  std::string colorMode {"per_object"};
  HsvRange gateHsv {20, 35, 80, 255, 80, 255};
  HsvRange markerHsv {5, 18, 120, 255, 100, 255};
  int morphKernel {3};

  double debugRateHz {5.0};
  double debugScale {0.5};
  std::string tuningSavePath {};

  std::unordered_map<std::string, int *> intParams;

  void frontCameraCb(sensor_msgs::msg::Image::SharedPtr msg);
  void cameraInfoCb(sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void createSlider();
  void getSliderValue();
  void declareParameters();
  void declareTuningParameters();
  rcl_interfaces::msg::SetParametersResult onParametersSet(
    const std::vector<rclcpp::Parameter> & parameters);
  void saveParams(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void publishDetection(
    const std_msgs::msg::Header & header, const DetectionCandidate & candidate,
    const rclcpp::Publisher<banyu_msgs::msg::Detection>::SharedPtr & classPub);
  void publishDebug(
    const std_msgs::msg::Header & header, const cv::Mat & annotated,
    const cv::Mat & gateMask, const cv::Mat & markerMask);

public:
  explicit PorDetection(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  float calculateDistance(const DetectionCandidate & candidate) const;
  bool detectObjectFromContours(
    const cv::Mat & origImage, cv::Mat & annotated, const cv::Mat & stats,
    const cv::Mat & centroids, DetectionCandidate & bestCandidate);
  // Per-object colour detection: the mask holds only one target colour, so the
  // class is known. The gate is the union of its coloured parts (posts + bar);
  // the marker is the largest coloured blob that is not wider than tall.
  bool detectColorTarget(
    const cv::Mat & origImage, cv::Mat & annotated, const cv::Mat & mask,
    int objectId, DetectionCandidate & best) const;
  static cv::Mat hsvMask(const cv::Mat & hsvImage, const HsvRange & range);

  // Setters for unit testing
  void setFocalLengths(double fx, double fy) {focalX = fx; focalY = fy;}
  void setFallbackFocalLength(double f) {fallbackFocalLengthPx = f;}
  void setBuoyDimensions(double h, double w) {buoyReferenceHeightM = h; buoyReferenceWidthM = w;}
  void setGateDimensions(double h, double w) {gateReferenceHeightM = h; gateReferenceWidthM = w;}
  void setEdgeMargin(int margin) {edgeMarginPx = margin;}
  void setMinDimensions(int h, int w, int area) {minHeightPx = h; minWidthPx = w; minArea = area;}
  void setTargetPriority(const std::string & prio) {priority = prio;}
  void setAspectLimits(int buoyMaxX100, int gateMinX100)
  {
    buoyAspectMax = buoyMaxX100; gateAspectMin = gateMinX100;
  }
};

}  // namespace por_detection
