#include "banyu_detection/por_detection.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv4/opencv2/opencv.hpp>

namespace por_detection
{

PorDetection::PorDetection(const rclcpp::NodeOptions & options)
: Node("por_detection", options)
{
  this->declareParameters();

  const auto imageTopic = this->declare_parameter<std::string>(
    "image_topic",
    "/front_camera/image_raw");
  _frontCameraSub = this->create_subscription<sensor_msgs::msg::Image>(
    imageTopic, rclcpp::SensorDataQoS(),
    std::bind(&PorDetection::frontCameraCb, this, std::placeholders::_1));

  const auto cameraInfoTopic = this->declare_parameter<std::string>(
    "camera_info_topic",
    "/camera/camera_info");
  _cameraInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    cameraInfoTopic, rclcpp::SensorDataQoS(),
    std::bind(&PorDetection::cameraInfoCb, this, std::placeholders::_1));

  _detectionPub = this->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection",
    rclcpp::SensorDataQoS());

  if (useGui) {this->createSlider();}

  RCLCPP_INFO(this->get_logger(), "por_detection node started");
}

void PorDetection::declareParameters()
{
  priority = this->declare_parameter<std::string>("target_priority", "largest");
  fallbackFocalLengthPx = this->declare_parameter<double>("fallback_focal_length_px", 0.0);
  buoyReferenceHeightM = this->declare_parameter<double>("buoy_reference_height_m", 1.5);
  buoyReferenceWidthM = this->declare_parameter<double>("buoy_reference_width_m", 0.3);
  gateReferenceHeightM = this->declare_parameter<double>("gate_reference_height_m", 2.0);
  gateReferenceWidthM = this->declare_parameter<double>("gate_reference_width_m", 2.0);
  edgeMarginPx = this->declare_parameter<int>("edge_margin_px", 5);
  minHeightPx = this->declare_parameter<int>("min_height_px", 10);
  minWidthPx = this->declare_parameter<int>("min_width_px", 10);
  minApparentSize = this->declare_parameter<double>("min_apparent_size", 0.001);
  maxDistanceM = this->declare_parameter<double>("max_distance_m", 20.0);
  useGui = this->declare_parameter<bool>("use_gui", false);
  minArea = this->declare_parameter<int>("min_area", 300);
  hMin = this->declare_parameter<int>("h_min", 25);
  hMax = this->declare_parameter<int>("h_max", 35);
  sMin = this->declare_parameter<int>("s_min", 75);
  sMax = this->declare_parameter<int>("s_max", 255);
  vMin = this->declare_parameter<int>("v_min", 50);
  vMax = this->declare_parameter<int>("v_max", 255);
  kernelSize = this->declare_parameter<int>("kernel_size", 5);
  buoyAspectMax = this->declare_parameter<int>("buoy_aspect_max", 65);
  gateAspectMin = this->declare_parameter<int>("gate_aspect_min", 85);
}

void PorDetection::frontCameraCb(sensor_msgs::msg::Image::SharedPtr msg)
{
  if (msg->data.empty()) {
    RCLCPP_WARN(this->get_logger(), "Image msg Empty");
    return;
  }

  if (useGui) {this->getSliderValue();}

  cv::Mat origImage = cv_bridge::toCvCopy(msg, "bgr8")->image;
  if (useGui) {
    cv::imshow("Original Frame", origImage);
  }

  cv::Mat blurred {};
  cv::GaussianBlur(origImage, blurred, cv::Size(kernelSize, kernelSize), 0);
  if (useGui) {
    cv::imshow("Blurred Frame", blurred);
  }

  cv::Mat hsvImage {};
  cv::cvtColor(blurred, hsvImage, cv::COLOR_BGR2HSV);
  if (useGui) {
    cv::imshow("HSV Frame", hsvImage);
  }

  cv::Mat lower {{hMin, sMin, vMin}};
  cv::Mat upper {{hMax, sMax, vMax}};
  cv::Mat mask {};
  cv::inRange(hsvImage, lower, upper, mask);
  if (useGui) {
    cv::imshow("Mask Image", mask);
  }

  std::vector<cv::Mat> contours {};
  cv::findContours(
    mask, contours, cv::RetrievalModes::RETR_EXTERNAL,
    cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);

  cv::Mat labels, stats, centroids;
  cv::connectedComponentsWithStats(mask, labels, stats, centroids);

  cv::Mat annotated {origImage.clone()};
  DetectionCandidate bestCandidate;

  bool foundBest = this->detectObjectFromContours(
    origImage, annotated, stats, centroids,
    bestCandidate);
  if (foundBest) {
    banyu_msgs::msg::Detection detection;
    detection.header = msg->header;
    detection.object_id = bestCandidate.objectId;
    detection.apparent_size = bestCandidate.apparentSize;
    detection.confidence = bestCandidate.confidence;
    detection.distance = bestCandidate.distance;
    detection.offset_x = bestCandidate.offsetX;
    detection.offset_y = bestCandidate.offsetY;

    _detectionPub->publish(detection);
  }
  if (useGui) {
    cv::imshow("Annotated", annotated);
    cv::Mat matched {};
    cv::bitwise_and(origImage, origImage, matched, mask);
    cv::drawContours(matched, contours, -1, CV_WHITE);
    cv::imshow("Matched Image", matched);
    cv::waitKey(1);
  }
}

void PorDetection::cameraInfoCb(sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
    return;
  }

  focalX = msg->k[0];
  focalY = msg->k[4];
}

float PorDetection::calculateDistance(const DetectionCandidate & candidate) const
{
  const double fx = focalX > 0.0 ? focalX : fallbackFocalLengthPx;
  const double fy = focalY > 0.0 ? focalY : fallbackFocalLengthPx;
  if ((fx <= 0.0 && fy <= 0.0) || candidate.width <= 0 || candidate.height <= 0) {
    return 0.0f;
  }

  const bool touchesHoriz = candidate.touchesLeft || candidate.touchesRight;
  const bool touchesVert = candidate.touchesTop || candidate.touchesBottom;

  const double refHeight = (candidate.objectId == 1) ? buoyReferenceHeightM : gateReferenceHeightM;
  const double refWidth = (candidate.objectId == 1) ? buoyReferenceWidthM : gateReferenceWidthM;

  double distHeight = 0.0;
  if (fy > 0.0 && candidate.height > 0 && refHeight > 0.0) {
    distHeight = fy * refHeight / static_cast<double>(candidate.height);
  }

  double distWidth = 0.0;
  if (fx > 0.0 && candidate.width > 0 && refWidth > 0.0) {
    distWidth = fx * refWidth / static_cast<double>(candidate.width);
  }

  // 1. Fully visible: neither dimension touches an image boundary.
  if (!touchesHoriz && !touchesVert) {
    if (candidate.objectId == 1) {
      // Slender buoy: height has 5x more pixels, providing superior resolution.
      return distHeight > 0.0 ? static_cast<float>(distHeight) : static_cast<float>(distWidth);
    } else {
      // Wide gate: average height and width estimates if both are valid.
      if (distWidth > 0.0 && distHeight > 0.0) {
        return static_cast<float>(0.5 * (distWidth + distHeight));
      }
      return static_cast<float>(distWidth > 0.0 ? distWidth : distHeight);
    }
  }

  // 2. Vertically truncated (tall marker close to ROV): width is intact.
  if (touchesVert && !touchesHoriz) {
    if (distWidth > 0.0) {
      return static_cast<float>(distWidth);
    }
  }

  // 3. Horizontally truncated (wide gate close to ROV): height is intact.
  if (touchesHoriz && !touchesVert) {
    if (distHeight > 0.0) {
      return static_cast<float>(distHeight);
    }
  }

  // 4. Both dimensions touch borders: both pixel dimensions are truncated (understated),
  // making both distances upper bounds. Return the tighter upper bound.
  if (distWidth > 0.0 && distHeight > 0.0) {
    return static_cast<float>(std::min(distWidth, distHeight));
  }
  return static_cast<float>(std::max(distWidth, distHeight));
}

bool PorDetection::detectObjectFromContours(
  const cv::Mat & origImage, cv::Mat & annotated, const cv::Mat & stats,
  const cv::Mat & centroids, DetectionCandidate & bestCandidate)
{
  candidates.clear();

  for (int i = 1; i < stats.rows; i++) {
    int x = stats.at<int>(i, cv::CC_STAT_LEFT);
    int y = stats.at<int>(i, cv::CC_STAT_TOP);
    int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
    int area = stats.at<int>(i, cv::CC_STAT_AREA);

    if (area < minArea) {continue;}

    const bool touchesLeft = (x <= edgeMarginPx);
    const bool touchesRight = (x + width >= origImage.cols - edgeMarginPx);
    const bool touchesTop = (y <= edgeMarginPx);
    const bool touchesBottom = (y + height >= origImage.rows - edgeMarginPx);

    const bool touchesHoriz = touchesLeft || touchesRight;
    const bool touchesVert = touchesTop || touchesBottom;

    if (height < minHeightPx || width < minWidthPx) {continue;}

    // Reject tiny edge slivers that lack sufficient dimension in the touching direction
    if (touchesHoriz && width < minWidthPx) {continue;}
    if (touchesVert && height < minHeightPx) {continue;}

    const double aspect = static_cast<double>(width) / static_cast<double>(height);
    const double cx = centroids.at<double>(i, 0);
    const double cy = centroids.at<double>(i, 1);
    const float confidence = std::clamp(
      static_cast<float>(area) / static_cast<float>(width * height),
      0.0f, 1.0f);

    const double buoyAspectMaxRatio = this->buoyAspectMax / 100.0;
    const double gateAspectMinRatio = this->gateAspectMin / 100.0;

    bool isBuoy = false;
    bool isGate = false;

    if (!touchesHoriz && !touchesVert) {
      // Unclipped: standard aspect ratio classification
      isBuoy = (aspect <= buoyAspectMaxRatio);
      isGate = (aspect >= gateAspectMinRatio);
      if (!isBuoy && !isGate) {
        if (confidence >= 0.50f && priority != "gate") {
          isBuoy = true;
        } else if (confidence < 0.45f && priority != "buoy") {
          isGate = true;
        }
      }
    } else if (touchesVert && !touchesHoriz) {
      // Vertically truncated, horizontally contained:
      // Object is tall (height spans to screen boundaries), but width is narrow.
      // Even if measured aspect > buoyAspectMaxRatio because height was cut, this is a buoy pole.
      if (width <= origImage.cols * 0.5) {
        isBuoy = true;
      } else {
        isGate = true;
      }
    } else if (touchesHoriz && !touchesVert) {
      // Horizontally truncated, vertically contained:
      // Object is wide (width spans to screen boundaries), but height is contained.
      // Even if measured aspect < gateAspectMinRatio because width was cut, this is a gate.
      isGate = true;
    } else {
      // Both touch edges (object is very close):
      if (priority == "buoy") {
        isBuoy = true;
      } else if (priority == "gate") {
        isGate = true;
      } else {
        if (confidence >= 0.50f) {
          isBuoy = true;
        } else {
          isGate = true;
        }
      }
    }

    if (!isBuoy && !isGate) {continue;}

    double targetCx = cx;
    double targetCy = cy;
    if (isGate) {
      targetCx = (touchesLeft && touchesRight) ? origImage.cols / 2.0 : (x + width / 2.0);
      targetCy = (touchesTop && touchesBottom) ? origImage.rows / 2.0 : (y + height / 2.0);
    } else {
      targetCx = (!touchesHoriz) ? cx : (x + width / 2.0);
      targetCy = (touchesTop && touchesBottom) ? origImage.rows / 2.0 : cy;
    }

    const float offsetX =
      static_cast<float>((targetCx - origImage.cols / 2.0) / (origImage.cols / 2.0));
    const float offsetY =
      static_cast<float>((targetCy - origImage.rows / 2.0) / (origImage.rows / 2.0));

    const float apparentSize = static_cast<float>(area) / (origImage.cols * origImage.rows);
    if (apparentSize < minApparentSize) {continue;}

    DetectionCandidate candidate;
    candidate.objectId = isBuoy ? 1 : 0;
    candidate.x = x;
    candidate.y = y;
    candidate.width = width;
    candidate.height = height;
    candidate.area = area;
    candidate.touchesLeft = touchesLeft;
    candidate.touchesRight = touchesRight;
    candidate.touchesTop = touchesTop;
    candidate.touchesBottom = touchesBottom;
    candidate.offsetX = offsetX;
    candidate.offsetY = offsetY;
    candidate.apparentSize = apparentSize;
    candidate.confidence = confidence;

    if (touchesVert && !touchesHoriz) {
      candidate.distanceSource = "width";
    } else if (touchesHoriz && !touchesVert) {
      candidate.distanceSource = "height";
    } else if (touchesHoriz && touchesVert) {
      candidate.distanceSource = "upper_bound";
    } else {
      candidate.distanceSource = isBuoy ? "height" : "dual";
    }

    candidate.distance = calculateDistance(candidate);

    if (maxDistanceM > 0.0 && candidate.distance > maxDistanceM) {continue;}

    candidates.push_back(candidate);

    const cv::Scalar color = isBuoy ? CV_YELLOW : isGate ? CV_GREEN : CV_GRAY;

    cv::rectangle(annotated, cv::Rect(x, y, width, height), color, 2);
    cv::circle(annotated, cv::Point(cvRound(targetCx), cvRound(targetCy)), 4, color, -1);

    char text[80];
    const char * label = isBuoy ? "buoy" : isGate ? "gate" : "unknown";
    snprintf(
      text, sizeof(text), "%s ar=%.2f d=%.2fm [%s] ox=%.2f oy=%.2f",
      label, aspect, candidate.distance, candidate.distanceSource.c_str(), offsetX, offsetY);
    cv::putText(
      annotated, text, cv::Point(
        x, std::max(
          y - 8,
          12)), cv::FONT_HERSHEY_SIMPLEX, 0.5, color,
      2);

    const int midX = origImage.cols / 2;
    const int midY = origImage.rows / 2;
    const cv::Scalar red(0, 0, 255);
    cv::line(
      annotated, {midX - 20, midY},
      {midX + 20, midY}, red, 2);
    cv::line(
      annotated, {midX, midY - 20},
      {midX, midY + 20}, red, 2);
  }

  bool foundBest = false;

  for (const auto & cd : candidates) {
    if (priority == "gate" && cd.objectId != 0) {continue;}
    if (priority == "buoy" && cd.objectId != 1) {continue;}

    if (!foundBest || cd.area > bestCandidate.area) {
      bestCandidate = cd;
      foundBest = true;
    }
  }

  return foundBest;
}

void PorDetection::createSlider()
{
  cv::namedWindow("Slider");

  cv::createTrackbar("HMin", "Slider", 0, 179);
  cv::createTrackbar("SMin", "Slider", 0, 255);
  cv::createTrackbar("VMin", "Slider", 0, 255);
  cv::createTrackbar("HMax", "Slider", 0, 179);
  cv::createTrackbar("SMax", "Slider", 0, 255);
  cv::createTrackbar("VMax", "Slider", 0, 255);

  cv::createTrackbar("minArea", "Slider", 0, 1000);
  cv::createTrackbar("BuoyARx100", "Slider", 0, 200);
  cv::createTrackbar("GateARx100", "Slider", 0, 300);

  cv::setTrackbarPos("HMin", "Slider", hMin);
  cv::setTrackbarPos("SMin", "Slider", sMin);
  cv::setTrackbarPos("VMin", "Slider", vMin);
  cv::setTrackbarPos("HMax", "Slider", hMax);
  cv::setTrackbarPos("SMax", "Slider", sMax);
  cv::setTrackbarPos("VMax", "Slider", vMax);
  cv::setTrackbarPos("minArea", "Slider", minArea);
  cv::setTrackbarPos("BuoyARx100", "Slider", buoyAspectMax);
  cv::setTrackbarPos("GateARx100", "Slider", gateAspectMin);
}

void PorDetection::getSliderValue()
{
  hMin = cv::getTrackbarPos("HMin", "Slider");
  sMin = cv::getTrackbarPos("SMin", "Slider");
  vMin = cv::getTrackbarPos("VMin", "Slider");

  hMax = cv::getTrackbarPos("HMax", "Slider");
  sMax = cv::getTrackbarPos("SMax", "Slider");
  vMax = cv::getTrackbarPos("VMax", "Slider");

  minArea = cv::getTrackbarPos("minArea", "Slider");
  buoyAspectMax = cv::getTrackbarPos("BuoyARx100", "Slider");
  gateAspectMin = cv::getTrackbarPos("GateARx100", "Slider");
}
}  // namespace por_detection

#ifndef BANYU_DETECTION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<por_detection::PorDetection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
