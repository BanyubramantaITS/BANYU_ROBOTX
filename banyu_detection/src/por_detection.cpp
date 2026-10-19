#include "banyu_detection/por_detection.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include <cstdlib>
#include <fstream>

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
  _gatePub = this->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection/gate", rclcpp::SensorDataQoS());
  _markerPub = this->create_publisher<banyu_msgs::msg::Detection>(
    "/banyu/detection/marker", rclcpp::SensorDataQoS());

  // Debug views for remote tuning (rqt_image_view / tools/por_hsv_tuner.py).
  // They are only rendered while someone subscribes.
  _debugAnnotatedPub = image_transport::create_publisher(
    this, "/banyu/detection/debug/annotated");
  _debugGateMaskPub = image_transport::create_publisher(
    this, "/banyu/detection/debug/mask_gate");
  _debugMarkerMaskPub = image_transport::create_publisher(
    this, "/banyu/detection/debug/mask_marker");

  _saveService = this->create_service<std_srvs::srv::Trigger>(
    "~/save_params",
    std::bind(&PorDetection::saveParams, this, std::placeholders::_1, std::placeholders::_2));
  _paramCallback = this->add_on_set_parameters_callback(
    std::bind(&PorDetection::onParametersSet, this, std::placeholders::_1));

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
  minApparentSize = this->declare_parameter<double>("min_apparent_size", 0.001);
  maxDistanceM = this->declare_parameter<double>("max_distance_m", 20.0);
  useGui = this->declare_parameter<bool>("use_gui", false);
  kernelSize = this->declare_parameter<int>("kernel_size", 5);
  this->declareTuningParameters();
}

void PorDetection::declareTuningParameters()
{
  colorMode = this->declare_parameter<std::string>("color_mode", colorMode);
  debugRateHz = this->declare_parameter<double>("debug_rate_hz", debugRateHz);
  debugScale = this->declare_parameter<double>("debug_scale", debugScale);
  const char * home = std::getenv("HOME");
  tuningSavePath = this->declare_parameter<std::string>(
    "tuning_save_path",
    std::string(home != nullptr ? home : "/tmp") + "/por_detection_tuned.yaml");

  // Slider-tunable integers (rqt_reconfigure shows IntegerRange params as sliders).
  const auto ranged = [this](const std::string & name, int * target, int low, int high,
      const std::string & description) {
      rcl_interfaces::msg::ParameterDescriptor descriptor;
      descriptor.description = description;
      rcl_interfaces::msg::IntegerRange range;
      range.from_value = low; range.to_value = high; range.step = 1;
      descriptor.integer_range.push_back(range);
      intParams[name] = target;
      *target = static_cast<int>(this->declare_parameter<int>(name, *target, descriptor));
    };
  ranged("gate_h_min", &gateHsv.hMin, 0, 179, "Gate hue min (wraps if > gate_h_max)");
  ranged("gate_h_max", &gateHsv.hMax, 0, 179, "Gate hue max");
  ranged("gate_s_min", &gateHsv.sMin, 0, 255, "Gate saturation min");
  ranged("gate_s_max", &gateHsv.sMax, 0, 255, "Gate saturation max");
  ranged("gate_v_min", &gateHsv.vMin, 0, 255, "Gate value min");
  ranged("gate_v_max", &gateHsv.vMax, 0, 255, "Gate value max");
  ranged("marker_h_min", &markerHsv.hMin, 0, 179, "Marker hue min (wraps if > marker_h_max)");
  ranged("marker_h_max", &markerHsv.hMax, 0, 179, "Marker hue max");
  ranged("marker_s_min", &markerHsv.sMin, 0, 255, "Marker saturation min");
  ranged("marker_s_max", &markerHsv.sMax, 0, 255, "Marker saturation max");
  ranged("marker_v_min", &markerHsv.vMin, 0, 255, "Marker value min");
  ranged("marker_v_max", &markerHsv.vMax, 0, 255, "Marker value max");
  ranged("morph_kernel", &morphKernel, 0, 15, "Mask opening kernel (0 = off)");
  ranged("min_area", &minArea, 0, 20000, "Minimum blob area in pixels");
  ranged("buoy_aspect_max", &buoyAspectMax, 0, 300, "Marker max width/height x100");
  ranged("gate_aspect_min", &gateAspectMin, 0, 300, "Gate min width/height x100 (single mode)");
  ranged("h_min", &hMin, 0, 179, "Single-mode hue min");
  ranged("h_max", &hMax, 0, 179, "Single-mode hue max");
  ranged("s_min", &sMin, 0, 255, "Single-mode saturation min");
  ranged("s_max", &sMax, 0, 255, "Single-mode saturation max");
  ranged("v_min", &vMin, 0, 255, "Single-mode value min");
  ranged("v_max", &vMax, 0, 255, "Single-mode value max");
  ranged("min_height_px", &minHeightPx, 0, 1000, "Minimum blob height in pixels");
  ranged("min_width_px", &minWidthPx, 0, 1000, "Minimum blob width in pixels");
}

rcl_interfaces::msg::SetParametersResult PorDetection::onParametersSet(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    const auto it = intParams.find(name);
    if (it != intParams.end()) {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = name + " must be an integer";
        return result;
      }
      *it->second = static_cast<int>(parameter.as_int());
    } else if (name == "color_mode") {
      const auto mode = parameter.as_string();
      if (mode != "per_object" && mode != "single") {
        result.successful = false;
        result.reason = "color_mode must be per_object or single";
        return result;
      }
      colorMode = mode;
    } else if (name == "debug_rate_hz") {
      debugRateHz = parameter.as_double();
    } else if (name == "debug_scale") {
      debugScale = parameter.as_double();
    } else if (name == "tuning_save_path") {
      tuningSavePath = parameter.as_string();
    } else if (name == "target_priority") {
      priority = parameter.as_string();
    } else if (name == "min_apparent_size") {
      minApparentSize = parameter.as_double();
    } else if (name == "max_distance_m") {
      maxDistanceM = parameter.as_double();
    }
  }
  return result;
}

void PorDetection::saveParams(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::ofstream out(tuningSavePath);
  if (!out) {
    response->success = false;
    response->message = "cannot write " + tuningSavePath;
    return;
  }
  const auto line = [&out](const std::string & key, int value) {
      out << "    " << key << ": " << value << "\n";
    };
  out << "# Saved by por_detection/save_params. Copy these values into\n"
      << "# banyu_detection/config/por_detection_params.yaml.\n"
      << "por_detection:\n  ros__parameters:\n"
      << "    color_mode: \"" << colorMode << "\"\n";
  for (const auto & key : {
      "gate_h_min", "gate_h_max", "gate_s_min", "gate_s_max", "gate_v_min", "gate_v_max",
      "marker_h_min", "marker_h_max", "marker_s_min", "marker_s_max", "marker_v_min",
      "marker_v_max", "morph_kernel", "min_area", "buoy_aspect_max", "gate_aspect_min",
      "h_min", "h_max", "s_min", "s_max", "v_min", "v_max", "min_height_px", "min_width_px"})
  {
    line(key, *intParams.at(key));
  }
  response->success = true;
  response->message = "saved to " + tuningSavePath;
  RCLCPP_INFO(this->get_logger(), "Tuning parameters %s", response->message.c_str());
}

cv::Mat PorDetection::hsvMask(const cv::Mat & hsvImage, const HsvRange & range)
{
  cv::Mat mask;
  if (range.hMin <= range.hMax) {
    cv::inRange(
      hsvImage, cv::Scalar(range.hMin, range.sMin, range.vMin),
      cv::Scalar(range.hMax, range.sMax, range.vMax), mask);
    return mask;
  }
  cv::Mat low, high;
  cv::inRange(
    hsvImage, cv::Scalar(range.hMin, range.sMin, range.vMin),
    cv::Scalar(179, range.sMax, range.vMax), high);
  cv::inRange(
    hsvImage, cv::Scalar(0, range.sMin, range.vMin),
    cv::Scalar(range.hMax, range.sMax, range.vMax), low);
  cv::bitwise_or(low, high, mask);
  return mask;
}

void PorDetection::publishDetection(
  const std_msgs::msg::Header & header, const DetectionCandidate & candidate,
  const rclcpp::Publisher<banyu_msgs::msg::Detection>::SharedPtr & classPub)
{
  banyu_msgs::msg::Detection detection;
  detection.header = header;
  detection.object_id = candidate.objectId;
  detection.apparent_size = candidate.apparentSize;
  detection.confidence = candidate.confidence;
  detection.distance = candidate.distance;
  detection.offset_x = candidate.offsetX;
  detection.offset_y = candidate.offsetY;
  _detectionPub->publish(detection);
  if (classPub) {classPub->publish(detection);}
}

void PorDetection::publishDebug(
  const std_msgs::msg::Header & header, const cv::Mat & annotated,
  const cv::Mat & gateMask, const cv::Mat & markerMask)
{
  if (_debugAnnotatedPub.getNumSubscribers() == 0 && _debugGateMaskPub.getNumSubscribers() == 0 &&
    _debugMarkerMaskPub.getNumSubscribers() == 0)
  {
    return;
  }
  const auto stamp = this->now();
  if (debugRateHz > 0.0 && _lastDebugPublish.nanoseconds() != 0 &&
    (stamp - _lastDebugPublish).seconds() < 1.0 / debugRateHz)
  {
    return;
  }
  _lastDebugPublish = stamp;
  const double scale = (debugScale > 0.0 && debugScale <= 1.0) ? debugScale : 1.0;
  const auto send = [&header, scale](image_transport::Publisher & pub, const cv::Mat & image,
      const std::string & encoding) {
      if (pub.getNumSubscribers() == 0 || image.empty()) {return;}
      cv::Mat scaled;
      cv::resize(image, scaled, cv::Size(), scale, scale, cv::INTER_AREA);
      pub.publish(cv_bridge::CvImage(header, encoding, scaled).toImageMsg());
    };
  send(_debugAnnotatedPub, annotated, "bgr8");
  send(_debugGateMaskPub, gateMask, "mono8");
  send(_debugMarkerMaskPub, markerMask, "mono8");
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

  cv::Mat annotated {origImage.clone()};
  const int midX = origImage.cols / 2;
  const int midY = origImage.rows / 2;
  cv::line(annotated, {midX - 20, midY}, {midX + 20, midY}, cv::Scalar(0, 0, 255), 2);
  cv::line(annotated, {midX, midY - 20}, {midX, midY + 20}, cv::Scalar(0, 0, 255), 2);

  if (colorMode == "per_object") {
    cv::Mat gateMask = hsvMask(hsvImage, gateHsv);
    cv::Mat markerMask = hsvMask(hsvImage, markerHsv);
    if (morphKernel > 1) {
      const auto kernel = cv::getStructuringElement(
        cv::MORPH_RECT, cv::Size(morphKernel, morphKernel));
      cv::morphologyEx(gateMask, gateMask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(markerMask, markerMask, cv::MORPH_OPEN, kernel);
    }
    DetectionCandidate gate;
    DetectionCandidate marker;
    if (detectColorTarget(origImage, annotated, gateMask, 0, gate)) {
      publishDetection(msg->header, gate, _gatePub);
    }
    if (detectColorTarget(origImage, annotated, markerMask, 1, marker)) {
      publishDetection(msg->header, marker, _markerPub);
    }
    publishDebug(msg->header, annotated, gateMask, markerMask);
    if (useGui) {
      cv::imshow("Gate Mask", gateMask);
      cv::imshow("Marker Mask", markerMask);
      cv::imshow("Annotated", annotated);
      cv::waitKey(1);
    }
    return;
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

  DetectionCandidate bestCandidate;

  bool foundBest = this->detectObjectFromContours(
    origImage, annotated, stats, centroids,
    bestCandidate);
  if (foundBest) {
    publishDetection(
      msg->header, bestCandidate, bestCandidate.objectId == 1 ? _markerPub : _gatePub);
  }
  publishDebug(msg->header, annotated, mask, cv::Mat());
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

bool PorDetection::detectColorTarget(
  const cv::Mat & origImage, cv::Mat & annotated, const cv::Mat & mask,
  int objectId, DetectionCandidate & best) const
{
  cv::Mat labels, stats, centroids;
  const int count = cv::connectedComponentsWithStats(mask, labels, stats, centroids);
  const bool isGate = (objectId == 0);

  // Gate: union of every coloured part, so two posts seen without the bar still
  // give the gate centre. Marker: the largest pole-shaped blob.
  cv::Rect box;
  int area = 0;
  bool found = false;
  for (int i = 1; i < count; ++i) {
    const int blobArea = stats.at<int>(i, cv::CC_STAT_AREA);
    if (blobArea < minArea) {continue;}
    const cv::Rect blob(
      stats.at<int>(i, cv::CC_STAT_LEFT), stats.at<int>(i, cv::CC_STAT_TOP),
      stats.at<int>(i, cv::CC_STAT_WIDTH), stats.at<int>(i, cv::CC_STAT_HEIGHT));
    if (isGate) {
      box = found ? (box | blob) : blob;
      area += blobArea;
      found = true;
      continue;
    }
    const bool touchesVert = blob.y <= edgeMarginPx ||
      blob.y + blob.height >= origImage.rows - edgeMarginPx;
    const double aspect = static_cast<double>(blob.width) / std::max(1, blob.height);
    if (!touchesVert && aspect > buoyAspectMax / 100.0) {continue;}
    if (!found || blobArea > area) {
      box = blob;
      area = blobArea;
      found = true;
    }
  }
  if (!found || box.width < minWidthPx || box.height < minHeightPx) {return false;}

  DetectionCandidate candidate;
  candidate.objectId = objectId;
  candidate.x = box.x;
  candidate.y = box.y;
  candidate.width = box.width;
  candidate.height = box.height;
  candidate.area = area;
  candidate.touchesLeft = box.x <= edgeMarginPx;
  candidate.touchesRight = box.x + box.width >= origImage.cols - edgeMarginPx;
  candidate.touchesTop = box.y <= edgeMarginPx;
  candidate.touchesBottom = box.y + box.height >= origImage.rows - edgeMarginPx;
  const bool touchesHoriz = candidate.touchesLeft || candidate.touchesRight;
  const bool touchesVert = candidate.touchesTop || candidate.touchesBottom;

  const double targetCx = (candidate.touchesLeft && candidate.touchesRight) ?
    origImage.cols / 2.0 : box.x + box.width / 2.0;
  const double targetCy = (candidate.touchesTop && candidate.touchesBottom) ?
    origImage.rows / 2.0 : box.y + box.height / 2.0;
  candidate.offsetX = static_cast<float>((targetCx - origImage.cols / 2.0) / (origImage.cols / 2.0));
  candidate.offsetY = static_cast<float>((targetCy - origImage.rows / 2.0) / (origImage.rows / 2.0));
  candidate.apparentSize = static_cast<float>(area) / (origImage.cols * origImage.rows);
  if (candidate.apparentSize < minApparentSize) {return false;}
  candidate.confidence = std::clamp(
    static_cast<float>(area) / static_cast<float>(box.area()), 0.0f, 1.0f);
  if (touchesVert && !touchesHoriz) {
    candidate.distanceSource = "width";
  } else if (touchesHoriz && !touchesVert) {
    candidate.distanceSource = "height";
  } else if (touchesHoriz && touchesVert) {
    candidate.distanceSource = "upper_bound";
  } else {
    candidate.distanceSource = isGate ? "dual" : "height";
  }
  candidate.distance = calculateDistance(candidate);
  if (maxDistanceM > 0.0 && candidate.distance > maxDistanceM) {return false;}

  const cv::Scalar color = isGate ? CV_GREEN : cv::Scalar(0, 140, 255);
  cv::rectangle(annotated, box, color, 2);
  cv::circle(annotated, cv::Point(cvRound(targetCx), cvRound(targetCy)), 4, color, -1);
  char text[80];
  snprintf(
    text, sizeof(text), "%s d=%.2fm [%s] ox=%.2f oy=%.2f", isGate ? "gate" : "marker",
    candidate.distance, candidate.distanceSource.c_str(), candidate.offsetX, candidate.offsetY);
  cv::putText(
    annotated, text, cv::Point(box.x, std::max(box.y - 8, 12)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
    color, 2);
  best = candidate;
  return true;
}

void PorDetection::createSlider()
{
  // Local OpenCV sliders (needs a display). For a headless vehicle use
  // rqt_reconfigure or tools/por_hsv_tuner.py from the laptop instead.
  const auto window = [](const std::string & name, const std::vector<std::pair<std::string, int>> & bars,
      const std::unordered_map<std::string, int *> & values) {
      cv::namedWindow(name, cv::WINDOW_NORMAL);
      for (const auto & [bar, max] : bars) {
        cv::createTrackbar(bar, name, nullptr, max);
        cv::setTrackbarPos(bar, name, *values.at(bar));
      }
    };
  const std::vector<std::pair<std::string, int>> filter {
    {"min_area", 5000}, {"morph_kernel", 15}, {"buoy_aspect_max", 300}, {"gate_aspect_min", 300}};
  if (colorMode == "per_object") {
    window("Gate HSV", {{"gate_h_min", 179}, {"gate_h_max", 179}, {"gate_s_min", 255},
        {"gate_s_max", 255}, {"gate_v_min", 255}, {"gate_v_max", 255}}, intParams);
    window("Marker HSV", {{"marker_h_min", 179}, {"marker_h_max", 179}, {"marker_s_min", 255},
        {"marker_s_max", 255}, {"marker_v_min", 255}, {"marker_v_max", 255}}, intParams);
  } else {
    window("Slider", {{"h_min", 179}, {"h_max", 179}, {"s_min", 255}, {"s_max", 255},
        {"v_min", 255}, {"v_max", 255}}, intParams);
  }
  window("Filter", filter, intParams);
}

void PorDetection::getSliderValue()
{
  const auto read = [this](const std::string & window, const std::vector<std::string> & bars) {
      for (const auto & bar : bars) {
        const int value = cv::getTrackbarPos(bar, window);
        if (value >= 0) {*intParams.at(bar) = value;}
      }
    };
  if (colorMode == "per_object") {
    read("Gate HSV", {"gate_h_min", "gate_h_max", "gate_s_min", "gate_s_max", "gate_v_min",
        "gate_v_max"});
    read("Marker HSV", {"marker_h_min", "marker_h_max", "marker_s_min", "marker_s_max",
        "marker_v_min", "marker_v_max"});
  } else {
    read("Slider", {"h_min", "h_max", "s_min", "s_max", "v_min", "v_max"});
  }
  read("Filter", {"min_area", "morph_kernel", "buoy_aspect_max", "gate_aspect_min"});
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
