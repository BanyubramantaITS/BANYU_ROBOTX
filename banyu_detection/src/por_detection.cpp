#include "banyu_detection/por_detection.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/opencv.hpp>

namespace por_detection {

PorDetection::PorDetection() : Node("por_detection") {
    this->declareParameters();

    _frontCameraSub = this->create_subscription<sensor_msgs::msg::Image>(
        "/front_camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&PorDetection::frontCameraCb, this, std::placeholders::_1));

    const auto cameraInfoTopic = this->declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    _cameraInfoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        cameraInfoTopic, rclcpp::SensorDataQoS(), std::bind(&PorDetection::cameraInfoCb, this, std::placeholders::_1));

    _detectionPub = this->create_publisher<banyu_msgs::msg::Detection>("/banyu/detection", rclcpp::SensorDataQoS());

    if (useGui) this->createSlider();

    RCLCPP_INFO(this->get_logger(), "por_detection node started");
}

void PorDetection::declareParameters() {
    priority = this->declare_parameter<std::string>("target_priority", "largest");
    fallbackFocalLengthPx = this->declare_parameter<double>("fallback_focal_length_px", 0.0);
    buoyReferenceHeightM = this->declare_parameter<double>("buoy_reference_height_m", 1.5);
    gateReferenceHeightM = this->declare_parameter<double>("gate_reference_height_m", 2.0);
    edgeMarginPx = this->declare_parameter<int>("edge_margin_px", 5);
    minHeightPx = this->declare_parameter<int>("min_height_px", 10);
    minApparentSize = this->declare_parameter<double>("min_apparent_size", 0.001);
    maxDistanceM = this->declare_parameter<double>("max_distance_m", 20.0);
    useGui = this->declare_parameter<bool>("use_gui", false);
}

void PorDetection::frontCameraCb(sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Image msg Empty");
        return;
    }

    if (useGui) this->getSliderValue();

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

    cv::Mat lower { { hMin, sMin, vMin } };
    cv::Mat upper { { hMax, sMax, vMax } };
    cv::Mat mask {};
    cv::inRange(hsvImage, lower, upper, mask);
    if (useGui) {
        cv::imshow("Mask Image", mask);
    }

    std::vector<cv::Mat> contours {};
    cv::findContours(mask, contours, cv::RetrievalModes::RETR_EXTERNAL,
                     cv::ContourApproximationModes::CHAIN_APPROX_SIMPLE);

    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(mask, labels, stats, centroids);

    cv::Mat annotated { origImage.clone() };
    DetectionCandidate bestCandidate;

    bool foundBest = this->detectObjectFromContours(origImage, annotated, stats, centroids, bestCandidate);
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
    cv::imshow("Annotated", annotated);

    cv::Mat matched {};
    cv::bitwise_and(origImage, origImage, matched, mask);
    cv::drawContours(matched, contours, -1, CV_WHITE);
    if (useGui) {
        cv::imshow("Matched Image", matched);
    }

    cv::waitKey(1);
}

void PorDetection::cameraInfoCb(sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if (msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
        return;
    }

    focalX = msg->k[0];
    focalY = msg->k[4];
}

float PorDetection::calculateDistance(const DetectionCandidate& candidate) const {
    const double focalLength = focalY > 0.0 ? focalY : fallbackFocalLengthPx;
    if (focalLength <= 0.0 || candidate.height <= 0) {
        return 0.0f;
    }

    const double referenceHeight = candidate.objectId == 1 ? buoyReferenceHeightM : gateReferenceHeightM;
    return static_cast<float>(focalLength * referenceHeight / static_cast<double>(candidate.height));
}

bool PorDetection::detectObjectFromContours(const cv::Mat& origImage, cv::Mat& annotated, const cv::Mat& stats,
                                            const cv::Mat& centroids, DetectionCandidate& bestCandidate) {
    candidates.clear();

    for (int i = 1; i < stats.rows; i++) {
        int x = stats.at<int>(i, cv::CC_STAT_LEFT);
        int y = stats.at<int>(i, cv::CC_STAT_TOP);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        int area = stats.at<int>(i, cv::CC_STAT_AREA);

        if (area < minArea) continue;

        // A component touching the image border is only partially visible,
        // so its pixel height cannot produce a valid monocular distance.
        const bool touchesEdge = x <= edgeMarginPx || y <= edgeMarginPx || x + width >= origImage.cols - edgeMarginPx ||
                                 y + height >= origImage.rows - edgeMarginPx;
        if (touchesEdge || height < minHeightPx) continue;

        double aspect = static_cast<double>(width) / static_cast<double>(height);
        double cx = centroids.at<double>(i, 0);
        double cy = centroids.at<double>(i, 1);

        const double buoyAspectMaxRatio = this->buoyAspectMax / 100.0;
        const double gateAspectMinRatio = this->gateAspectMin / 100.0;
        const bool isBuoy = aspect <= buoyAspectMaxRatio;
        const bool isGate = aspect >= gateAspectMinRatio;

        const double targetCx = isGate ? x + width / 2.0 : cx;
        const double targetCy = isGate ? y + height / 2.0 : cy;

        if (!isBuoy && !isGate) continue;

        const float offsetX = static_cast<float>((targetCx - origImage.cols / 2.0) / (origImage.cols / 2.0));
        const float offsetY = static_cast<float>((targetCy - origImage.rows / 2.0) / (origImage.rows / 2.0));

        const float apparentSize = static_cast<float>(area) / (origImage.cols * origImage.rows);
        if (apparentSize < minApparentSize) continue;

        float confidence = std::clamp(static_cast<float>(area) / static_cast<float>(width * height), 0.0f, 1.0f);

        DetectionCandidate candidate;
        candidate.objectId = (isBuoy) ? 1 : (isGate) ? 0 : -1;
        candidate.x = x;
        candidate.y = y;
        candidate.width = width;
        candidate.height = height;
        candidate.area = area;
        candidate.offsetX = offsetX;
        candidate.offsetY = offsetY;
        candidate.apparentSize = apparentSize;
        candidate.confidence = confidence;
        candidate.distance = calculateDistance(candidate);

        // A zero distance means that no camera calibration is available.
        // Otherwise reject estimates outside the configured operating range.
        if (maxDistanceM > 0.0 && candidate.distance > maxDistanceM) continue;

        candidates.push_back(candidate);

        const cv::Scalar color = isBuoy ? CV_YELLOW : isGate ? CV_GREEN : CV_GRAY;

        cv::rectangle(annotated, cv::Rect(x, y, width, height), color, 2);
        cv::circle(annotated, cv::Point(cvRound(targetCx), cvRound(targetCy)), 4, color, -1);

        char text[64];
        const char* label = isBuoy ? "buoy" : isGate ? "gate" : "unknown";
        snprintf(text, sizeof(text), "%s ar=%.2f fill=%.2f d=%.2fm ox=%.2f oy=%.2f", label, aspect, confidence,
                 candidate.distance, offsetX, offsetY);
        cv::putText(annotated, text, cv::Point(x, std::max(y - 8, 12)), cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);

        const cv::Scalar red(0, 0, 255);
        cv::line(annotated, { origImage.cols / 2 - 20, origImage.rows / 2 },
                 { origImage.cols / 2 + 20, origImage.rows / 2 }, red, 1);
        cv::line(annotated, { origImage.cols / 2, origImage.rows / 2 - 20 },
                 { origImage.cols / 2, origImage.rows / 2 + 20 }, red, 1);
    }

    bool foundBest = false;

    for (const auto& cd : candidates) {
        if (priority == "gate" && cd.objectId != 0) continue;
        if (priority == "buoy" && cd.objectId != 1) continue;

        if (!foundBest || cd.area > bestCandidate.area) {
            bestCandidate = cd;
            foundBest = true;
        }
    }

    return foundBest;
}

void PorDetection::createSlider() {
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

void PorDetection::getSliderValue() {
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

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<por_detection::PorDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
