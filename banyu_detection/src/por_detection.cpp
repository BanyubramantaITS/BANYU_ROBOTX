#include "banyu_detection/por_detection.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv4/opencv2/opencv.hpp>

namespace por_detection {

PorDetection::PorDetection() : Node("por_detection") {
    _frontCameraSub = this->create_subscription<sensor_msgs::msg::Image>(
        "/front_camera/image_raw", rclcpp::SensorDataQoS(),
        std::bind(&PorDetection::frontCameraCb, this, std::placeholders::_1));

    this->createHSVSlider();

    RCLCPP_INFO(this->get_logger(), "por_detection node started");
}

void PorDetection::frontCameraCb(sensor_msgs::msg::Image::SharedPtr msg) {
    if (msg->data.empty()) {
        RCLCPP_WARN(this->get_logger(), "Image msg Empty");
        rclcpp::shutdown();
    }
    cv::Mat origImage = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::imshow("Original Frame", origImage);

    cv::Mat blurred {};
    cv::GaussianBlur(origImage, blurred, cv::Size(5, 5), 0);
    cv::imshow("Blurred Frame", blurred);

    this->getHSVSliderValue();

    cv::Mat hsvImage {};
    cv::cvtColor(blurred, hsvImage, cv::COLOR_BGR2HSV);
    cv::imshow("HSV Frame", hsvImage);

    cv::Mat lower { { hMin, sMin, vMin } };
    cv::Mat upper { { hMax, sMax, vMax } };
    cv::Mat mask {};
    cv::inRange(hsvImage, lower, upper, mask);

    // morphological opening (removes small objects from the foreground)
    cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));

    // morphological closing (removes small holes from the foreground)
    cv::dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::erode(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::imshow("Mask Image", mask);

    cv::Mat matched {};
    cv::bitwise_and(origImage, origImage, matched, mask);
    cv::imshow("Matched Image", matched);

    cv::waitKey(1);
}

void PorDetection::createHSVSlider() {
    cv::namedWindow("HSVSlider");

    cv::createTrackbar("HMin", "HSVSlider", 0, 179);
    cv::createTrackbar("SMin", "HSVSlider", 0, 255);
    cv::createTrackbar("VMin", "HSVSlider", 0, 255);
    cv::createTrackbar("HMax", "HSVSlider", 0, 179);
    cv::createTrackbar("SMax", "HSVSlider", 0, 255);
    cv::createTrackbar("VMax", "HSVSlider", 0, 255);

    (hMin != 179) ? cv::setTrackbarPos("HMin", "HSVSlider", hMin) : cv::setTrackbarPos("HMin", "HSVSlider", 179);
    (sMin != 255) ? cv::setTrackbarPos("SMin", "HSVSlider", sMin) : cv::setTrackbarPos("SMin", "HSVSlider", 255);
    (vMin != 255) ? cv::setTrackbarPos("VMin", "HSVSlider", vMin) : cv::setTrackbarPos("VMin", "HSVSlider", 255);
    (hMax != 179) ? cv::setTrackbarPos("HMax", "HSVSlider", hMax) : cv::setTrackbarPos("HMax", "HSVSlider", 179);
    (sMax != 255) ? cv::setTrackbarPos("SMax", "HSVSlider", sMax) : cv::setTrackbarPos("SMax", "HSVSlider", 255);
    (vMax != 255) ? cv::setTrackbarPos("VMax", "HSVSlider", vMax) : cv::setTrackbarPos("VMax", "HSVSlider", 255);
}

void PorDetection::getHSVSliderValue() {
    hMin = cv::getTrackbarPos("HMin", "HSVSlider");
    sMin = cv::getTrackbarPos("SMin", "HSVSlider");
    vMin = cv::getTrackbarPos("VMin", "HSVSlider");

    hMax = cv::getTrackbarPos("HMax", "HSVSlider");
    sMax = cv::getTrackbarPos("SMax", "HSVSlider");
    vMax = cv::getTrackbarPos("VMax", "HSVSlider");
}
}  // namespace por_detection

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<por_detection::PorDetection>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
