#pragma once

#include <banyu_msgs/msg/detection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace por_detection {
class PorDetection : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _frontCameraSub;

    int hMin { 25 };
    int sMin { 75 };
    int vMin { 50 };
    int hMax { 35 };
    int sMax { 255 };
    int vMax { 255 };

    void frontCameraCb(sensor_msgs::msg::Image::SharedPtr msg);
    void createHSVSlider();
    void getHSVSliderValue();

public:
    PorDetection();
};

}  // namespace por_detection
