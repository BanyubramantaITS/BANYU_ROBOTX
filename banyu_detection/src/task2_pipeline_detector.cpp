#include <cv_bridge/cv_bridge.hpp>

#include <algorithm>
#include <banyu_msgs/msg/pipeline_observation.hpp>
#include <banyu_msgs/msg/pipeline_ring_observation.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "banyu_detection/pipeline_vision.hpp"

namespace task2_pipeline_detector
{

struct PipelineLine
{
  cv::Vec4f line {};
  cv::Point2f point {};
  double length {0.0};
  double residual {0.0};
  double aspect {0.0};
  double offset_x {0.0};
  double offset_y {0.0};
  double heading {0.0};
  double confidence {0.0};
};

struct RingCandidate
{
  int color {banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN};
  cv::Point2f center {};
  double area {0.0};
  double confidence {0.0};
};

class Task2PipelineDetector : public rclcpp::Node
{
public:
  Task2PipelineDetector()
  : Node("task2_pipeline_detector")
  {
    declareParameters();

    observation_pub_ = create_publisher<banyu_msgs::msg::PipelineObservation>(
      observation_topic_,
      rclcpp::SensorDataQoS());
    ring_observation_pub_ = create_publisher<banyu_msgs::msg::PipelineRingObservation>(
      ring_observation_topic_, rclcpp::SensorDataQoS());

    if (image_topic_.empty()) {
      throw std::invalid_argument("image_topic must not be empty");
    }
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&Task2PipelineDetector::imageCallback, this, std::placeholders::_1));

    if (use_camera_info_ && !camera_info_topic_.empty()) {
      camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        std::bind(&Task2PipelineDetector::cameraInfoCallback, this, std::placeholders::_1));
    }

    if (use_gui_) {
      createSlider();
    }

    RCLCPP_INFO(
      get_logger(), "task2_pipeline_detector listening on %s (GUI: %s)",
      image_topic_.c_str(), use_gui_ ? "enabled" : "disabled");
  }

  ~Task2PipelineDetector() override
  {
    if (use_gui_) {
      cv::destroyAllWindows();
    }
  }

private:
  void declareParameters()
  {
    use_gui_ = declare_parameter<bool>("use_gui", false);
    image_topic_ = declare_parameter<std::string>("image_topic", "/front_camera/image_raw");
    camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
    use_camera_info_ = declare_parameter<bool>("use_camera_info", true);
    observation_topic_ = declare_parameter<std::string>(
      "observation_topic",
      "/banyu/pipeline/observation");
    ring_observation_topic_ = declare_parameter<std::string>(
      "ring_observation_topic",
      "/banyu/pipeline/ring_observation");

    const auto rect = declare_parameter<std::vector<int64_t>>("probe_rect", std::vector<int64_t>{});
    if (rect.size() == 4) {
      probe_rect_ = cv::Rect(static_cast<int>(rect[0]), static_cast<int>(rect[1]),
                             static_cast<int>(rect[2]), static_cast<int>(rect[3]));
    }

    gray_threshold_ = declare_parameter<int>("gray_threshold", 170);
    hsv_value_threshold_ = declare_parameter<int>("hsv_value_threshold", 150);
    hsv_saturation_max_ = declare_parameter<int>("hsv_saturation_max", 125);
    hsv_h_min_ = declare_parameter<int>("hsv_h_min", 0);
    hsv_s_min_ = declare_parameter<int>("hsv_s_min", 0);
    hsv_v_min_ = declare_parameter<int>("hsv_v_min", hsv_value_threshold_);
    hsv_h_max_ = declare_parameter<int>("hsv_h_max", 179);
    hsv_s_max_ = declare_parameter<int>("hsv_s_max", hsv_saturation_max_);
    hsv_v_max_ = declare_parameter<int>("hsv_v_max", 255);
    use_top_hat_ = declare_parameter<bool>("use_top_hat", true);
    top_hat_kernel_size_ = declare_parameter<int>("top_hat_kernel_size", 31);
    top_hat_threshold_ = declare_parameter<int>("top_hat_threshold", 12);
    blur_kernel_size_ = declare_parameter<int>("blur_kernel_size", 3);
    morphology_kernel_size_ = declare_parameter<int>("morphology_kernel_size", 5);
    morphology_open_iterations_ = declare_parameter<int>("morphology_open_iterations", 1);
    morphology_close_iterations_ = declare_parameter<int>("morphology_close_iterations", 2);
    roi_y_min_ratio_ = declare_parameter<double>("roi_y_min_ratio", 0.05);
    roi_y_max_ratio_ = declare_parameter<double>("roi_y_max_ratio", 0.98);

    min_component_area_px_ = declare_parameter<int>("min_component_area_px", 20);
    min_line_length_px_ = declare_parameter<double>("min_line_length_px", 18.0);
    min_aspect_ratio_ = declare_parameter<double>("min_aspect_ratio", 1.5);
    max_line_residual_px_ = declare_parameter<double>("max_line_residual_px", 10.0);
    max_heading_error_rad_ = declare_parameter<double>("max_heading_error_rad", M_PI_2);
    lookahead_y_ratio_ = declare_parameter<double>("lookahead_y_ratio", 0.65);
    local_fit_band_ratio_ = declare_parameter<double>("local_fit_band_ratio", 0.2);
    lookahead_extrapolation_ratio_ =
      declare_parameter<double>("lookahead_extrapolation_ratio", 0.35);
    min_pipeline_confidence_ = declare_parameter<double>("min_pipeline_confidence", 0.20);
    confidence_length_scale_px_ = declare_parameter<double>("confidence_length_scale_px", 100.0);
    confidence_area_scale_px_ = declare_parameter<double>("confidence_area_scale_px", 500.0);

    red_ring_h_low_min_ = declare_parameter<int>("red_ring_h_low_min", 0);
    red_ring_h_low_max_ = declare_parameter<int>("red_ring_h_low_max", 15);
    red_ring_h_high_min_ = declare_parameter<int>("red_ring_h_high_min", 165);
    red_ring_h_high_max_ = declare_parameter<int>("red_ring_h_high_max", 179);
    red_ring_s_min_ = declare_parameter<int>("red_ring_s_min", 40);
    red_ring_s_max_ = declare_parameter<int>("red_ring_s_max", 255);
    red_ring_v_min_ = declare_parameter<int>("red_ring_v_min", 30);
    red_ring_v_max_ = declare_parameter<int>("red_ring_v_max", 255);
    ring_min_component_area_px_ = declare_parameter<int>("ring_min_component_area_px", 8);
    red_ring_min_area_ = declare_parameter<int>("red_ring_min_area", ring_min_component_area_px_);

    green_ring_h_min_ = declare_parameter<int>("green_ring_h_min", 35);
    green_ring_h_max_ = declare_parameter<int>("green_ring_h_max", 95);
    green_ring_s_min_ = declare_parameter<int>("green_ring_s_min", 40);
    green_ring_s_max_ = declare_parameter<int>("green_ring_s_max", 255);
    green_ring_v_min_ = declare_parameter<int>("green_ring_v_min", 30);
    green_ring_v_max_ = declare_parameter<int>("green_ring_v_max", 255);
    green_ring_min_area_ =
      declare_parameter<int>("green_ring_min_area", ring_min_component_area_px_);

    ring_morphology_kernel_size_ = declare_parameter<int>("ring_morphology_kernel_size", 3);
    ring_open_iterations_ = declare_parameter<int>("ring_open_iterations", 1);
    ring_close_iterations_ = declare_parameter<int>("ring_close_iterations", 1);
    ring_area_confidence_scale_px_ =
      declare_parameter<double>("ring_area_confidence_scale_px", 80.0);
    ring_association_distance_px_ =
      declare_parameter<double>("ring_association_distance_px", 120.0);
    ring_max_aspect_ratio_ = declare_parameter<double>("ring_max_aspect_ratio", 4.5);
    ring_min_confidence_ = declare_parameter<double>("ring_min_confidence", 0.15);

    normalizeParameters();
  }

  static int oddKernel(int value, int fallback)
  {
    value = value > 0 ? value : fallback;
    return value % 2 == 0 ? value + 1 : value;
  }

  void normalizeParameters()
  {
    gray_threshold_ = std::clamp(gray_threshold_, 0, 255);
    hsv_value_threshold_ = std::clamp(hsv_value_threshold_, 0, 255);
    hsv_saturation_max_ = std::clamp(hsv_saturation_max_, 0, 255);
    hsv_h_min_ = std::clamp(hsv_h_min_, 0, 179);
    hsv_h_max_ = std::clamp(hsv_h_max_, 0, 179);
    hsv_s_min_ = std::clamp(hsv_s_min_, 0, 255);
    hsv_s_max_ = std::clamp(hsv_s_max_, 0, 255);
    hsv_v_min_ = std::clamp(hsv_v_min_, 0, 255);
    hsv_v_max_ = std::clamp(hsv_v_max_, 0, 255);
    top_hat_threshold_ = std::clamp(top_hat_threshold_, 0, 255);
    red_ring_h_low_min_ = std::clamp(red_ring_h_low_min_, 0, 179);
    red_ring_h_low_max_ = std::clamp(red_ring_h_low_max_, 0, 179);
    red_ring_h_high_min_ = std::clamp(red_ring_h_high_min_, 0, 179);
    red_ring_h_high_max_ = std::clamp(red_ring_h_high_max_, 0, 179);
    red_ring_s_min_ = std::clamp(red_ring_s_min_, 0, 255);
    red_ring_s_max_ = std::clamp(red_ring_s_max_, 0, 255);
    red_ring_v_min_ = std::clamp(red_ring_v_min_, 0, 255);
    red_ring_v_max_ = std::clamp(red_ring_v_max_, 0, 255);
    red_ring_min_area_ = std::max(1, red_ring_min_area_);

    green_ring_h_min_ = std::clamp(green_ring_h_min_, 0, 179);
    green_ring_h_max_ = std::clamp(green_ring_h_max_, 0, 179);
    green_ring_s_min_ = std::clamp(green_ring_s_min_, 0, 255);
    green_ring_s_max_ = std::clamp(green_ring_s_max_, 0, 255);
    green_ring_v_min_ = std::clamp(green_ring_v_min_, 0, 255);
    green_ring_v_max_ = std::clamp(green_ring_v_max_, 0, 255);
    green_ring_min_area_ = std::max(1, green_ring_min_area_);

    top_hat_kernel_size_ = oddKernel(top_hat_kernel_size_, 31);
    blur_kernel_size_ = oddKernel(blur_kernel_size_, 3);
    morphology_kernel_size_ = oddKernel(morphology_kernel_size_, 5);
    ring_morphology_kernel_size_ = oddKernel(ring_morphology_kernel_size_, 3);
    morphology_open_iterations_ = std::max(0, morphology_open_iterations_);
    morphology_close_iterations_ = std::max(0, morphology_close_iterations_);
    ring_open_iterations_ = std::max(0, ring_open_iterations_);
    ring_close_iterations_ = std::max(0, ring_close_iterations_);
    min_component_area_px_ = std::max(1, min_component_area_px_);
    ring_min_component_area_px_ = std::max(1, ring_min_component_area_px_);
    min_line_length_px_ = std::max(1.0, min_line_length_px_);
    min_aspect_ratio_ = std::max(1.0, min_aspect_ratio_);
    max_line_residual_px_ = std::max(0.1, max_line_residual_px_);
    max_heading_error_rad_ = std::clamp(max_heading_error_rad_, 0.1, M_PI);
    confidence_length_scale_px_ = std::max(1.0, confidence_length_scale_px_);
    confidence_area_scale_px_ = std::max(1.0, confidence_area_scale_px_);
    ring_area_confidence_scale_px_ = std::max(1.0, ring_area_confidence_scale_px_);
    ring_association_distance_px_ = std::max(0.1, ring_association_distance_px_);
    ring_max_aspect_ratio_ = std::max(1.0, ring_max_aspect_ratio_);
    lookahead_y_ratio_ = std::clamp(lookahead_y_ratio_, 0.0, 1.0);
    local_fit_band_ratio_ = std::clamp(local_fit_band_ratio_, 0.0, 0.5);
    lookahead_extrapolation_ratio_ = std::max(0.0, lookahead_extrapolation_ratio_);
    roi_y_min_ratio_ = std::clamp(roi_y_min_ratio_, 0.0, 1.0);
    roi_y_max_ratio_ = std::clamp(roi_y_max_ratio_, roi_y_min_ratio_, 1.0);
    min_pipeline_confidence_ = std::clamp(min_pipeline_confidence_, 0.0, 1.0);
    ring_min_confidence_ = std::clamp(ring_min_confidence_, 0.0, 1.0);
  }

  void createSlider()
  {
    cv::namedWindow("Slider");
    cv::createTrackbar("HMin", "Slider", nullptr, 179);
    cv::createTrackbar("SMin", "Slider", nullptr, 255);
    cv::createTrackbar("VMin", "Slider", nullptr, 255);
    cv::createTrackbar("HMax", "Slider", nullptr, 179);
    cv::createTrackbar("SMax", "Slider", nullptr, 255);
    cv::createTrackbar("VMax", "Slider", nullptr, 255);
    cv::createTrackbar("GrayThresh", "Slider", nullptr, 255);
    cv::createTrackbar("minArea", "Slider", nullptr, 1000);
    cv::createTrackbar("minLineLen", "Slider", nullptr, 500);
    cv::createTrackbar("AspectX10", "Slider", nullptr, 100);

    cv::setTrackbarPos("HMin", "Slider", hsv_h_min_);
    cv::setTrackbarPos("SMin", "Slider", hsv_s_min_);
    cv::setTrackbarPos("VMin", "Slider", hsv_v_min_);
    cv::setTrackbarPos("HMax", "Slider", hsv_h_max_);
    cv::setTrackbarPos("SMax", "Slider", hsv_s_max_);
    cv::setTrackbarPos("VMax", "Slider", hsv_v_max_);
    cv::setTrackbarPos("GrayThresh", "Slider", gray_threshold_);
    cv::setTrackbarPos("minArea", "Slider", min_component_area_px_);
    cv::setTrackbarPos("minLineLen", "Slider", static_cast<int>(min_line_length_px_));
    cv::setTrackbarPos("AspectX10", "Slider", static_cast<int>(min_aspect_ratio_ * 10.0));

    cv::namedWindow("Slider - Red Ring");
    cv::createTrackbar("HLowMax", "Slider - Red Ring", nullptr, 179);
    cv::createTrackbar("HHighMin", "Slider - Red Ring", nullptr, 179);
    cv::createTrackbar("SMin", "Slider - Red Ring", nullptr, 255);
    cv::createTrackbar("SMax", "Slider - Red Ring", nullptr, 255);
    cv::createTrackbar("VMin", "Slider - Red Ring", nullptr, 255);
    cv::createTrackbar("VMax", "Slider - Red Ring", nullptr, 255);
    cv::createTrackbar("minArea", "Slider - Red Ring", nullptr, 500);

    cv::setTrackbarPos("HLowMax", "Slider - Red Ring", red_ring_h_low_max_);
    cv::setTrackbarPos("HHighMin", "Slider - Red Ring", red_ring_h_high_min_);
    cv::setTrackbarPos("SMin", "Slider - Red Ring", red_ring_s_min_);
    cv::setTrackbarPos("SMax", "Slider - Red Ring", red_ring_s_max_);
    cv::setTrackbarPos("VMin", "Slider - Red Ring", red_ring_v_min_);
    cv::setTrackbarPos("VMax", "Slider - Red Ring", red_ring_v_max_);
    cv::setTrackbarPos("minArea", "Slider - Red Ring", red_ring_min_area_);

    cv::namedWindow("Slider - Green Ring");
    cv::createTrackbar("HMin", "Slider - Green Ring", nullptr, 179);
    cv::createTrackbar("HMax", "Slider - Green Ring", nullptr, 179);
    cv::createTrackbar("SMin", "Slider - Green Ring", nullptr, 255);
    cv::createTrackbar("SMax", "Slider - Green Ring", nullptr, 255);
    cv::createTrackbar("VMin", "Slider - Green Ring", nullptr, 255);
    cv::createTrackbar("VMax", "Slider - Green Ring", nullptr, 255);
    cv::createTrackbar("minArea", "Slider - Green Ring", nullptr, 500);

    cv::setTrackbarPos("HMin", "Slider - Green Ring", green_ring_h_min_);
    cv::setTrackbarPos("HMax", "Slider - Green Ring", green_ring_h_max_);
    cv::setTrackbarPos("SMin", "Slider - Green Ring", green_ring_s_min_);
    cv::setTrackbarPos("SMax", "Slider - Green Ring", green_ring_s_max_);
    cv::setTrackbarPos("VMin", "Slider - Green Ring", green_ring_v_min_);
    cv::setTrackbarPos("VMax", "Slider - Green Ring", green_ring_v_max_);
    cv::setTrackbarPos("minArea", "Slider - Green Ring", green_ring_min_area_);
  }

  void getSliderValue()
  {
    hsv_h_min_ = cv::getTrackbarPos("HMin", "Slider");
    hsv_s_min_ = cv::getTrackbarPos("SMin", "Slider");
    hsv_v_min_ = cv::getTrackbarPos("VMin", "Slider");
    hsv_h_max_ = cv::getTrackbarPos("HMax", "Slider");
    hsv_s_max_ = cv::getTrackbarPos("SMax", "Slider");
    hsv_v_max_ = cv::getTrackbarPos("VMax", "Slider");
    gray_threshold_ = cv::getTrackbarPos("GrayThresh", "Slider");
    min_component_area_px_ = std::max(1, cv::getTrackbarPos("minArea", "Slider"));
    min_line_length_px_ =
      std::max(1.0, static_cast<double>(cv::getTrackbarPos("minLineLen", "Slider")));
    const int aspect_x10 = cv::getTrackbarPos("AspectX10", "Slider");
    min_aspect_ratio_ = std::max(1.0, static_cast<double>(aspect_x10) / 10.0);

    red_ring_h_low_max_ = cv::getTrackbarPos("HLowMax", "Slider - Red Ring");
    red_ring_h_high_min_ = cv::getTrackbarPos("HHighMin", "Slider - Red Ring");
    red_ring_s_min_ = cv::getTrackbarPos("SMin", "Slider - Red Ring");
    red_ring_s_max_ = cv::getTrackbarPos("SMax", "Slider - Red Ring");
    red_ring_v_min_ = cv::getTrackbarPos("VMin", "Slider - Red Ring");
    red_ring_v_max_ = cv::getTrackbarPos("VMax", "Slider - Red Ring");
    red_ring_min_area_ = std::max(1, cv::getTrackbarPos("minArea", "Slider - Red Ring"));

    green_ring_h_min_ = cv::getTrackbarPos("HMin", "Slider - Green Ring");
    green_ring_h_max_ = cv::getTrackbarPos("HMax", "Slider - Green Ring");
    green_ring_s_min_ = cv::getTrackbarPos("SMin", "Slider - Green Ring");
    green_ring_s_max_ = cv::getTrackbarPos("SMax", "Slider - Green Ring");
    green_ring_v_min_ = cv::getTrackbarPos("VMin", "Slider - Green Ring");
    green_ring_v_max_ = cv::getTrackbarPos("VMax", "Slider - Green Ring");
    green_ring_min_area_ = std::max(1, cv::getTrackbarPos("minArea", "Slider - Green Ring"));
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    if (!msg || msg->width == 0 || msg->height == 0 || !std::isfinite(msg->k[0]) ||
      !std::isfinite(msg->k[2]) || !std::isfinite(msg->k[4]) ||
      !std::isfinite(msg->k[5]) || msg->k[0] <= 0.0 || msg->k[4] <= 0.0)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(camera_mutex_);
    camera_width_ = msg->width;
    camera_height_ = msg->height;
    camera_cx_ = msg->k[2];
    camera_cy_ = msg->k[5];
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    const uint64_t sequence = ++frame_sequence_;
    if (!msg) {
      return;
    }

    banyu_msgs::msg::PipelineObservation observation = makeLostObservation(*msg, sequence);
    banyu_msgs::msg::PipelineRingObservation ring = makeLostRingObservation(*msg, sequence);

    try {
      if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
        throw std::runtime_error("empty image");
      }
      if (!isSupportedEncoding(msg->encoding)) {
        throw std::runtime_error("unsupported image encoding: " + msg->encoding);
      }

      const cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      if (image.empty() || image.cols <= 0 || image.rows <= 0) {
        throw std::runtime_error("converted image is empty");
      }

      // Primary: edge-pair pipe fitting from pipeline_vision
      const cv::Mat edge_mask = banyu_detection::pipeMask(image, probe_rect_);
      const auto ahead_fit = banyu_detection::fitPipe(edge_mask,
          banyu_detection::aheadBand(image.rows));
      const auto ahead_centroid = banyu_detection::pipeCentroid(edge_mask);

      bool pipeline_detected = false;
      PipelineLine pipeline;

      if (ahead_fit.has_value()) {
        observation.status = banyu_msgs::msg::PipelineObservation::TRACKED;
        observation.heading_error_rad = std::clamp(
          ahead_fit->angle, static_cast<float>(-M_PI_2), static_cast<float>(M_PI_2));
        observation.offset_x = std::clamp(
          static_cast<float>(ahead_fit->lateral / (image.cols * 0.5f)), -1.0f, 1.0f);
        if (ahead_centroid.has_value()) {
          observation.offset_y = std::clamp(
            static_cast<float>((ahead_centroid->y - image.rows * 0.5f) / (image.rows * 0.5f)),
              -1.0f, 1.0f);
        } else {
          observation.offset_y = 0.0f;
        }
        const int inliers = cv::countNonZero(edge_mask);
        observation.confidence = std::clamp(static_cast<float>(inliers / 300.0f), 0.35f, 1.0f);
        pipeline_detected = true;

        pipeline.line = cv::Vec4f(std::sin(ahead_fit->angle), -std::cos(ahead_fit->angle),
                                  image.cols * 0.5f + ahead_fit->lateral, image.rows * 0.5f);
        pipeline.heading = ahead_fit->angle;
        pipeline.offset_x = observation.offset_x;
        pipeline.offset_y = observation.offset_y;
        pipeline.confidence = observation.confidence;
      }

      cv::Mat hsv;
      cv::Mat gray;
      cv::Mat blurred;
      cv::GaussianBlur(image, blurred, cv::Size(blur_kernel_size_, blur_kernel_size_), 0.0);
      cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);
      cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

      if (use_gui_) {
        getSliderValue();
      }

      cv::Mat pipeline_mask;
      if (!pipeline_detected) {
        pipeline_mask = makePipelineMask(gray, hsv);
        if (findPipeline(pipeline_mask, image.size(), pipeline)) {
          observation.status = banyu_msgs::msg::PipelineObservation::TRACKED;
          observation.offset_x = static_cast<float>(pipeline.offset_x);
          observation.offset_y = static_cast<float>(pipeline.offset_y);
          observation.heading_error_rad = static_cast<float>(pipeline.heading);
          observation.confidence = static_cast<float>(pipeline.confidence);
          pipeline_detected = true;
        }
      }

      // Detect light nodes on the pipeline (top-hat multi-color detection)
      const cv::Mat & lights_mask =
        (!edge_mask.empty() && cv::countNonZero(edge_mask) > 0) ? edge_mask : pipeline_mask;
      const auto lights = banyu_detection::detectLights(image, lights_mask);
      bool light_found = false;

      if (!lights.empty()) {
        const banyu_detection::Light * chosen = nullptr;
        for (const auto & l : lights) {
          if (l.colour == banyu_detection::Light::kRed) {
            chosen = &l;
            break;
          }
        }
        if (!chosen) {
          for (const auto & l : lights) {
            if (l.colour == banyu_detection::Light::kBlue) {
              chosen = &l;
              break;
            }
          }
        }
        if (!chosen) {
          chosen = &lights.front();
        }

        ring.visible = true;
        ring.color = (chosen->colour == banyu_detection::Light::kRed) ?
          banyu_msgs::msg::PipelineRingObservation::COLOR_RED :
          ((chosen->colour == banyu_detection::Light::kBlue) ?
          banyu_msgs::msg::PipelineRingObservation::COLOR_BLUE :
          banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN);
        ring.confidence = 0.9f;
        ring.offset_x = std::clamp(static_cast<float>(normalizedX(chosen->x, image.cols)), -1.0f,
            1.0f);
        ring.offset_y = std::clamp(static_cast<float>(normalizedY(chosen->y, image.rows)), -1.0f,
            1.0f);
        light_found = true;
      }

      cv::Mat red_mask;
      cv::Mat green_mask;
      if (!light_found) {
        RingCandidate best_red;
        bool found_red = false;
        RingCandidate best_green;
        bool found_green = false;

        findRings(
          hsv, pipeline, pipeline_detected,
          best_red, found_red, best_green,
          found_green, red_mask, green_mask);

        if (found_red) {
          ring.visible = true;
          ring.color = static_cast<uint8_t>(best_red.color);
          ring.confidence = static_cast<float>(best_red.confidence);
          ring.offset_x = static_cast<float>(normalizedX(best_red.center.x, image.cols));
          ring.offset_y = static_cast<float>(normalizedY(best_red.center.y, image.rows));
        } else if (found_green) {
          ring.visible = true;
          ring.color = static_cast<uint8_t>(best_green.color);
          ring.confidence = static_cast<float>(best_green.confidence);
          ring.offset_x = static_cast<float>(normalizedX(best_green.center.x, image.cols));
          ring.offset_y = static_cast<float>(normalizedY(best_green.center.y, image.rows));
        }
      }

      if (use_gui_) {
        cv::Mat annotated = image.clone();
        if (observation.status == banyu_msgs::msg::PipelineObservation::TRACKED) {
          const double vx = pipeline.line[0];
          const double vy = pipeline.line[1];
          const double x0 = pipeline.line[2];
          const double y0 = pipeline.line[3];
          const cv::Point pt1(static_cast<int>(x0 - vx * 500.0), static_cast<int>(y0 - vy * 500.0));
          const cv::Point pt2(static_cast<int>(x0 + vx * 500.0), static_cast<int>(y0 + vy * 500.0));
          const double lookahead_y = (pipeline.offset_y * 0.5 + 0.5) * image.rows;
          const double lookahead_x = (pipeline.offset_x * 0.5 + 0.5) * image.cols;
          cv::circle(
            annotated,
            cv::Point(static_cast<int>(lookahead_x), static_cast<int>(lookahead_y)), 6, cv::Scalar(
              0, 0, 255), -1);

          char buf[128];
          snprintf(
            buf, sizeof(buf), "PIPE: yaw=%.1f deg off=%.2f conf=%.2f",
            pipeline.heading * 180.0 / M_PI, pipeline.offset_x,
            pipeline.confidence);
          cv::putText(
            annotated, buf, cv::Point(15, 30), cv::FONT_HERSHEY_SIMPLEX, 0.55,
            cv::Scalar(0, 255, 0), 2);
        } else {
          cv::putText(
            annotated, "PIPE: NO_PIPE", cv::Point(
              15,
              30), cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(
              0, 0, 255), 2);
        }

        if (ring.visible) {
          const cv::Point ring_pt(
            static_cast<int>((ring.offset_x * 0.5 + 0.5) * image.cols),
            static_cast<int>((ring.offset_y * 0.5 + 0.5) * image.rows));
          const cv::Scalar ring_color = (ring.color ==
            banyu_msgs::msg::PipelineRingObservation::COLOR_RED) ?
            cv::Scalar(0, 0, 255) :
            (ring.color == banyu_msgs::msg::PipelineRingObservation::COLOR_BLUE ?
            cv::Scalar(255, 0, 0) :
            cv::Scalar(0, 255, 0));
          cv::circle(annotated, ring_pt, 20, ring_color, 2);
          cv::putText(
            annotated,
            ring.color == banyu_msgs::msg::PipelineRingObservation::COLOR_RED ? "RED RING" : "RING",
            cv::Point(ring_pt.x - 30, ring_pt.y - 25),
            cv::FONT_HERSHEY_SIMPLEX, 0.55, ring_color, 2);
        }

        cv::imshow("Original Frame", image);
        cv::imshow("Pipeline Mask", pipeline_mask.empty() ? edge_mask : pipeline_mask);
        if (!red_mask.empty()) {cv::imshow("Red Ring Mask", red_mask);}
        if (!green_mask.empty()) {cv::imshow("Green Ring Mask", green_mask);}
        cv::imshow("Annotated", annotated);
        cv::waitKey(1);
      }
    } catch (const cv_bridge::Exception & error) {
      observation.status = banyu_msgs::msg::PipelineObservation::INVALID;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 5000, "Cannot convert camera image: %s", error.what());
    } catch (const cv::Exception & error) {
      observation.status = banyu_msgs::msg::PipelineObservation::INVALID;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 5000, "OpenCV rejected camera image: %s", error.what());
    } catch (const std::exception & error) {
      observation.status = banyu_msgs::msg::PipelineObservation::INVALID;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), 5000, "Camera detector skipped image: %s", error.what());
    }

    observation_pub_->publish(observation);
    ring_observation_pub_->publish(ring);
  }

  static bool isSupportedEncoding(const std::string & encoding)
  {
    return encoding == sensor_msgs::image_encodings::BGR8 ||
           encoding == sensor_msgs::image_encodings::RGB8 ||
           encoding == sensor_msgs::image_encodings::MONO8 ||
           encoding == sensor_msgs::image_encodings::BGRA8 ||
           encoding == sensor_msgs::image_encodings::RGBA8 ||
           encoding == sensor_msgs::image_encodings::MONO16;
  }

  banyu_msgs::msg::PipelineObservation makeLostObservation(
    const sensor_msgs::msg::Image & image,
    uint64_t sequence) const
  {
    banyu_msgs::msg::PipelineObservation observation;
    observation.header = image.header;
    observation.frame_sequence = sequence;
    observation.status = banyu_msgs::msg::PipelineObservation::NO_PIPE;
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    observation.offset_x = unavailable;
    observation.offset_y = unavailable;
    observation.heading_error_rad = unavailable;
    observation.confidence = 0.0F;
    return observation;
  }

  banyu_msgs::msg::PipelineRingObservation makeLostRingObservation(
    const sensor_msgs::msg::Image & image, uint64_t sequence) const
  {
    banyu_msgs::msg::PipelineRingObservation ring;
    ring.header = image.header;
    ring.frame_sequence = sequence;
    ring.visible = false;
    ring.color = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
    ring.confidence = 0.0F;
    const float unavailable = std::numeric_limits<float>::quiet_NaN();
    ring.offset_x = unavailable;
    ring.offset_y = unavailable;
    return ring;
  }

  cv::Mat makePipelineMask(const cv::Mat & gray, const cv::Mat & hsv) const
  {
    cv::Mat hsv_mask;
    cv::inRange(
      hsv, cv::Scalar(hsv_h_min_, hsv_s_min_, hsv_v_min_),
      cv::Scalar(hsv_h_max_, hsv_s_max_, hsv_v_max_), hsv_mask);
    cv::Mat mask;
    if (gray_threshold_ > 0) {
      cv::Mat gray_mask;
      cv::inRange(gray, cv::Scalar(gray_threshold_), cv::Scalar(255), gray_mask);
      cv::bitwise_and(gray_mask, hsv_mask, mask);
    } else {
      mask = hsv_mask.clone();
    }

    if (use_top_hat_) {
      cv::Mat top_hat;
      const cv::Mat top_hat_kernel =
        cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(top_hat_kernel_size_, top_hat_kernel_size_));
      cv::morphologyEx(gray, top_hat, cv::MORPH_TOPHAT, top_hat_kernel);
      cv::Mat local_bright;
      cv::inRange(top_hat, cv::Scalar(top_hat_threshold_), cv::Scalar(255), local_bright);
      cv::bitwise_and(local_bright, hsv_mask, local_bright);
      cv::bitwise_or(mask, local_bright, mask);
    }

    const cv::Mat morphology_kernel =
      cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(morphology_kernel_size_, morphology_kernel_size_));
    if (morphology_open_iterations_ > 0) {
      cv::morphologyEx(
        mask, mask, cv::MORPH_OPEN, morphology_kernel, cv::Point(
          -1,
          -1),
        morphology_open_iterations_);
    }
    if (morphology_close_iterations_ > 0) {
      cv::morphologyEx(
        mask, mask, cv::MORPH_CLOSE, morphology_kernel, cv::Point(
          -1,
          -1),
        morphology_close_iterations_);
    }

    const int top = static_cast<int>(std::floor(roi_y_min_ratio_ * gray.rows));
    const int bottom = static_cast<int>(std::ceil(roi_y_max_ratio_ * gray.rows));
    if (top > 0) {
      mask(cv::Rect(0, 0, mask.cols, std::min(top, mask.rows))).setTo(0);
    }
    if (bottom < mask.rows) {
      mask(cv::Rect(0, std::max(0, bottom), mask.cols, mask.rows - std::max(0, bottom))).setTo(0);
    }
    return mask;
  }

  static bool fitPipelineLine(
    const std::vector<cv::Point2f> & points, cv::Vec4f & line, double & length,
    double & residual)
  {
    cv::fitLine(points, line, cv::DIST_L2, 0.0, 0.01, 0.01);
    const double vx = line[0];
    const double vy = line[1];
    const double norm = std::hypot(vx, vy);
    if (!std::isfinite(norm) || norm < 1.0e-6) {
      return false;
    }
    double min_projection = std::numeric_limits<double>::max();
    double max_projection = std::numeric_limits<double>::lowest();
    double squared_residual = 0.0;
    for (const cv::Point2f & point : points) {
      const double dx = point.x - line[2];
      const double dy = point.y - line[3];
      min_projection = std::min(min_projection, dx * vx + dy * vy);
      max_projection = std::max(max_projection, dx * vx + dy * vy);
      const double perpendicular = dx * vy - dy * vx;
      squared_residual += perpendicular * perpendicular;
    }
    length = max_projection - min_projection;
    residual = std::sqrt(squared_residual / points.size());
    return true;
  }

  bool findPipeline(const cv::Mat & mask, const cv::Size & image_size, PipelineLine & best) const
  {
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int component_count = cv::connectedComponentsWithStats(
      mask, labels, stats, centroids, 8,
      CV_32S);
    bool found = false;

    for (int label = 1; label < component_count; ++label) {
      const int area = stats.at<int>(label, cv::CC_STAT_AREA);
      if (area < min_component_area_px_) {
        continue;
      }
      const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
      const int y = stats.at<int>(label, cv::CC_STAT_TOP);
      const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
      const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      if (width < 2 || height < 2) {
        continue;
      }

      std::vector<cv::Point2f> points;
      points.reserve(static_cast<size_t>(area));
      for (int row = y; row < y + height; ++row) {
        for (int col = x; col < x + width; ++col) {
          if (labels.at<int>(row, col) == label) {
            points.emplace_back(static_cast<float>(col), static_cast<float>(row));
          }
        }
      }
      if (points.size() < 2) {
        continue;
      }

      const double short_side = static_cast<double>(std::min(width, height));
      const double long_side = static_cast<double>(std::max(width, height));
      double aspect = long_side / short_side;
      if (points.size() >= 5) {
        const cv::RotatedRect rot_rect = cv::minAreaRect(points);
        const double rot_short =
          std::max(1.0f, std::min(rot_rect.size.width, rot_rect.size.height));
        const double rot_long = std::max(rot_rect.size.width, rot_rect.size.height);
        aspect = std::max(aspect, static_cast<double>(rot_long / rot_short));
      }
      if (aspect < min_aspect_ratio_) {
        continue;
      }

      cv::Vec4f line;
      double length = 0.0;
      double residual = 0.0;
      if (!fitPipelineLine(points, line, length, residual)) {
        continue;
      }
      if (residual > max_line_residual_px_ && local_fit_band_ratio_ > 0.0) {
        // Bends and node housings make one line fit the whole blob badly; fit the straight
        // segment inside a band around the lookahead row that guidance actually uses.
        const double center_row = lookahead_y_ratio_ * image_size.height;
        const double half_band = local_fit_band_ratio_ * image_size.height;
        std::vector<cv::Point2f> local_points;
        for (const cv::Point2f & point : points) {
          if (std::abs(point.y - center_row) <= half_band) {
            local_points.push_back(point);
          }
        }
        if (local_points.size() >= 5) {
          fitPipelineLine(local_points, line, length, residual);
        }
      }
      if (length < min_line_length_px_ || residual > max_line_residual_px_) {
        continue;
      }
      const double vx = line[0];
      const double vy = line[1];

      double direction_x = vx;
      double direction_y = vy;
      if (direction_y > 0.0) {
        direction_x = -direction_x;
        direction_y = -direction_y;
      }

      double ref_x = line[2];
      double ref_y = line[3];
      bool has_lookahead_intersection = false;
      const double lookahead_y = lookahead_y_ratio_ * image_size.height;
      if (std::abs(direction_y) >= 1.0e-2) {
        const double line_y_min = static_cast<double>(y);
        const double line_y_max = static_cast<double>(y + height);
        const double distance_outside = lookahead_y < line_y_min ? line_y_min - lookahead_y :
          lookahead_y > line_y_max ? lookahead_y - line_y_max :
          0.0;
        if (distance_outside <= lookahead_extrapolation_ratio_ * image_size.height) {
          const double intersection_x = line[2] + (lookahead_y - line[3]) * direction_x /
            direction_y;
          if (std::isfinite(intersection_x) && intersection_x >= 0.0 &&
            intersection_x < image_size.width)
          {
            ref_x = intersection_x;
            ref_y = lookahead_y;
            has_lookahead_intersection = true;
          }
        }
      }

      if (!has_lookahead_intersection) {
        if (std::abs(direction_x) >= 1.0e-3) {
          const double center_col = image_size.width * 0.5;
          const double crossing_y = line[3] + (center_col - line[2]) * direction_y / direction_x;
          if (std::isfinite(crossing_y) && crossing_y >= 0.0 && crossing_y < image_size.height) {
            ref_x = center_col;
            ref_y = crossing_y;
          }
        }
      }

      // Use image-up as vehicle-forward; positive image-right is clockwise.
      const double heading = std::clamp(
        std::atan2(
          direction_x,
          -direction_y), -M_PI / 2.0, M_PI / 2.0);
      if (max_heading_error_rad_ < M_PI_2 && std::abs(heading) > max_heading_error_rad_) {
        continue;
      }
      const double confidence =
        pipelineConfidence(length, residual, aspect, static_cast<double>(area));
      if (confidence < min_pipeline_confidence_) {
        continue;
      }

      PipelineLine candidate;
      candidate.line = line;
      candidate.point = cv::Point2f(line[2], line[3]);
      candidate.length = length;
      candidate.residual = residual;
      candidate.aspect = aspect;
      candidate.offset_x = normalizedX(ref_x, image_size.width);
      candidate.offset_y = normalizedY(ref_y, image_size.height);
      candidate.heading = heading;
      candidate.confidence = confidence;
      if (!found || candidate.confidence > best.confidence) {
        best = candidate;
        found = true;
      }
    }
    return found;
  }

  double pipelineConfidence(double length, double residual, double aspect, double area) const
  {
    const double length_score = std::clamp(length / confidence_length_scale_px_, 0.0, 1.0);
    const double residual_score = std::clamp(1.0 - residual / max_line_residual_px_, 0.0, 1.0);
    const double aspect_score =
      std::clamp((aspect - min_aspect_ratio_) / std::max(1.0, min_aspect_ratio_), 0.0, 1.0);
    const double area_score = std::clamp(area / confidence_area_scale_px_, 0.0, 1.0);
    return 0.35 * length_score + 0.30 * residual_score + 0.20 * aspect_score + 0.15 * area_score;
  }

  void findRings(
    const cv::Mat & hsv, const PipelineLine & pipeline, bool pipeline_tracked,
    RingCandidate & best_red, bool & found_red,
    RingCandidate & best_green, bool & found_green, cv::Mat & out_red_mask,
    cv::Mat & out_green_mask) const
  {
    cv::Mat red_low_mask;
    cv::inRange(
      hsv, cv::Scalar(red_ring_h_low_min_, red_ring_s_min_, red_ring_v_min_),
      cv::Scalar(red_ring_h_low_max_, red_ring_s_max_, red_ring_v_max_), red_low_mask);
    cv::Mat red_high_mask;
    cv::inRange(
      hsv, cv::Scalar(red_ring_h_high_min_, red_ring_s_min_, red_ring_v_min_),
      cv::Scalar(red_ring_h_high_max_, red_ring_s_max_, red_ring_v_max_), red_high_mask);
    cv::bitwise_or(red_low_mask, red_high_mask, out_red_mask);

    cv::inRange(
      hsv, cv::Scalar(green_ring_h_min_, green_ring_s_min_, green_ring_v_min_),
      cv::Scalar(green_ring_h_max_, green_ring_s_max_, green_ring_v_max_), out_green_mask);

    cleanRingMask(out_red_mask);
    cleanRingMask(out_green_mask);

    found_red = false;
    found_green = false;
    if (pipeline_tracked) {
      inspectRingMask(
        out_red_mask, banyu_msgs::msg::PipelineRingObservation::COLOR_RED, hsv, pipeline, best_red,
        found_red,
        red_ring_min_area_);
      inspectRingMask(
        out_green_mask, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN, hsv, pipeline,
        best_green, found_green,
        green_ring_min_area_);
    }
  }

  void cleanRingMask(cv::Mat & mask) const
  {
    const cv::Mat kernel =
      cv::getStructuringElement(
      cv::MORPH_ELLIPSE,
      cv::Size(ring_morphology_kernel_size_, ring_morphology_kernel_size_));
    if (ring_open_iterations_ > 0) {
      cv::morphologyEx(
        mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1),
        ring_open_iterations_);
    }
    if (ring_close_iterations_ > 0) {
      cv::morphologyEx(
        mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(
          -1,
          -1), ring_close_iterations_);
    }
  }

  void inspectRingMask(
    const cv::Mat & mask, int color, const cv::Mat & hsv, const PipelineLine & pipeline,
    RingCandidate & best, bool & found,
    int min_area) const
  {
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int component_count = cv::connectedComponentsWithStats(
      mask, labels, stats, centroids, 8,
      CV_32S);
    for (int label = 1; label < component_count; ++label) {
      const int area = stats.at<int>(label, cv::CC_STAT_AREA);
      if (area < min_area) {
        continue;
      }
      const cv::Point2f center(static_cast<float>(centroids.at<double>(label, 0)),
        static_cast<float>(centroids.at<double>(label, 1)));
      const double distance = distanceToLine(center, pipeline.line);
      if (distance > ring_association_distance_px_) {
        continue;
      }

      const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
      const int y = stats.at<int>(label, cv::CC_STAT_TOP);
      const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
      const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
      const double ring_aspect = static_cast<double>(std::max(width, height)) / std::max(
        1, std::min(
          width, height));
      if (ring_aspect > ring_max_aspect_ratio_) {
        continue;
      }
      double mean_saturation = 0.0;
      double mean_value = 0.0;
      int sample_count = 0;
      for (int row = y; row < y + height; ++row) {
        for (int col = x; col < x + width; ++col) {
          if (labels.at<int>(row, col) == label) {
            const cv::Vec3b pixel = hsv.at<cv::Vec3b>(row, col);
            mean_saturation += pixel[1];
            mean_value += pixel[2];
            ++sample_count;
          }
        }
      }
      if (sample_count == 0) {
        continue;
      }
      mean_saturation /= sample_count * 255.0;
      mean_value /= sample_count * 255.0;
      const double area_score = std::clamp(
        static_cast<double>(area) / ring_area_confidence_scale_px_, 0.0, 1.0);
      const double association_score = std::clamp(
        1.0 - distance / ring_association_distance_px_,
        0.0, 1.0);
      const double confidence = 0.45 * area_score + 0.30 * association_score + 0.15 *
        mean_saturation + 0.10 * mean_value;
      if (confidence < ring_min_confidence_) {
        continue;
      }

      RingCandidate candidate;
      candidate.color = color;
      candidate.center = center;
      candidate.area = area;
      candidate.confidence = confidence;
      if (!found || candidate.confidence > best.confidence) {
        best = candidate;
        found = true;
      }
    }
  }

  static double distanceToLine(const cv::Point2f & point, const cv::Vec4f & line)
  {
    const double dx = point.x - line[2];
    const double dy = point.y - line[3];
    return std::abs(dx * line[1] - dy * line[0]);
  }

  double normalizedX(double x, int width) const
  {
    double center = width * 0.5;
    double half_width = center;
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (camera_width_ == static_cast<uint32_t>(width) && camera_cx_ > 0.0 && camera_cx_ < width) {
        center = camera_cx_;
        half_width = std::max(center, width - center);
      }
    }
    return std::clamp((x - center) / std::max(1.0, half_width), -1.0, 1.0);
  }

  double normalizedY(double y, int height) const
  {
    double center = height * 0.5;
    double half_height = center;
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (camera_height_ == static_cast<uint32_t>(height) && camera_cy_ > 0.0 &&
        camera_cy_ < height)
      {
        center = camera_cy_;
        half_height = std::max(center, height - center);
      }
    }
    return std::clamp((y - center) / std::max(1.0, half_height), -1.0, 1.0);
  }

  std::string image_topic_;
  std::string camera_info_topic_;
  bool use_camera_info_ {true};
  std::string observation_topic_;
  std::string ring_observation_topic_;

  bool use_gui_ {false};
  int gray_threshold_ {170};
  int hsv_value_threshold_ {150};
  int hsv_saturation_max_ {125};
  int hsv_h_min_ {0};
  int hsv_s_min_ {0};
  int hsv_v_min_ {150};
  int hsv_h_max_ {179};
  int hsv_s_max_ {125};
  int hsv_v_max_ {255};
  bool use_top_hat_ {true};
  int top_hat_kernel_size_ {31};
  int top_hat_threshold_ {12};
  int blur_kernel_size_ {3};
  int morphology_kernel_size_ {5};
  int morphology_open_iterations_ {1};
  int morphology_close_iterations_ {2};
  double roi_y_min_ratio_ {0.05};
  double roi_y_max_ratio_ {0.98};

  int min_component_area_px_ {20};
  double min_line_length_px_ {18.0};
  double min_aspect_ratio_ {1.5};
  double max_line_residual_px_ {10.0};
  double local_fit_band_ratio_ {0.2};
  double max_heading_error_rad_ {M_PI_2};
  double lookahead_y_ratio_ {0.65};
  double lookahead_extrapolation_ratio_ {0.35};
  double min_pipeline_confidence_ {0.20};
  double confidence_length_scale_px_ {100.0};
  double confidence_area_scale_px_ {500.0};

  int red_ring_h_low_min_ {0};
  int red_ring_h_low_max_ {15};
  int red_ring_h_high_min_ {165};
  int red_ring_h_high_max_ {179};
  int red_ring_s_min_ {40};
  int red_ring_s_max_ {255};
  int red_ring_v_min_ {30};
  int red_ring_v_max_ {255};
  int red_ring_min_area_ {8};

  int green_ring_h_min_ {35};
  int green_ring_h_max_ {95};
  int green_ring_s_min_ {40};
  int green_ring_s_max_ {255};
  int green_ring_v_min_ {30};
  int green_ring_v_max_ {255};
  int green_ring_min_area_ {8};

  int ring_morphology_kernel_size_ {3};
  int ring_open_iterations_ {1};
  int ring_close_iterations_ {1};
  int ring_min_component_area_px_ {8};
  double ring_area_confidence_scale_px_ {80.0};
  double ring_association_distance_px_ {120.0};
  double ring_max_aspect_ratio_ {4.5};
  double ring_min_confidence_ {0.15};

  cv::Rect probe_rect_{};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<banyu_msgs::msg::PipelineObservation>::SharedPtr observation_pub_;
  rclcpp::Publisher<banyu_msgs::msg::PipelineRingObservation>::SharedPtr ring_observation_pub_;

  uint64_t frame_sequence_ {0};
  mutable std::mutex camera_mutex_;
  uint32_t camera_width_ {0};
  uint32_t camera_height_ {0};
  double camera_cx_ {0.0};
  double camera_cy_ {0.0};
};

}  // namespace task2_pipeline_detector

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<task2_pipeline_detector::Task2PipelineDetector>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    fprintf(stderr, "task2_pipeline_detector failed: %s\n", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
