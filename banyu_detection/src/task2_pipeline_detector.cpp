#include <banyu_msgs/msg/pipeline_observation.hpp>
#include <banyu_msgs/msg/pipeline_ring_observation.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace task2_pipeline_detector {

struct PipelineLine {
    cv::Vec4f line{};
    cv::Point2f point{};
    double length{0.0};
    double residual{0.0};
    double aspect{0.0};
    double offset_x{0.0};
    double offset_y{0.0};
    double heading{0.0};
    double confidence{0.0};
};

struct RingCandidate {
    int color{banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN};
    cv::Point2f center{};
    double area{0.0};
    double confidence{0.0};
};

class Task2PipelineDetector : public rclcpp::Node {
public:
    Task2PipelineDetector() : Node("task2_pipeline_detector") {
        declareParameters();

        observation_pub_ = create_publisher<banyu_msgs::msg::PipelineObservation>(
            observation_topic_, rclcpp::SensorDataQoS());
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

        RCLCPP_INFO(get_logger(), "task2_pipeline_detector listening on %s", image_topic_.c_str());
    }

private:
    void declareParameters() {
        image_topic_ = declare_parameter<std::string>("image_topic", "/front_camera/image_raw");
        camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/camera/camera_info");
        use_camera_info_ = declare_parameter<bool>("use_camera_info", true);
        observation_topic_ = declare_parameter<std::string>(
            "observation_topic", "/banyu/pipeline/observation");
        ring_observation_topic_ = declare_parameter<std::string>(
            "ring_observation_topic", "/banyu/pipeline/ring_observation");

        gray_threshold_ = declare_parameter<int>("gray_threshold", 170);
        hsv_value_threshold_ = declare_parameter<int>("hsv_value_threshold", 150);
        hsv_saturation_max_ = declare_parameter<int>("hsv_saturation_max", 125);
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
        lookahead_y_ratio_ = declare_parameter<double>("lookahead_y_ratio", 0.65);
        lookahead_extrapolation_ratio_ = declare_parameter<double>(
            "lookahead_extrapolation_ratio", 0.35);
        min_pipeline_confidence_ = declare_parameter<double>("min_pipeline_confidence", 0.20);
        confidence_length_scale_px_ = declare_parameter<double>(
            "confidence_length_scale_px", 100.0);
        confidence_area_scale_px_ = declare_parameter<double>(
            "confidence_area_scale_px", 500.0);

        red_hue_low_min_ = declare_parameter<int>("red_hue_low_min", 0);
        red_hue_low_max_ = declare_parameter<int>("red_hue_low_max", 10);
        red_hue_high_min_ = declare_parameter<int>("red_hue_high_min", 170);
        red_hue_high_max_ = declare_parameter<int>("red_hue_high_max", 179);
        green_hue_min_ = declare_parameter<int>("green_hue_min", 35);
        green_hue_max_ = declare_parameter<int>("green_hue_max", 90);
        ring_saturation_min_ = declare_parameter<int>("ring_saturation_min", 80);
        ring_value_min_ = declare_parameter<int>("ring_value_min", 45);
        ring_morphology_kernel_size_ = declare_parameter<int>("ring_morphology_kernel_size", 3);
        ring_open_iterations_ = declare_parameter<int>("ring_open_iterations", 1);
        ring_close_iterations_ = declare_parameter<int>("ring_close_iterations", 1);
        ring_min_component_area_px_ = declare_parameter<int>("ring_min_component_area_px", 4);
        ring_area_confidence_scale_px_ = declare_parameter<double>(
            "ring_area_confidence_scale_px", 80.0);
        ring_association_distance_px_ = declare_parameter<double>(
            "ring_association_distance_px", 45.0);
        ring_min_confidence_ = declare_parameter<double>("ring_min_confidence", 0.20);

        normalizeParameters();
    }

    static int oddKernel(int value, int fallback) {
        value = value > 0 ? value : fallback;
        return value % 2 == 0 ? value + 1 : value;
    }

    void normalizeParameters() {
        gray_threshold_ = std::clamp(gray_threshold_, 0, 255);
        hsv_value_threshold_ = std::clamp(hsv_value_threshold_, 0, 255);
        hsv_saturation_max_ = std::clamp(hsv_saturation_max_, 0, 255);
        top_hat_threshold_ = std::clamp(top_hat_threshold_, 0, 255);
        red_hue_low_min_ = std::clamp(red_hue_low_min_, 0, 179);
        red_hue_low_max_ = std::clamp(red_hue_low_max_, 0, 179);
        red_hue_high_min_ = std::clamp(red_hue_high_min_, 0, 179);
        red_hue_high_max_ = std::clamp(red_hue_high_max_, 0, 179);
        green_hue_min_ = std::clamp(green_hue_min_, 0, 179);
        green_hue_max_ = std::clamp(green_hue_max_, 0, 179);
        ring_saturation_min_ = std::clamp(ring_saturation_min_, 0, 255);
        ring_value_min_ = std::clamp(ring_value_min_, 0, 255);

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
        max_line_residual_px_ = std::max(0.1, max_line_residual_px_);
        confidence_length_scale_px_ = std::max(1.0, confidence_length_scale_px_);
        confidence_area_scale_px_ = std::max(1.0, confidence_area_scale_px_);
        ring_area_confidence_scale_px_ = std::max(1.0, ring_area_confidence_scale_px_);
        ring_association_distance_px_ = std::max(0.1, ring_association_distance_px_);
        lookahead_y_ratio_ = std::clamp(lookahead_y_ratio_, 0.0, 1.0);
        lookahead_extrapolation_ratio_ = std::max(0.0, lookahead_extrapolation_ratio_);
        roi_y_min_ratio_ = std::clamp(roi_y_min_ratio_, 0.0, 1.0);
        roi_y_max_ratio_ = std::clamp(roi_y_max_ratio_, roi_y_min_ratio_, 1.0);
        min_pipeline_confidence_ = std::clamp(min_pipeline_confidence_, 0.0, 1.0);
        ring_min_confidence_ = std::clamp(ring_min_confidence_, 0.0, 1.0);
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        if (!msg || msg->width == 0 || msg->height == 0 ||
            !std::isfinite(msg->k[0]) || !std::isfinite(msg->k[2]) ||
            !std::isfinite(msg->k[4]) || !std::isfinite(msg->k[5]) ||
            msg->k[0] <= 0.0 || msg->k[4] <= 0.0) {
            return;
        }
        std::lock_guard<std::mutex> lock(camera_mutex_);
        camera_width_ = msg->width;
        camera_height_ = msg->height;
        camera_cx_ = msg->k[2];
        camera_cy_ = msg->k[5];
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
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

            cv::Mat hsv;
            cv::Mat gray;
            cv::Mat blurred;
            cv::GaussianBlur(image, blurred,
                             cv::Size(blur_kernel_size_, blur_kernel_size_), 0.0);
            cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

            const cv::Mat pipeline_mask = makePipelineMask(gray, hsv);
            PipelineLine pipeline;
            if (findPipeline(pipeline_mask, image.size(), pipeline)) {
                observation.status = banyu_msgs::msg::PipelineObservation::TRACKED;
                observation.offset_x = static_cast<float>(pipeline.offset_x);
                observation.offset_y = static_cast<float>(pipeline.offset_y);
                observation.heading_error_rad = static_cast<float>(pipeline.heading);
                observation.confidence = static_cast<float>(pipeline.confidence);
            }

            if (observation.status == banyu_msgs::msg::PipelineObservation::TRACKED) {
                RingCandidate ring_candidate;
                if (findRing(hsv, pipeline, ring_candidate)) {
                    ring.visible = true;
                    ring.color = static_cast<uint8_t>(ring_candidate.color);
                    ring.confidence = static_cast<float>(ring_candidate.confidence);
                    ring.offset_x = static_cast<float>(normalizedX(ring_candidate.center.x, image.cols));
                    ring.offset_y = static_cast<float>(normalizedY(ring_candidate.center.y, image.rows));
                }
            }
        } catch (const cv_bridge::Exception& error) {
            observation.status = banyu_msgs::msg::PipelineObservation::INVALID;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Cannot convert camera image: %s", error.what());
        } catch (const cv::Exception& error) {
            observation.status = banyu_msgs::msg::PipelineObservation::INVALID;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "OpenCV rejected camera image: %s", error.what());
        } catch (const std::exception& error) {
            observation.status = banyu_msgs::msg::PipelineObservation::INVALID;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Camera detector skipped image: %s", error.what());
        }

        observation_pub_->publish(observation);
        ring_observation_pub_->publish(ring);
    }

    static bool isSupportedEncoding(const std::string& encoding) {
        return encoding == sensor_msgs::image_encodings::BGR8 ||
               encoding == sensor_msgs::image_encodings::RGB8 ||
               encoding == sensor_msgs::image_encodings::MONO8 ||
               encoding == sensor_msgs::image_encodings::BGRA8 ||
               encoding == sensor_msgs::image_encodings::RGBA8 ||
               encoding == sensor_msgs::image_encodings::MONO16;
    }

    banyu_msgs::msg::PipelineObservation makeLostObservation(
        const sensor_msgs::msg::Image& image, uint64_t sequence) const {
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
        const sensor_msgs::msg::Image& image, uint64_t sequence) const {
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

    cv::Mat makePipelineMask(const cv::Mat& gray, const cv::Mat& hsv) const {
        cv::Mat gray_mask;
        cv::Mat hsv_mask;
        cv::inRange(gray, cv::Scalar(gray_threshold_), cv::Scalar(255), gray_mask);
        cv::inRange(hsv, cv::Scalar(0, 0, hsv_value_threshold_),
                    cv::Scalar(179, hsv_saturation_max_, 255), hsv_mask);
        cv::Mat mask;
        cv::bitwise_and(gray_mask, hsv_mask, mask);

        if (use_top_hat_) {
            cv::Mat top_hat;
            const cv::Mat top_hat_kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(top_hat_kernel_size_, top_hat_kernel_size_));
            cv::morphologyEx(gray, top_hat, cv::MORPH_TOPHAT, top_hat_kernel);
            cv::Mat local_bright;
            cv::inRange(top_hat, cv::Scalar(top_hat_threshold_), cv::Scalar(255), local_bright);
            cv::bitwise_and(local_bright, hsv_mask, local_bright);
            cv::bitwise_or(mask, local_bright, mask);
        }

        const cv::Mat morphology_kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(morphology_kernel_size_, morphology_kernel_size_));
        if (morphology_open_iterations_ > 0) {
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morphology_kernel,
                             cv::Point(-1, -1), morphology_open_iterations_);
        }
        if (morphology_close_iterations_ > 0) {
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, morphology_kernel,
                             cv::Point(-1, -1), morphology_close_iterations_);
        }

        const int top = static_cast<int>(std::floor(roi_y_min_ratio_ * gray.rows));
        const int bottom = static_cast<int>(std::ceil(roi_y_max_ratio_ * gray.rows));
        if (top > 0) {
            mask(cv::Rect(0, 0, mask.cols, std::min(top, mask.rows))).setTo(0);
        }
        if (bottom < mask.rows) {
            mask(cv::Rect(0, std::max(0, bottom), mask.cols,
                          mask.rows - std::max(0, bottom))).setTo(0);
        }
        return mask;
    }

    bool findPipeline(const cv::Mat& mask, const cv::Size& image_size,
                      PipelineLine& best) const {
        cv::Mat labels;
        cv::Mat stats;
        cv::Mat centroids;
        const int component_count = cv::connectedComponentsWithStats(
            mask, labels, stats, centroids, 8, CV_32S);
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

            const double short_side = static_cast<double>(std::min(width, height));
            const double long_side = static_cast<double>(std::max(width, height));
            const double aspect = long_side / short_side;
            if (aspect < min_aspect_ratio_) {
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

            cv::Vec4f line;
            cv::fitLine(points, line, cv::DIST_L2, 0.0, 0.01, 0.01);
            const double vx = line[0];
            const double vy = line[1];
            const double norm = std::hypot(vx, vy);
            if (!std::isfinite(norm) || norm < 1.0e-6) {
                continue;
            }

            double min_projection = std::numeric_limits<double>::max();
            double max_projection = std::numeric_limits<double>::lowest();
            double squared_residual = 0.0;
            for (const cv::Point2f& point : points) {
                const double dx = point.x - line[2];
                const double dy = point.y - line[3];
                min_projection = std::min(min_projection, dx * vx + dy * vy);
                max_projection = std::max(max_projection, dx * vx + dy * vy);
                const double perpendicular = dx * vy - dy * vx;
                squared_residual += perpendicular * perpendicular;
            }
            const double length = max_projection - min_projection;
            const double residual = std::sqrt(squared_residual / points.size());
            if (length < min_line_length_px_ || residual > max_line_residual_px_) {
                continue;
            }

            double direction_x = vx;
            double direction_y = vy;
            if (direction_y > 0.0) {
                direction_x = -direction_x;
                direction_y = -direction_y;
            }
            if (std::abs(direction_y) < 1.0e-3) {
                continue;
            }

            const double lookahead_y = lookahead_y_ratio_ * image_size.height;
            const double line_y_min = static_cast<double>(y);
            const double line_y_max = static_cast<double>(y + height);
            const double distance_outside =
                lookahead_y < line_y_min ? line_y_min - lookahead_y :
                lookahead_y > line_y_max ? lookahead_y - line_y_max : 0.0;
            if (distance_outside > lookahead_extrapolation_ratio_ * image_size.height) {
                continue;
            }

            const double intersection_x = line[2] +
                (lookahead_y - line[3]) * direction_x / direction_y;
            if (!std::isfinite(intersection_x) || intersection_x < 0.0 ||
                intersection_x >= image_size.width) {
                continue;
            }

            // Use image-up as vehicle-forward; positive image-right is clockwise.
            const double heading = std::clamp(
                std::atan2(direction_x, -direction_y), -M_PI / 2.0, M_PI / 2.0);
            const double confidence = pipelineConfidence(
                length, residual, aspect, static_cast<double>(area));
            if (confidence < min_pipeline_confidence_) {
                continue;
            }

            PipelineLine candidate;
            candidate.line = line;
            candidate.point = cv::Point2f(line[2], line[3]);
            candidate.length = length;
            candidate.residual = residual;
            candidate.aspect = aspect;
            candidate.offset_x = normalizedX(intersection_x, image_size.width);
            candidate.offset_y = normalizedY(lookahead_y, image_size.height);
            candidate.heading = heading;
            candidate.confidence = confidence;
            if (!found || candidate.confidence > best.confidence) {
                best = candidate;
                found = true;
            }
        }
        return found;
    }

    double pipelineConfidence(double length, double residual, double aspect,
                              double area) const {
        const double length_score = std::clamp(length / confidence_length_scale_px_, 0.0, 1.0);
        const double residual_score = std::clamp(
            1.0 - residual / max_line_residual_px_, 0.0, 1.0);
        const double aspect_score = std::clamp(
            (aspect - min_aspect_ratio_) / std::max(1.0, min_aspect_ratio_), 0.0, 1.0);
        const double area_score = std::clamp(area / confidence_area_scale_px_, 0.0, 1.0);
        return 0.35 * length_score + 0.30 * residual_score +
               0.20 * aspect_score + 0.15 * area_score;
    }

    bool findRing(const cv::Mat& hsv, const PipelineLine& pipeline,
                  RingCandidate& best) const {
        cv::Mat red_mask;
        cv::Mat green_mask;
        cv::inRange(hsv, cv::Scalar(red_hue_low_min_, ring_saturation_min_, ring_value_min_),
                    cv::Scalar(red_hue_low_max_, 255, 255), red_mask);
        cv::Mat red_high_mask;
        cv::inRange(hsv, cv::Scalar(red_hue_high_min_, ring_saturation_min_, ring_value_min_),
                    cv::Scalar(red_hue_high_max_, 255, 255), red_high_mask);
        cv::bitwise_or(red_mask, red_high_mask, red_mask);
        cv::inRange(hsv, cv::Scalar(green_hue_min_, ring_saturation_min_, ring_value_min_),
                    cv::Scalar(green_hue_max_, 255, 255), green_mask);

        cleanRingMask(red_mask);
        cleanRingMask(green_mask);

        bool found = false;
        inspectRingMask(red_mask, banyu_msgs::msg::PipelineRingObservation::COLOR_RED,
                        hsv, pipeline, best, found);
        inspectRingMask(green_mask, banyu_msgs::msg::PipelineRingObservation::COLOR_GREEN,
                        hsv, pipeline, best, found);
        return found;
    }

    void cleanRingMask(cv::Mat& mask) const {
        const cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(ring_morphology_kernel_size_, ring_morphology_kernel_size_));
        if (ring_open_iterations_ > 0) {
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel,
                             cv::Point(-1, -1), ring_open_iterations_);
        }
        if (ring_close_iterations_ > 0) {
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel,
                             cv::Point(-1, -1), ring_close_iterations_);
        }
    }

    void inspectRingMask(const cv::Mat& mask, int color, const cv::Mat& hsv,
                         const PipelineLine& pipeline, RingCandidate& best,
                         bool& found) const {
        cv::Mat labels;
        cv::Mat stats;
        cv::Mat centroids;
        const int component_count = cv::connectedComponentsWithStats(
            mask, labels, stats, centroids, 8, CV_32S);
        for (int label = 1; label < component_count; ++label) {
            const int area = stats.at<int>(label, cv::CC_STAT_AREA);
            if (area < ring_min_component_area_px_) {
                continue;
            }
            const cv::Point2f center(
                static_cast<float>(centroids.at<double>(label, 0)),
                static_cast<float>(centroids.at<double>(label, 1)));
            const double distance = distanceToLine(center, pipeline.line);
            if (distance > ring_association_distance_px_) {
                continue;
            }

            const int x = stats.at<int>(label, cv::CC_STAT_LEFT);
            const int y = stats.at<int>(label, cv::CC_STAT_TOP);
            const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
            const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
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
                1.0 - distance / ring_association_distance_px_, 0.0, 1.0);
            const double confidence = 0.45 * area_score + 0.30 * association_score +
                                      0.15 * mean_saturation + 0.10 * mean_value;
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

    static double distanceToLine(const cv::Point2f& point, const cv::Vec4f& line) {
        const double dx = point.x - line[2];
        const double dy = point.y - line[3];
        return std::abs(dx * line[1] - dy * line[0]);
    }

    double normalizedX(double x, int width) const {
        double center = width * 0.5;
        double half_width = center;
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            if (camera_width_ == static_cast<uint32_t>(width) && camera_cx_ > 0.0 &&
                camera_cx_ < width) {
                center = camera_cx_;
                half_width = std::max(center, width - center);
            }
        }
        return std::clamp((x - center) / std::max(1.0, half_width), -1.0, 1.0);
    }

    double normalizedY(double y, int height) const {
        double center = height * 0.5;
        double half_height = center;
        {
            std::lock_guard<std::mutex> lock(camera_mutex_);
            if (camera_height_ == static_cast<uint32_t>(height) && camera_cy_ > 0.0 &&
                camera_cy_ < height) {
                center = camera_cy_;
                half_height = std::max(center, height - center);
            }
        }
        return std::clamp((y - center) / std::max(1.0, half_height), -1.0, 1.0);
    }

    std::string image_topic_;
    std::string camera_info_topic_;
    bool use_camera_info_{true};
    std::string observation_topic_;
    std::string ring_observation_topic_;

    int gray_threshold_{170};
    int hsv_value_threshold_{150};
    int hsv_saturation_max_{125};
    bool use_top_hat_{true};
    int top_hat_kernel_size_{31};
    int top_hat_threshold_{12};
    int blur_kernel_size_{3};
    int morphology_kernel_size_{5};
    int morphology_open_iterations_{1};
    int morphology_close_iterations_{2};
    double roi_y_min_ratio_{0.05};
    double roi_y_max_ratio_{0.98};

    int min_component_area_px_{20};
    double min_line_length_px_{18.0};
    double min_aspect_ratio_{1.5};
    double max_line_residual_px_{10.0};
    double lookahead_y_ratio_{0.65};
    double lookahead_extrapolation_ratio_{0.35};
    double min_pipeline_confidence_{0.20};
    double confidence_length_scale_px_{100.0};
    double confidence_area_scale_px_{500.0};

    int red_hue_low_min_{0};
    int red_hue_low_max_{10};
    int red_hue_high_min_{170};
    int red_hue_high_max_{179};
    int green_hue_min_{35};
    int green_hue_max_{90};
    int ring_saturation_min_{80};
    int ring_value_min_{45};
    int ring_morphology_kernel_size_{3};
    int ring_open_iterations_{1};
    int ring_close_iterations_{1};
    int ring_min_component_area_px_{4};
    double ring_area_confidence_scale_px_{80.0};
    double ring_association_distance_px_{45.0};
    double ring_min_confidence_{0.20};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<banyu_msgs::msg::PipelineObservation>::SharedPtr observation_pub_;
    rclcpp::Publisher<banyu_msgs::msg::PipelineRingObservation>::SharedPtr ring_observation_pub_;

    uint64_t frame_sequence_{0};
    mutable std::mutex camera_mutex_;
    uint32_t camera_width_{0};
    uint32_t camera_height_{0};
    double camera_cx_{0.0};
    double camera_cy_{0.0};
};

}  // namespace task2_pipeline_detector

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<task2_pipeline_detector::Task2PipelineDetector>();
        rclcpp::spin(node);
    } catch (const std::exception& error) {
        fprintf(stderr, "task2_pipeline_detector failed: %s\n", error.what());
    }
    rclcpp::shutdown();
    return 0;
}
