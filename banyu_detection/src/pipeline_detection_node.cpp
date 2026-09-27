// Bottom camera -> banyu_msgs/PipeObservation (pipe fit ahead/behind, end of pipe, light nodes).

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <banyu_msgs/msg/pipe_observation.hpp>

#include "banyu_detection/pipeline_vision.hpp"

namespace {

banyu_msgs::msg::PipeFit toMsg(const std::optional<banyu_detection::PipeFit>& fit) {
  banyu_msgs::msg::PipeFit m;
  m.valid = fit.has_value();
  if (fit) {
    m.angle = fit->angle;
    m.lateral = fit->lateral;
  }
  return m;
}

}  // namespace

class PipelineDetection : public rclcpp::Node {
 public:
  PipelineDetection() : Node("pipeline_detection") {
    image_noise_ = declare_parameter<double>("image_noise", 0.0);  // DN; sim-only extra sensor noise
    const auto topic = declare_parameter<std::string>("image_topic", "/bottom_camera/image_raw");
    pub_ = create_publisher<banyu_msgs::msg::PipeObservation>("/banyu/pipeline/observation", rclcpp::SensorDataQoS());
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>("/banyu/pipeline/debug", 1);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        topic, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { onImage(msg); });
  }

 private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv::Mat bgr = cv_bridge::toCvCopy(msg, "bgr8")->image;
    if (image_noise_ > 0) {
      cv::Mat noise(bgr.size(), CV_16SC3);
      cv::randn(noise, 0, image_noise_);
      cv::Mat noisy;
      bgr.convertTo(noisy, CV_16SC3);
      noisy += noise;
      noisy.convertTo(bgr, CV_8UC3);  // saturates to 0..255
    }

    using namespace banyu_detection;
    const cv::Mat mask = pipeMask(bgr);
    const auto ahead = fitPipe(mask, aheadBand(bgr.rows));
    const auto centroid = pipeCentroid(mask);
    const auto lights = detectLights(bgr, mask);

    banyu_msgs::msg::PipeObservation obs;
    obs.header = msg->header;
    obs.width = bgr.cols;
    obs.height = bgr.rows;
    obs.reference_row = referenceRow(bgr.rows);
    obs.pipe_in_view = centroid.has_value();
    if (centroid) {
      obs.centroid_x = centroid->x;
      obs.centroid_y = centroid->y;
    }
    obs.ahead = toMsg(ahead);
    obs.behind = toMsg(fitPipe(mask, behindBand(bgr.rows)));
    obs.ends_ahead = pipeEndsHere(mask, 1);
    obs.ends_behind = pipeEndsHere(mask, -1);
    for (const auto& l : lights) {
      banyu_msgs::msg::LightNode node;
      node.colour = l.red ? banyu_msgs::msg::LightNode::RED : banyu_msgs::msg::LightNode::GREEN;
      node.x = l.x;
      node.y = l.y;
      obs.lights.push_back(node);
    }
    pub_->publish(obs);

    if (debug_pub_->get_subscription_count() > 0) {
      debug_pub_->publish(*cv_bridge::CvImage(msg->header, "bgr8", drawDebug(bgr, mask, ahead, lights)).toImageMsg());
    }
  }

  double image_noise_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<banyu_msgs::msg::PipeObservation>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PipelineDetection>());
  rclcpp::shutdown();
  return 0;
}
