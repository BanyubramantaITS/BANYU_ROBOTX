// OCS-side relay: banyu_msgs/PipelineSurveyReport and ResourceDeliveryRequest -> RoboCommand
// RxReport over MQTT.
//
// Runs on the topside OCS, never on the vehicle. Handbook 3.4: "Only the OCS connects to
// RoboCommand. Team vehicles must not connect directly to the RoboCommand network or MQTT
// broker." The surfaced UUV publishes its report on the ROS graph over the team link; this node
// alone speaks MQTT.
//
// ponytail: Task 2 reports only. The run also needs 2 Hz heartbeats per vehicle, RxCourse and the
// RunDeclaration (3.4.10/3.4.11) - the original Python rc_bridge did those. They share
// RxReport.seq with this report, so they belong in this node when ported.
#include <memory>
#include <string>

#include <mqtt/async_client.h>
#include <rclcpp/rclcpp.hpp>

#include <banyu_msgs/msg/pipeline_survey_report.hpp>
#include <banyu_msgs/msg/resource_delivery_request.hpp>

#include "banyu_robocommand/rx_report.hpp"

class ReportBridge : public rclcpp::Node {
 public:
  ReportBridge() : Node("robocommand_report_bridge") {
    team_ = declare_parameter<std::string>("team_id", "ITSN");
    vehicle_ = declare_parameter<std::string>("vehicle_id", "UUV1");
    const auto host = declare_parameter<std::string>("broker_host", "localhost");
    const auto port = declare_parameter<int>("broker_port", 1883);

    const std::string uri = "tcp://" + host + ":" + std::to_string(port);
    client_ = std::make_unique<mqtt::async_client>(uri, team_ + "-ocs-report-bridge");
    client_->set_connected_handler([this, uri](const std::string&) { RCLCPP_INFO(get_logger(), "MQTT connected to %s", uri.c_str()); });
    client_->set_connection_lost_handler([this](const std::string&) { RCLCPP_WARN(get_logger(), "MQTT connection lost, reconnecting"); });
    auto opts = mqtt::connect_options_builder().clean_session().automatic_reconnect(std::chrono::seconds(1), std::chrono::seconds(30)).finalize();
    client_->connect(opts);  // asynchronous; automatic_reconnect covers a broker that is not up yet

    sub_ = create_subscription<banyu_msgs::msg::PipelineSurveyReport>(
        "/banyu/task2/pipeline_survey", rclcpp::QoS(10).transient_local(),
        [this](banyu_msgs::msg::PipelineSurveyReport::ConstSharedPtr msg) { onReport(*msg); });
    // The UAV takes its re-tasking from this same topic; it reports its own echo of the request.
    request_sub_ = create_subscription<banyu_msgs::msg::ResourceDeliveryRequest>(
        "/banyu/task2/resource_request", rclcpp::QoS(10).transient_local(),
        [this](banyu_msgs::msg::ResourceDeliveryRequest::ConstSharedPtr msg) {
          publish(banyu_robocommand::encodeResourceDeliveryRequest(*msg, team_, vehicle_, ++seq_, now().nanoseconds()),
                  "ResourceDeliveryRequest (circle colour " + std::to_string(msg->delivery_circle_color) + ")");
        });
  }

  ~ReportBridge() override {
    try {
      if (client_->is_connected()) client_->disconnect()->wait();
    } catch (const mqtt::exception&) {
    }
  }

 private:
  void onReport(const banyu_msgs::msg::PipelineSurveyReport& msg) {
    publish(banyu_robocommand::encodePipelineSurveyReport(msg, team_, vehicle_, ++seq_, now().nanoseconds()),
            "PipelineSurveyReport (" + std::to_string(msg.segment_states.size()) + " segments)");
  }

  void publish(const std::string& payload, const std::string& what) {
    const std::string topic = banyu_robocommand::reportTopic(team_, vehicle_);
    try {
      client_->publish(topic, payload.data(), payload.size(), 1, false);
      RCLCPP_INFO(get_logger(), "%s seq %u -> %s", what.c_str(), seq_, topic.c_str());
    } catch (const mqtt::exception& e) {
      RCLCPP_ERROR(get_logger(), "MQTT publish failed: %s", e.what());
    }
  }

  std::string team_, vehicle_;
  uint32_t seq_{0};
  std::unique_ptr<mqtt::async_client> client_;
  rclcpp::Subscription<banyu_msgs::msg::PipelineSurveyReport>::SharedPtr sub_;
  rclcpp::Subscription<banyu_msgs::msg::ResourceDeliveryRequest>::SharedPtr request_sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReportBridge>());
  rclcpp::shutdown();
  return 0;
}
