#include "banyu_vehicle_execution/calibrated_depth_node.hpp"

#include <cmath>
#include <chrono>

namespace banyu_vehicle_execution
{

CalibratedDepthNode::CalibratedDepthNode(const rclcpp::NodeOptions & options)
: Node("calibrated_depth_node", options)
{
  source_id_ = declare_parameter<std::string>("source_id", "pressure_depth");
  datum_ = declare_parameter<std::string>("datum", "water_surface");
  calibration_id_ = declare_parameter<std::string>(
    "calibration_id", "surface_rel_alt_datum");
  surface_datum_offset_ = declare_parameter<double>("surface_datum_offset", 0.0);
  auto_calibrate_at_start_ = declare_parameter<bool>(
    "auto_calibrate_surface_at_start", true);
  max_rel_alt_age_s_ = declare_parameter<double>("max_rel_alt_age_s", 1.0);
  const bool manually_calibrated = declare_parameter<bool>("manually_calibrated", false);
  calibrated_ = manually_calibrated || !auto_calibrate_at_start_;

  const auto rel_alt_topic = declare_parameter<std::string>(
    "topics.rel_alt", "/mavros/global_position/rel_alt");
  const auto water_depth_topic = declare_parameter<std::string>(
    "topics.water_depth", "/banyu/vehicle/water_depth");
  const auto lifecycle_topic = declare_parameter<std::string>(
    "topics.run_lifecycle", "/banyu/mission/run_lifecycle");

  depth_pub_ = create_publisher<banyu_msgs::msg::WaterDepth>(
    water_depth_topic, rclcpp::QoS(10).reliable());

  rel_alt_sub_ = create_subscription<std_msgs::msg::Float64>(
    rel_alt_topic, rclcpp::SensorDataQoS(),
    std::bind(&CalibratedDepthNode::handleRelAlt, this, std::placeholders::_1));

  lifecycle_sub_ = create_subscription<banyu_msgs::msg::RunLifecycle>(
    lifecycle_topic, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&CalibratedDepthNode::handleLifecycle, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "CalibratedDepthNode initialized: source='%s', datum='%s', auto_calibrate=%d",
    source_id_.c_str(), datum_.c_str(), auto_calibrate_at_start_);
}

bool CalibratedDepthNode::isCalibrated() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return calibrated_;
}

double CalibratedDepthNode::surfaceDatumOffset() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return surface_datum_offset_;
}

uint32_t CalibratedDepthNode::runId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return run_id_;
}

uint64_t CalibratedDepthNode::epoch() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return epoch_;
}

void CalibratedDepthNode::calibrateDatum(double surface_rel_alt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  surface_datum_offset_ = surface_rel_alt;
  calibrated_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Calibrated water surface datum offset to %.3f m",
    surface_datum_offset_);
}

void CalibratedDepthNode::handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  const bool new_run = (msg->run_id != run_id_ || msg->epoch != epoch_);
  run_id_ = msg->run_id;
  epoch_ = msg->epoch;

  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (new_run && auto_calibrate_at_start_) {
    calibrated_ = false;
  }

  if (auto_calibrate_at_start_ && !calibrated_ && rel_alt_received_) {
    const double age = (now() - last_rel_alt_time_).seconds();
    if (age >= 0.0 && age <= max_rel_alt_age_s_ &&
      (msg->phase == Lifecycle::PREPARING || msg->phase == Lifecycle::READY ||
      msg->phase == Lifecycle::ACTIVE))
    {
      surface_datum_offset_ = last_rel_alt_;
      calibrated_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Auto-calibrated surface datum offset to %.3f m for run %u (epoch %lu)",
        surface_datum_offset_, run_id_, epoch_);
    }
  }
}

void CalibratedDepthNode::handleRelAlt(const std_msgs::msg::Float64::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto current_stamp = now();
  if (!msg || !std::isfinite(msg->data)) {
    banyu_msgs::msg::WaterDepth invalid_msg;
    invalid_msg.header.stamp = current_stamp;
    invalid_msg.header.frame_id = "base_link";
    invalid_msg.run_id = run_id_;
    invalid_msg.epoch = epoch_;
    invalid_msg.depth_m = 0.0;
    invalid_msg.valid = false;
    invalid_msg.calibrated = calibrated_;
    invalid_msg.datum = datum_;
    invalid_msg.source_id = source_id_;
    invalid_msg.calibration_id = calibration_id_;
    depth_pub_->publish(invalid_msg);
    return;
  }

  last_rel_alt_ = msg->data;
  last_rel_alt_time_ = current_stamp;
  rel_alt_received_ = true;

  if (auto_calibrate_at_start_ && !calibrated_) {
    surface_datum_offset_ = last_rel_alt_;
    calibrated_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Initial surface datum auto-calibrated to %.3f m",
      surface_datum_offset_);
  }

  const double depth_m = surface_datum_offset_ - last_rel_alt_;

  banyu_msgs::msg::WaterDepth depth_msg;
  depth_msg.header.stamp = current_stamp;
  depth_msg.header.frame_id = "base_link";
  depth_msg.run_id = run_id_;
  depth_msg.epoch = epoch_;
  depth_msg.depth_m = depth_m;
  depth_msg.valid = true;
  depth_msg.calibrated = calibrated_;
  depth_msg.datum = datum_;
  depth_msg.source_id = source_id_;
  depth_msg.calibration_id = calibration_id_;

  depth_pub_->publish(depth_msg);
}

}  // namespace banyu_vehicle_execution

#ifndef BANYU_VEHICLE_EXECUTION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_vehicle_execution::CalibratedDepthNode>());
  rclcpp::shutdown();
  return 0;
}
#endif
