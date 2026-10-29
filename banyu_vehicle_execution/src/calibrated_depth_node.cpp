#include "banyu_vehicle_execution/calibrated_depth_node.hpp"

#include <cmath>
#include <chrono>

namespace banyu_vehicle_execution
{

namespace
{
constexpr double kMaxDatumSurfaceDepthM = 0.5;
}  // namespace

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
  calibrated_ = manually_calibrated;

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
  if (!std::isfinite(surface_rel_alt)) {
    return;
  }
  surface_datum_offset_ = surface_rel_alt;
  calibrated_ = true;
  RCLCPP_INFO(
    get_logger(),
    "Calibrated water surface datum offset to %.3f m",
    surface_datum_offset_);
}

bool CalibratedDepthNode::isNearSurface(double alt) const
{
  return std::isfinite(alt) && alt >= -0.3 && alt <= 0.5;
}

void CalibratedDepthNode::handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  const bool new_run = (msg->run_id != run_id_);
  run_id_ = msg->run_id;
  epoch_ = msg->epoch;
  current_phase_ = msg->phase;
  if (new_run && auto_calibrate_at_start_) {
    // A pre-start datum stays valid while the vehicle is still at the surface relative to it;
    // raw rel_alt alone drifts out of the surface band with barometer drift.
    const bool at_datum_surface = pre_start_calibration_ && calibrated_ &&
      std::abs(surface_datum_offset_ - last_rel_alt_) <= kMaxDatumSurfaceDepthM;
    if (!rel_alt_received_ || !(isNearSurface(last_rel_alt_) || at_datum_surface)) {
      calibrated_ = false;
    }
  }
  if (current_phase_ >= banyu_msgs::msg::RunLifecycle::ACTIVE) {
    pre_start_calibration_ = false;
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

  // Refresh the datum at the surface before every run, not only the first one.
  if (auto_calibrate_at_start_ && (!calibrated_ || !pre_start_calibration_) &&
    isNearSurface(last_rel_alt_))
  {
    if (current_phase_ < banyu_msgs::msg::RunLifecycle::ACTIVE) {
      surface_datum_offset_ = last_rel_alt_;
      calibrated_ = true;
      pre_start_calibration_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Initial surface datum auto-calibrated to %.3f m",
        surface_datum_offset_);
    }
  }

  // The datum can be dropped at the run start (the vehicle was not inside the surface band at that moment). Without it the
  // Surface action never starts and the report is never sent, so take it again while the vehicle is still at the surface and
  // has not dived since: a real surface reading cannot be a dive.
  if (last_rel_alt_ < surface_datum_offset_ - 1.0) {been_deep_ = true;}
  if (!calibrated_ && auto_calibrate_at_start_ && !been_deep_ && isNearSurface(last_rel_alt_)) {
    surface_datum_offset_ = last_rel_alt_;
    calibrated_ = true;
    RCLCPP_WARN(
      get_logger(), "Surface datum was missing: re-calibrated at the surface to %.3f m", surface_datum_offset_);
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
