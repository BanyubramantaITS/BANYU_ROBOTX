#include "banyu_mission/action/arm_and_set_guided.hpp"

#include <utility>

namespace banyu_mission
{

ArmAndSetGuided::ArmAndSetGuided(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, std::shared_ptr<MissionState> state,
  std::string set_mode_service, std::string arming_service,
  std::string setpoint_topic)
: BT::StatefulActionNode(name, config),
  node_(std::move(node)),
  state_(std::move(state)),
  set_mode_service_(std::move(set_mode_service)),
  arming_service_(std::move(arming_service)),
  setpoint_topic_(std::move(setpoint_topic)),
  set_mode_client_(node_->create_client<mavros_msgs::srv::SetMode>(set_mode_service_)),
  arming_client_(node_->create_client<mavros_msgs::srv::CommandBool>(arming_service_)),
  setpoint_pub_(node_->create_publisher<mavros_msgs::msg::PositionTarget>(
      setpoint_topic_, rclcpp::QoS(10)))
{
}

BT::NodeStatus ArmAndSetGuided::onStart()
{
  std::lock_guard<std::mutex> lock(mutex_);
  generation_++;
  start_time_ = std::chrono::steady_clock::now();
  set_mode_done_ = false;
  set_mode_success_ = false;
  arming_done_ = false;
  arming_success_ = false;

  const bool ready = state_->vehicleReady() &&
    (!state_->gpsRequired() || state_->gpsAvailable());
  if (!ready) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Preflight: waiting for vehicle readiness%s before arming...",
      state_->gpsRequired() ? " and GPS fix" : "");
    step_ = Step::WaitingForReadiness;
    return BT::NodeStatus::RUNNING;
  }

  const bool services_ready =
    set_mode_client_->wait_for_service(std::chrono::milliseconds(0)) &&
    arming_client_->wait_for_service(std::chrono::milliseconds(0));

  if (services_ready) {
    sendSetModeRequest();
    step_ = Step::WaitingForSetMode;
  } else {
    RCLCPP_INFO(
      node_->get_logger(),
      "Waiting for MAVROS set_mode and arming services to become available...");
    step_ = Step::WaitingForServices;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ArmAndSetGuided::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
    start_time_).count() > timeout_s_)
  {
    RCLCPP_ERROR(
      node_->get_logger(), "ArmAndSetGuided timed out after %.1f seconds", timeout_s_);
    return BT::NodeStatus::FAILURE;
  }
  if (step_ != Step::WaitingForReadiness && !state_->vehicleReady()) {
    return BT::NodeStatus::FAILURE;
  }

  if (step_ == Step::WaitingForReadiness) {
    if (!state_->vehicleReady() ||
      (state_->gpsRequired() && !state_->gpsAvailable()))
    {
      return BT::NodeStatus::RUNNING;
    }
    const bool services_ready =
      set_mode_client_->wait_for_service(std::chrono::milliseconds(0)) &&
      arming_client_->wait_for_service(std::chrono::milliseconds(0));
    if (services_ready) {
      sendSetModeRequest();
      step_ = Step::WaitingForSetMode;
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "Waiting for MAVROS set_mode and arming services to become available...");
      step_ = Step::WaitingForServices;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (step_ == Step::WaitingForServices) {
    const bool services_ready =
      set_mode_client_->wait_for_service(std::chrono::milliseconds(0)) &&
      arming_client_->wait_for_service(std::chrono::milliseconds(0));
    if (services_ready) {
      sendSetModeRequest();
      step_ = Step::WaitingForSetMode;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (step_ == Step::WaitingForSetMode) {
    if (!set_mode_done_) {
      return BT::NodeStatus::RUNNING;
    }
    if (!set_mode_success_) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to set flight mode to GUIDED");
      return BT::NodeStatus::FAILURE;
    }
    if (state_->snapshot().mode != "GUIDED") {return BT::NodeStatus::RUNNING;}
    publishZeroVelocitySetpoint();
    RCLCPP_INFO(node_->get_logger(), "Flight mode set to GUIDED. Arming vehicle...");
    sendArmingRequest();
    step_ = Step::WaitingForArming;
    return BT::NodeStatus::RUNNING;
  }

  if (step_ == Step::WaitingForArming) {
    publishZeroVelocitySetpoint();
    if (!arming_done_) {
      return BT::NodeStatus::RUNNING;
    }
    if (!arming_success_) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to arm vehicle");
      return BT::NodeStatus::FAILURE;
    }
    const auto vehicle = state_->snapshot();
    if (vehicle.mode != "GUIDED" || !vehicle.armed) {
      return BT::NodeStatus::RUNNING;
    }
    RCLCPP_INFO(node_->get_logger(), "Vehicle successfully armed in GUIDED mode");
    state_->markAutonomyEngaged();
    step_ = Step::Done;
    return BT::NodeStatus::SUCCESS;
  }

  if (step_ == Step::Done) {
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ArmAndSetGuided::onHalted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  generation_++;
  set_mode_client_->prune_pending_requests();
  arming_client_->prune_pending_requests();
  step_ = Step::WaitingForReadiness;
}

void ArmAndSetGuided::sendSetModeRequest()
{
  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = "GUIDED";
  const auto generation = generation_;

  set_mode_client_->async_send_request(
    request, [this, generation](
      rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != generation_) {
        return;
      }
      try {
        auto response = future.get();
        set_mode_done_ = true;
        set_mode_success_ = response->mode_sent;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "SetMode service call exception: %s", e.what());
        set_mode_done_ = true;
        set_mode_success_ = false;
      }
    });
}

void ArmAndSetGuided::sendArmingRequest()
{
  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = true;
  const auto generation = generation_;

  arming_client_->async_send_request(
    request, [this, generation](
      rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (generation != generation_) {
        return;
      }
      try {
        auto response = future.get();
        arming_done_ = true;
        arming_success_ = response->success;
      } catch (const std::exception & e) {
        RCLCPP_ERROR(node_->get_logger(), "Arming service call exception: %s", e.what());
        arming_done_ = true;
        arming_success_ = false;
      }
    });
}

void ArmAndSetGuided::publishZeroVelocitySetpoint()
{
  if (!setpoint_pub_) {
    return;
  }
  mavros_msgs::msg::PositionTarget target;
  target.header.stamp = node_->now();
  target.header.frame_id = "map";
  target.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
  target.type_mask =
    mavros_msgs::msg::PositionTarget::IGNORE_PX |
    mavros_msgs::msg::PositionTarget::IGNORE_PY |
    mavros_msgs::msg::PositionTarget::IGNORE_PZ |
    mavros_msgs::msg::PositionTarget::IGNORE_AFX |
    mavros_msgs::msg::PositionTarget::IGNORE_AFY |
    mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
    mavros_msgs::msg::PositionTarget::IGNORE_YAW;
  target.yaw = 0.0F;
  target.yaw_rate = 0.0F;
  setpoint_pub_->publish(target);
}

}  // namespace banyu_mission
