#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "rclcpp_action/rclcpp_action/rclcpp_action.hpp"

#include "banyu_msgs/action/acquire_pipeline.hpp"
#include "banyu_msgs/action/align_heading.hpp"
#include "banyu_msgs/action/center_on_ring.hpp"
#include "banyu_msgs/action/dive_to_depth.hpp"
#include "banyu_msgs/action/follow_pipeline.hpp"
#include "banyu_msgs/action/hold_probe_on_ring.hpp"
#include "banyu_msgs/action/navigate_surface_gps.hpp"
#include "banyu_msgs/action/search_pipeline.hpp"
#include "banyu_msgs/action/send_mission_report.hpp"
#include "banyu_msgs/action/surface.hpp"
#include "banyu_msgs/srv/acknowledge_ring_repair.hpp"
#include "banyu_mission/banyu_mission.hpp"

namespace banyu_mission
{

template<typename ActionT>
class ActionClientNode : public BT::StatefulActionNode
{
public:
  using Client = rclcpp_action::Client<ActionT>;
  using GoalHandle = typename Client::GoalHandle;
  using Goal = typename ActionT::Goal;
  using Result = typename ActionT::Result;

  ActionClientNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, std::string action_name)
  : BT::StatefulActionNode(name, config), node_(std::move(node)),
    client_(rclcpp_action::create_client<ActionT>(node_, action_name))
  {}

  BT::NodeStatus onStart() override
  {
    typename ActionT::Goal goal;
    try {
      goal = makeGoal();
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_->get_logger(), "%s goal mapping failed: %s",
        name().c_str(), error.what());
      return BT::NodeStatus::FAILURE;
    }

    if (!client_->wait_for_action_server(std::chrono::milliseconds(0))) {
      RCLCPP_DEBUG(node_->get_logger(), "%s action server is unavailable", name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      generation = ++generation_;
      goal_handle_.reset();
      result_done_ = false;
      result_success_ = false;
      cancel_requested_.store(false);
    }

    typename Client::SendGoalOptions options;
    options.goal_response_callback = [this, generation](typename GoalHandle::SharedPtr handle) {
        bool cancel = cancel_generation_.load() >= generation;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          if (generation == generation_) {
            goal_handle_ = handle;
            if (!handle) {
              result_done_ = true;
              result_success_ = false;
              return;
            }
            cancel = cancel || cancel_requested_.load();
          }
        }
        if (cancel && handle) {
          try {
            client_->async_cancel_goal(handle);
          } catch (const std::exception &) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (generation == generation_) {
              result_done_ = true;
              result_success_ = false;
            }
          }
        }
      };
    options.result_callback =
      [this, generation, goal](const typename Client::WrappedResult & result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (generation != generation_) {
          return;
        }
        result_done_ = true;
        const bool transport_success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        const auto * value = result.result.get();
        const bool semantic_success = value != nullptr && validateResult(goal, *value);
        result_success_ = transport_success && semantic_success;
        if (!result_success_) {
          RCLCPP_WARN(
            node_->get_logger(), "%s action failed: transport=%s%s%s",
            name().c_str(), transport_success ? "ok" : "failed",
            value ? ", semantic=" : "", value ? (semantic_success ? "success" : "failure") : "");
        }
        if (value) {
          reportResult(*value);
        }
      };

    try {
      client_->async_send_goal(goal, options);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_->get_logger(), "%s goal submission failed: %s",
        name().c_str(), error.what());
      std::lock_guard<std::mutex> lock(mutex_);
      result_done_ = true;
      result_success_ = false;
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!result_done_) {
      return BT::NodeStatus::RUNNING;
    }
    return result_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    typename GoalHandle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cancel_requested_.store(true);
      cancel_generation_.store(generation_);
      handle = goal_handle_;
    }
    if (handle) {
      try {
        client_->async_cancel_goal(handle);
      } catch (const std::exception &) {
        // The goal may already have reached a terminal state.
      }
    }
  }

  template<typename ResultT>
  static bool semanticSuccess(const ResultT & result)
  {
    return resultHasSuccess(result);
  }

protected:
  virtual typename ActionT::Goal makeGoal() = 0;

  virtual bool validateResult(const Goal &, const Result & result) const
  {
    return resultHasSuccess(result);
  }

  template<typename ResultT>
  static auto resultHasSuccess(const ResultT & result) -> decltype(result.success, bool ())
  {
    return result.success;
  }

  static bool resultHasSuccess(...)
  {
    return false;
  }

  template<typename ResultT>
  void reportResult(const ResultT & result)
  {
    reportResultFields(result, 0);
  }

  template<typename ResultT>
  auto reportResultFields(const ResultT & result, int) -> decltype(result.reason, void ())
  {
    RCLCPP_DEBUG(node_->get_logger(), "%s result: %s", name().c_str(), result.reason.c_str());
    reportAcknowledgement(result, 0);
  }

  template<typename ResultT>
  void reportResultFields(const ResultT &, long) {}

  template<typename ResultT>
  auto reportAcknowledgement(const ResultT & result, int) -> decltype(
    result.acknowledged, result.acknowledged_crc32, result.attempts, void ())
  {
    RCLCPP_INFO(
      node_->get_logger(), "%s report ack=%s crc32=%u attempts=%u",
      name().c_str(), result.acknowledged ? "true" : "false",
      result.acknowledged_crc32, result.attempts);
  }

  template<typename ResultT>
  void reportAcknowledgement(const ResultT &, long) {}

  template<typename T>
  T input(const std::string & port) const
  {
    auto value = getInput<T>(port);
    if (!value) {
      throw std::runtime_error(value.error());
    }
    return value.value();
  }

  rclcpp::Node::SharedPtr node_;

private:
  typename Client::SharedPtr client_;
  typename GoalHandle::SharedPtr goal_handle_;
  std::mutex mutex_;
  std::atomic_bool cancel_requested_{false};
  std::atomic<uint64_t> cancel_generation_{0};
  uint64_t generation_{0};
  bool result_done_{false};
  bool result_success_{false};
};

class AcknowledgeRedRingNode final : public BT::StatefulActionNode
{
public:
  AcknowledgeRedRingNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, std::shared_ptr<MissionState> state,
    std::string service_name)
  : BT::StatefulActionNode(name, config), node_(std::move(node)), state_(std::move(state)),
    client_(node_->create_client<banyu_msgs::srv::AcknowledgeRingRepair>(service_name)) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override
  {
    const uint32_t index = state_->pendingRedIndex();
    if (!state_->redPending()) {
      return BT::NodeStatus::FAILURE;
    }
    if (!client_->wait_for_service(std::chrono::milliseconds(0))) {
      RCLCPP_WARN(node_->get_logger(), "ring repair acknowledgement service unavailable");
      return BT::NodeStatus::FAILURE;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      generation_++;
      request_index_ = index;
      done_ = false;
      success_ = false;
      const auto generation = generation_;
      auto request = std::make_shared<banyu_msgs::srv::AcknowledgeRingRepair::Request>();
      request->ring_index = index;
      client_->async_send_request(
        request, [this, generation](
          rclcpp::Client<banyu_msgs::srv::AcknowledgeRingRepair>::SharedFuture future) {
          const auto response = future.get();
          std::lock_guard<std::mutex> callback_lock(mutex_);
          if (generation != generation_) {
            return;
          }
          done_ = true;
          success_ = response->success && response->accepted_ring_index == request_index_;
          if (success_) {
            state_->acknowledgeRed(request_index_);
          }
        });
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!done_) {
      return BT::NodeStatus::RUNNING;
    }
    return success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    generation_++;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<MissionState> state_;
  rclcpp::Client<banyu_msgs::srv::AcknowledgeRingRepair>::SharedPtr client_;
  std::mutex mutex_;
  uint64_t generation_{0};
  uint32_t request_index_{0};
  bool done_{false};
  bool success_{false};
};

class NavigateSurfaceGpsNode final
  : public ActionClientNode<banyu_msgs::action::NavigateSurfaceGps>
{
public:
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("target_latitude"), BT::InputPort<double>("target_longitude"),
      BT::InputPort<double>("start_latitude"), BT::InputPort<double>("start_longitude"),
      BT::InputPort<double>("tolerance")};
  }
  NavigateSurfaceGpsNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.target_latitude_deg = input<double>("target_latitude");
    goal.target_longitude_deg = input<double>("target_longitude");
    goal.start_latitude_deg = input<double>("start_latitude");
    goal.start_longitude_deg = input<double>("start_longitude");
    goal.tolerance_m = input<double>("tolerance");
    return goal;
  }
};

class AlignHeadingNode final : public ActionClientNode<banyu_msgs::action::AlignHeading>
{
public:
  static BT::PortsList providedPorts()
  {return {BT::InputPort<double>("target_heading"), BT::InputPort<double>("tolerance")};}
  AlignHeadingNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.target_heading_rad = input<double>("target_heading");
    goal.tolerance_rad = input<double>("tolerance");
    return goal;
  }
};

class DiveToDepthNode final : public ActionClientNode<banyu_msgs::action::DiveToDepth>
{
public:
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("target_depth"), BT::InputPort<double>("target_heading"),
      BT::InputPort<double>("depth_tolerance"), BT::InputPort<double>("heading_tolerance")};
  }
  DiveToDepthNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.target_depth_m = input<double>("target_depth");
    goal.target_heading_rad = input<double>("target_heading");
    goal.depth_tolerance_m = input<double>("depth_tolerance");
    goal.heading_tolerance_rad = input<double>("heading_tolerance");
    return goal;
  }
};

class AcquirePipelineNode final : public ActionClientNode<banyu_msgs::action::AcquirePipeline>
{
public:
  using Base = ActionClientNode<banyu_msgs::action::AcquirePipeline>;
  using Result = Base::Result;

  static bool semanticResultSuccess(const Result & result)
  {
    return result.success && result.acquired;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("camera_name"), BT::InputPort<bool>("use_heading_prior"),
      BT::InputPort<double>("heading_prior"), BT::InputPort<double>("heading_tolerance")};
  }
  AcquirePipelineNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.camera_name = input<std::string>("camera_name");
    goal.use_heading_prior = input<bool>("use_heading_prior");
    goal.heading_prior_rad = input<double>("heading_prior");
    goal.heading_prior_tolerance_rad = input<double>("heading_tolerance");
    return goal;
  }

  bool validateResult(const Goal &, const Result & result) const override
  {
    return semanticResultSuccess(result);
  }
};

class SearchPipelineNode final : public ActionClientNode<banyu_msgs::action::SearchPipeline>
{
public:
  using Base = ActionClientNode<banyu_msgs::action::SearchPipeline>;
  using Result = Base::Result;

  static bool semanticResultSuccess(const Result & result)
  {
    return result.success && result.found;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("camera_name"), BT::InputPort<bool>("use_heading_prior"),
      BT::InputPort<double>("heading_prior"), BT::InputPort<double>("search_span"),
      BT::InputPort<double>("sweep_rate")};
  }
  SearchPipelineNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.camera_name = input<std::string>("camera_name");
    goal.use_heading_prior = input<bool>("use_heading_prior");
    goal.heading_prior_rad = input<double>("heading_prior");
    goal.search_span_rad = input<double>("search_span");
    goal.sweep_rate_rad_s = input<double>("sweep_rate");
    return goal;
  }

  bool validateResult(const Goal &, const Result & result) const override
  {
    return semanticResultSuccess(result);
  }
};

class FollowPipelineNode final : public ActionClientNode<banyu_msgs::action::FollowPipeline>
{
public:
  using Base = ActionClientNode<banyu_msgs::action::FollowPipeline>;
  using Result = Base::Result;

  static bool semanticResultSuccess(const Result & result, bool expected_enable)
  {
    return result.success && result.enabled == expected_enable;
  }

  static BT::PortsList providedPorts() {return {BT::InputPort<bool>("enable")};}
  FollowPipelineNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.enable = input<bool>("enable");
    return goal;
  }

  bool validateResult(const Goal & goal, const Result & result) const override
  {
    return semanticResultSuccess(result, goal.enable);
  }
};

class CenterOnRingNode final : public ActionClientNode<banyu_msgs::action::CenterOnRing>
{
public:
  static BT::PortsList providedPorts()
  {return {BT::InputPort<std::string>("ring_color"), BT::InputPort<double>("tolerance")};}
  CenterOnRingNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.ring_color = input<std::string>("ring_color");
    goal.tolerance = input<double>("tolerance");
    return goal;
  }
};

class HoldProbeOnRingNode final : public ActionClientNode<banyu_msgs::action::HoldProbeOnRing>
{
public:
  static BT::PortsList providedPorts()
  {return {BT::InputPort<double>("hold_duration"), BT::InputPort<double>("alignment_tolerance")};}
  HoldProbeOnRingNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.hold_duration_s = input<double>("hold_duration");
    goal.alignment_tolerance = input<double>("alignment_tolerance");
    return goal;
  }
};

class SurfaceNode final : public ActionClientNode<banyu_msgs::action::Surface>
{
public:
  static BT::PortsList providedPorts()
  {return {BT::InputPort<double>("target_depth"), BT::InputPort<double>("depth_tolerance")};}
  SurfaceNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.target_depth_m = input<double>("target_depth");
    goal.depth_tolerance_m = input<double>("depth_tolerance");
    return goal;
  }
};

class SendMissionReportNode final
  : public ActionClientNode<banyu_msgs::action::SendMissionReport>
{
public:
  using Base = ActionClientNode<banyu_msgs::action::SendMissionReport>;
  using Result = Base::Result;

  static bool semanticResultSuccess(const Result & result, uint32_t expected_crc)
  {
    return result.success && result.acknowledged && result.acknowledged_crc32 == expected_crc;
  }

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("mission_id"), BT::InputPort<std::vector<uint8_t>>(
        "ring_order"),
      BT::InputPort<uint32_t>("crc32"), BT::InputPort<uint8_t>("max_retries")};
  }
  SendMissionReportNode(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, const std::string & action)
  : ActionClientNode(name, config, std::move(node), action) {}

protected:
  Goal makeGoal() override
  {
    Goal goal;
    goal.mission_id = input<std::string>("mission_id");
    goal.ring_order = input<std::vector<uint8_t>>("ring_order");
    goal.crc32 = input<uint32_t>("crc32");
    goal.max_retries = input<uint8_t>("max_retries");
    const auto stamp = node_->now();
    goal.report_time.sec = static_cast<int32_t>(stamp.nanoseconds() / 1000000000LL);
    goal.report_time.nanosec = static_cast<uint32_t>(stamp.nanoseconds() % 1000000000LL);
    return goal;
  }

  bool validateResult(const Goal & goal, const Result & result) const override
  {
    return semanticResultSuccess(result, goal.crc32);
  }
};

}  // namespace banyu_mission
