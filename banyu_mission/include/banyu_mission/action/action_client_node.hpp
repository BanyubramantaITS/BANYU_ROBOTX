#ifndef BANYU_MISSION__ACTION__ACTION_CLIENT_NODE_HPP_
#define BANYU_MISSION__ACTION__ACTION_CLIENT_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "banyu_msgs/msg/action_lease.hpp"
#include "banyu_msgs/msg/run_lifecycle.hpp"
#include "banyu_mission/mission_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action/rclcpp_action.hpp"

namespace banyu_mission
{

namespace detail
{
template<typename G>
auto setGoalRunContextImpl(G & g, uint32_t run_id, uint64_t epoch, int)
-> decltype(g.run_id = run_id, g.epoch = epoch, void())
{
  g.run_id = run_id;
  g.epoch = epoch;
}

template<typename G>
void setGoalRunContextImpl(G &, uint32_t, uint64_t, ...)
{
}

template<typename G>
void setGoalRunContext(G & g, uint32_t run_id, uint64_t epoch)
{
  setGoalRunContextImpl(g, run_id, epoch, 0);
}
}  // namespace detail

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
    rclcpp::Node::SharedPtr node, std::string action_name,
    std::shared_ptr<MissionState> state = nullptr)
  : BT::StatefulActionNode(name, config), node_(std::move(node)),
    action_name_(std::move(action_name)),
    client_(rclcpp_action::create_client<ActionT>(node_, action_name_)),
    state_(std::move(state))
  {
    std::string lease_topic = "/banyu/execution/lease";
    if (node_->has_parameter("topics.lease")) {
      lease_topic = node_->get_parameter("topics.lease").as_string();
    } else {
      try {
        lease_topic =
          node_->declare_parameter<std::string>("topics.lease", "/banyu/execution/lease");
      } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException &) {
        lease_topic = node_->get_parameter("topics.lease").as_string();
      }
    }
    lease_pub_ = node_->create_publisher<banyu_msgs::msg::ActionLease>(lease_topic, rclcpp::QoS(1));

    // Goals without a MissionState take their run context from the mission's RunLifecycle.
    std::string lifecycle_topic = "/banyu/mission/run_lifecycle";
    if (node_->has_parameter("topics.run_lifecycle")) {
      lifecycle_topic = node_->get_parameter("topics.run_lifecycle").as_string();
    }
    lifecycle_sub_ = node_->create_subscription<banyu_msgs::msg::RunLifecycle>(
      lifecycle_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const banyu_msgs::msg::RunLifecycle::SharedPtr msg) {
        current_run_id_.store(msg->run_id);
        current_epoch_.store(msg->epoch);
      });
  }

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

    uint32_t run_id = state_ ? state_->currentRunId() : current_run_id_.load();
    uint64_t epoch = state_ ? state_->currentRunEpoch() : current_epoch_.load();
    detail::setGoalRunContext(goal, run_id, epoch);

    if (!client_->wait_for_action_server(std::chrono::milliseconds(0))) {
      RCLCPP_DEBUG(node_->get_logger(), "%s action server is unavailable", name().c_str());
      return BT::NodeStatus::FAILURE;
    }

    uint64_t generation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      generation = ++s_action_generation_;
      generation_ = generation;
      goal_run_id_ = run_id;
      goal_epoch_ = epoch;
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
            node_->get_logger(), "%s action failed: transport=%s%s%s%s%s",
            name().c_str(), transport_success ? "ok" : "failed",
            value ? ", semantic=" : "", value ? (semantic_success ? "success" : "failure") : "",
            (value && !value->reason.empty()) ? ", reason=" : "",
            (value && !value->reason.empty()) ? value->reason.c_str() : "");
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
    if (lease_pub_ && goal_handle_ && !result_done_) {
      banyu_msgs::msg::ActionLease lease;
      lease.header.stamp = node_->now();
      lease.header.frame_id = "mission";
      const auto id = goal_handle_->get_goal_id();
      std::copy(id.begin(), id.end(), lease.goal_id.uuid.begin());
      lease.owner = action_name_;
      lease.active = true;
      lease.generation = generation_;
      lease.run_id = goal_run_id_;
      lease.epoch = goal_epoch_;
      lease.expires_at = node_->now() + rclcpp::Duration::from_seconds(1.5);
      lease_pub_->publish(lease);
    }
    if (!result_done_) {
      return BT::NodeStatus::RUNNING;
    }
    return result_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    typename GoalHandle::SharedPtr handle;
    uint64_t gen = 0;
    uint32_t run_id = 0;
    uint64_t epoch = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cancel_requested_.store(true);
      cancel_generation_.store(generation_);
      gen = generation_;
      run_id = goal_run_id_;
      epoch = goal_epoch_;
      handle = goal_handle_;
    }
    if (lease_pub_ && handle) {
      banyu_msgs::msg::ActionLease lease;
      lease.header.stamp = node_->now();
      lease.header.frame_id = "mission";
      const auto id = handle->get_goal_id();
      std::copy(id.begin(), id.end(), lease.goal_id.uuid.begin());
      lease.owner = action_name_;
      lease.active = false;
      lease.generation = gen;
      lease.run_id = run_id;
      lease.epoch = epoch;
      lease.expires_at = node_->now();
      lease_pub_->publish(lease);
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
  std::string action_name_;

private:
  inline static std::atomic<uint64_t> s_action_generation_{0};
  typename Client::SharedPtr client_;
  typename GoalHandle::SharedPtr goal_handle_;
  rclcpp::Publisher<banyu_msgs::msg::ActionLease>::SharedPtr lease_pub_;
  std::mutex mutex_;
  std::atomic_bool cancel_requested_{false};
  std::atomic<uint64_t> cancel_generation_{0};
  uint64_t generation_{0};
  bool result_done_{false};
  bool result_success_{false};
  std::shared_ptr<MissionState> state_;
  rclcpp::Subscription<banyu_msgs::msg::RunLifecycle>::SharedPtr lifecycle_sub_;
  std::atomic<uint32_t> current_run_id_{0};
  std::atomic<uint64_t> current_epoch_{0};
  uint32_t goal_run_id_{0};
  uint64_t goal_epoch_{0};
};

}  // namespace banyu_mission

#endif  // BANYU_MISSION__ACTION__ACTION_CLIENT_NODE_HPP_
