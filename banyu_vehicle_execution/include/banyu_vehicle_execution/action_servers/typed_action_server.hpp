#ifndef BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__TYPED_ACTION_SERVER_HPP_
#define BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__TYPED_ACTION_SERVER_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "banyu_vehicle_execution/action_servers/action_server_base.hpp"
#include "banyu_vehicle_execution/action_servers/action_server_coordinator.hpp"

namespace banyu_vehicle_execution
{

template<typename ActionT>
class TypedActionServer : public ActionServerBase
{
public:
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;

  TypedActionServer(
    rclcpp::Node * node,
    const std::string & action_name,
    ActionServerCoordinator * coordinator)
  : node_(node), action_name_(action_name), coordinator_(coordinator)
  {
    server_ = rclcpp_action::create_server<ActionT>(
      node_,
      action_name_,
      std::bind(&TypedActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TypedActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&TypedActionServer::handleAccepted, this, std::placeholders::_1));
  }

  std::string name() const override {return action_name_;}
  bool hasActiveGoal() const override {return active_handle_ != nullptr;}
  bool hasPendingGoal() const override {return pending_handle_ != nullptr;}
  GoalIdentity activeIdentity() const override {return active_identity_;}
  GoalIdentity pendingIdentity() const override {return pending_identity_;}

  void activatePending() override
  {
    active_handle_ = pending_handle_;
    active_identity_ = pending_identity_;
    pending_handle_.reset();
    pending_identity_ = {};
    if (active_handle_) {
      onGoalStarted();
    }
  }

  void cancelActive() override
  {
    if (active_handle_) {
      auto result = std::make_shared<Result>();
      result->success = false;
      result->reason = "Goal cancelled";
      active_handle_->canceled(result);
      active_handle_.reset();
      active_identity_ = {};
    }
  }

  void abortActive(const std::string & reason) override
  {
    if (active_handle_) {
      auto result = std::make_shared<Result>();
      result->success = false;
      result->reason = reason;
      active_handle_->abort(result);
      active_handle_.reset();
      active_identity_ = {};
    }
  }

  void cancelPending() override
  {
    if (pending_handle_) {
      auto result = std::make_shared<Result>();
      result->success = false;
      result->reason = "Pending goal cancelled";
      pending_handle_->canceled(result);
      pending_handle_.reset();
      pending_identity_ = {};
    }
  }

  void abortPending(const std::string & reason) override
  {
    if (pending_handle_) {
      auto result = std::make_shared<Result>();
      result->success = false;
      result->reason = reason;
      pending_handle_->abort(result);
      pending_handle_.reset();
      pending_identity_ = {};
    }
  }

protected:
  virtual void onGoalStarted() {}
  virtual bool validateGoal(const Goal & /*goal*/, std::string & /*reason*/) {return true;}

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Goal> goal)
  {
    std::string reason;
    if (!validateGoal(*goal, reason)) {
      RCLCPP_WARN(
        node_->get_logger(), "%s goal rejected: %s", action_name_.c_str(),
        reason.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    GoalIdentity id;
    std::copy(uuid.begin(), uuid.end(), id.uuid.begin());
    id.owner = action_name_;
    id.generation = ++generation_counter_;

    const auto admission = coordinator_->requestGoal(id);
    if (!admission.accepted()) {
      RCLCPP_WARN(node_->get_logger(), "%s goal rejected by arbiter", action_name_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (admission.admission == GoalAdmission::Pending) {
      pending_identity_ = id;
    } else if (admission.admission == GoalAdmission::Activated) {
      active_identity_ = id;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    const auto id = goal_handle->get_goal_id();
    if (active_handle_ && active_identity_.uuid == id) {
      coordinator_->cancelGoal(banyu_vehicle_execution::GoalHandle{active_identity_});
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    if (pending_handle_ && pending_identity_.uuid == id) {
      coordinator_->cancelGoal(banyu_vehicle_execution::GoalHandle{pending_identity_});
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    return rclcpp_action::CancelResponse::REJECT;
  }

  void handleAccepted(const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    if (pending_identity_.valid() && pending_identity_.uuid == goal_handle->get_goal_id()) {
      pending_handle_ = goal_handle;
    } else if (active_identity_.valid() && active_identity_.uuid == goal_handle->get_goal_id()) {
      active_handle_ = goal_handle;
      onGoalStarted();
    }
  }

  rclcpp::Node * node_{nullptr};
  std::string action_name_;
  ActionServerCoordinator * coordinator_{nullptr};
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;

  std::shared_ptr<ServerGoalHandle> active_handle_;
  GoalIdentity active_identity_{};
  std::shared_ptr<ServerGoalHandle> pending_handle_;
  GoalIdentity pending_identity_{};
  uint64_t generation_counter_{0};
};

}  // namespace banyu_vehicle_execution

#endif  // BANYU_VEHICLE_EXECUTION__ACTION_SERVERS__TYPED_ACTION_SERVER_HPP_
