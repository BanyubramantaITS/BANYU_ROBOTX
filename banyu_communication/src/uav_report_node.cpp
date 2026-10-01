#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "banyu_communication/report_state_machine.hpp"
#include "banyu_msgs/action/send_mission_report.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace banyu_communication
{

class UavReportNode : public rclcpp::Node
{
public:
  using Action = banyu_msgs::action::SendMissionReport;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  UavReportNode()
  : Node("uav_report_node"),
    retry_state_(read_attempt_limit())
  {
    const auto retry_period_ms = std::max<int64_t>(
      declare_parameter<int64_t>("retry_period_ms", 1000), 1);
    const auto report_topic = declare_parameter<std::string>("report_topic", "mission_report");
    const auto ack_topic = declare_parameter<std::string>("ack_topic", "mission_report_ack");
    const auto action_name = declare_parameter<std::string>(
      "action_name", "send_mission_report");

    report_publisher_ = create_publisher<banyu_msgs::msg::MissionReport>(report_topic, 10);
    ack_subscription_ = create_subscription<banyu_msgs::msg::MissionReportAck>(
      ack_topic, 10,
      std::bind(&UavReportNode::ack_callback, this, std::placeholders::_1));
    action_server_ = rclcpp_action::create_server<Action>(
      this, action_name,
      std::bind(
        &UavReportNode::goal_callback, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&UavReportNode::cancel_callback, this, std::placeholders::_1),
      std::bind(&UavReportNode::accepted_callback, this, std::placeholders::_1));
    retry_timer_ = create_wall_timer(
      std::chrono::milliseconds(retry_period_ms),
      std::bind(&UavReportNode::retry_timer_callback, this));
    cancel_timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&UavReportNode::cancel_timer_callback, this));
  }

private:
  uint8_t read_attempt_limit()
  {
    const auto configured = declare_parameter<int64_t>("max_attempts", 4);
    return static_cast<uint8_t>(std::clamp<int64_t>(configured, 1, 255));
  }

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal>)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_goal_ || pending_canceled_goal_ ? rclcpp_action::GoalResponse::REJECT :
           rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_goal_ || active_goal_ != goal_handle) {
        return rclcpp_action::CancelResponse::REJECT;
      }

      // Transition and clear the retry state before returning so a concurrent ACK
      // or retry timer cannot win. The action server marks the goal canceling
      // immediately after this callback returns.
      retry_state_.cancel();
      pending_canceled_goal_ = active_goal_;
      pending_canceled_result_ = make_result(false, "goal canceled", false);
      active_goal_.reset();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void accepted_callback(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const auto validation = validate_report(goal->mission_id, goal->ring_order, goal->crc32);

    if (!validation.valid) {
      auto result = std::make_shared<Action::Result>();
      result->success = false;
      result->reason = validation.reason;
      result->acknowledged = false;
      result->attempts = 0;
      result->acknowledged_crc32 = 0;
      goal_handle->abort(result);
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_ || pending_canceled_goal_) {
      auto result = std::make_shared<Action::Result>();
      result->success = false;
      result->reason = "another report is already active";
      goal_handle->abort(result);
      return;
    }

    active_goal_ = goal_handle;
    banyu_msgs::msg::MissionReport report;
    report.mission_id = goal->mission_id;
    report.ring_sequence = goal->ring_order;
    report.source_timestamp = goal->report_time;
    report.crc32 = goal->crc32;
    retry_state_.start(report, goal->max_retries);
    publish_report_locked("published");
  }

  void publish_report_locked(const std::string & status)
  {
    const auto attempt = retry_state_.record_publication();
    report_publisher_->publish(retry_state_.report());

    if (active_goal_) {
      auto feedback = std::make_shared<Action::Feedback>();
      const auto stamp = now().nanoseconds();
      feedback->stamp.sec = static_cast<int32_t>(stamp / 1000000000LL);
      feedback->stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
      feedback->attempt = attempt;
      feedback->acknowledged = false;
      feedback->status = status;
      active_goal_->publish_feedback(feedback);
    }
  }

  void ack_callback(const banyu_msgs::msg::MissionReportAck::SharedPtr ack)
  {
    std::shared_ptr<GoalHandle> goal;
    std::shared_ptr<Action::Result> result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_goal_ ||
        retry_state_.on_ack(*ack) != ReportRetryStateMachine::AckEvent::Accepted)
      {
        return;
      }
      goal = active_goal_;
      result = make_result(true, "acknowledged", true);
      active_goal_.reset();
    }
    goal->succeed(result);
  }

  void retry_timer_callback()
  {
    std::shared_ptr<GoalHandle> goal;
    std::shared_ptr<Action::Result> result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_goal_) {
        return;
      }

      const auto event = retry_state_.on_timer();
      if (event == ReportRetryStateMachine::TimerEvent::Publish) {
        publish_report_locked("retry published");
        return;
      }
      if (event == ReportRetryStateMachine::TimerEvent::Exhausted) {
        goal = active_goal_;
        result = make_result(false, "acknowledgement timeout", false);
        active_goal_.reset();
      }
    }
    if (goal) {
      goal->abort(result);
    }
  }

  void cancel_timer_callback()
  {
    std::shared_ptr<GoalHandle> goal;
    std::shared_ptr<Action::Result> result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!pending_canceled_goal_ || !pending_canceled_goal_->is_canceling()) {
        return;
      }
      goal = pending_canceled_goal_;
      result = pending_canceled_result_;
      pending_canceled_goal_.reset();
      pending_canceled_result_.reset();
    }
    goal->canceled(result);
  }

  std::shared_ptr<Action::Result> make_result(
    bool success, const std::string & reason, bool acknowledged) const
  {
    auto result = std::make_shared<Action::Result>();
    result->success = success;
    result->reason = reason;
    result->acknowledged = acknowledged;
    result->attempts = retry_state_.attempts();
    result->acknowledged_crc32 = retry_state_.acknowledged_crc32();
    return result;
  }

  rclcpp::Publisher<banyu_msgs::msg::MissionReport>::SharedPtr report_publisher_;
  rclcpp::Subscription<banyu_msgs::msg::MissionReportAck>::SharedPtr ack_subscription_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  ReportRetryStateMachine retry_state_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::shared_ptr<GoalHandle> pending_canceled_goal_;
  std::shared_ptr<Action::Result> pending_canceled_result_;
  mutable std::mutex mutex_;
};

}  // namespace banyu_communication

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_communication::UavReportNode>());
  rclcpp::shutdown();
  return 0;
}
