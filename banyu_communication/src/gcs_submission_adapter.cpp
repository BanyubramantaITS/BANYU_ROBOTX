#include "banyu_communication/gcs_submission_adapter.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include "banyu_msgs/task2_submission.hpp"
#include "rx_msgs/msg/rx_task.hpp"

namespace banyu_communication
{

GcsSubmissionAdapter::GcsSubmissionAdapter(const rclcpp::NodeOptions & options)
: Node("gcs_submission_adapter", options)
{
  forward_to_rx_topics_ = declare_parameter<bool>("forward_to_rx_topics", true);
  lifecycle_timeout_s_ = declare_parameter<double>("lifecycle_timeout_s", 5.0);

  const auto submission_topic = declare_parameter<std::string>(
    "topics.task2_submission", "/banyu/communication/task2_submission");
  const auto lifecycle_topic = declare_parameter<std::string>(
    "topics.run_lifecycle", "/banyu/mission/run_lifecycle");
  const auto command_topic = declare_parameter<std::string>(
    "topics.command", "/system/mission/command");
  const auto survey_topic = declare_parameter<std::string>(
    "topics.pipeline_survey", "/system/vehicle/uuv/task2/pipeline_survey");
  const auto delivery_topic = declare_parameter<std::string>(
    "topics.delivery", "/system/vehicle/uuv/task2/delivery");

  const char * home = std::getenv("HOME");
  std::string default_journal = "/tmp/banyu_gcs_adapter_dedup.journal";
  journal_path_ = declare_parameter<std::string>("journal_path", default_journal);
  if (home && journal_path_.rfind("~/", 0) == 0) {
    journal_path_ = std::string(home) + journal_path_.substr(1);
  }
  loadJournal();

  submission_sub_ = create_subscription<banyu_msgs::msg::Task2Submission>(
    submission_topic, rclcpp::QoS(32).reliable().transient_local(),
    std::bind(&GcsSubmissionAdapter::handleSubmission, this, std::placeholders::_1));

  lifecycle_sub_ = create_subscription<banyu_msgs::msg::RunLifecycle>(
    lifecycle_topic, rclcpp::QoS(10).reliable().transient_local(),
    std::bind(&GcsSubmissionAdapter::handleLifecycle, this, std::placeholders::_1));

  command_sub_ = create_subscription<rx_msgs::msg::Command>(
    command_topic, rclcpp::QoS(10).reliable(),
    std::bind(&GcsSubmissionAdapter::handleCommand, this, std::placeholders::_1));

  survey_pub_ = create_publisher<rx_msgs::msg::PipelineSurvey>(
    survey_topic, rclcpp::QoS(10).reliable().transient_local());

  delivery_pub_ = create_publisher<rx_msgs::msg::ResourceDelivery>(
    delivery_topic, rclcpp::QoS(10).reliable().transient_local());

  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&GcsSubmissionAdapter::handleParameters, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "GcsSubmissionAdapter initialized: forward_to_rx=%d, submission_topic='%s', "
    "lifecycle_topic='%s', active_run_id=%u, journal='%s'",
    forward_to_rx_topics_, submission_topic.c_str(), lifecycle_topic.c_str(),
    active_run_id_, journal_path_.c_str());
}

std::size_t GcsSubmissionAdapter::acceptedCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return accepted_count_;
}

std::size_t GcsSubmissionAdapter::duplicateCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return duplicate_count_;
}

std::size_t GcsSubmissionAdapter::rejectedCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return rejected_count_;
}

std::size_t GcsSubmissionAdapter::pendingCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t count = 0;
  for (const auto & pair : accepted_by_key_) {
    if (pair.second.pub_state == PublicationState::PENDING) {
      ++count;
    }
  }
  return count;
}

std::size_t GcsSubmissionAdapter::attemptedCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::size_t count = 0;
  for (const auto & pair : accepted_by_key_) {
    if (pair.second.pub_state == PublicationState::ATTEMPTED) {
      ++count;
    }
  }
  return count;
}

std::size_t GcsSubmissionAdapter::corruptEntryCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return corrupt_entry_count_;
}

uint32_t GcsSubmissionAdapter::activeRunId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_run_id_;
}

uint64_t GcsSubmissionAdapter::activeEpoch() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return active_epoch_;
}

uint32_t GcsSubmissionAdapter::historicalHintRunId() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return historical_hint_run_id_;
}

bool GcsSubmissionAdapter::hasReport(const std::string & report_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return report_id_to_key_.find(report_id) != report_id_to_key_.end();
}

GcsSubmissionAdapter::PublicationState GcsSubmissionAdapter::publicationState(
  const std::string & report_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = report_id_to_key_.find(report_id);
  if (it == report_id_to_key_.end()) {
    return PublicationState::PENDING;
  }
  const auto entry_it = accepted_by_key_.find(it->second);
  if (entry_it == accepted_by_key_.end()) {
    return PublicationState::PENDING;
  }
  return entry_it->second.pub_state;
}

bool GcsSubmissionAdapter::hasValidAuthority(uint32_t run_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return hasValidAuthorityLocked(run_id);
}

bool GcsSubmissionAdapter::lifecycleReceived() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return lifecycle_received_;
}

bool GcsSubmissionAdapter::hasValidAuthorityLocked(uint32_t run_id) const
{
  if (!lifecycle_received_ || active_run_id_ == 0 || run_id != active_run_id_) {
    return false;
  }
  const bool terminal = (phase_ == banyu_msgs::msg::RunLifecycle::SUCCEEDED ||
    phase_ == banyu_msgs::msg::RunLifecycle::FAILED ||
    phase_ == banyu_msgs::msg::RunLifecycle::INTERRUPTED);
  if (terminal) {
    return true;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_lifecycle_time_).count();
  return elapsed <= lifecycle_timeout_s_;
}

rcl_interfaces::msg::SetParametersResult GcsSubmissionAdapter::handleParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto & param : parameters) {
    if (param.get_name() == "forward_to_rx_topics") {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool prev = forward_to_rx_topics_;
      forward_to_rx_topics_ = param.as_bool();
      RCLCPP_INFO(
        get_logger(),
        "Runtime parameter update: forward_to_rx_topics changed from %d to %d",
        prev, forward_to_rx_topics_);
      if (!prev && forward_to_rx_topics_) {
        reconcilePendingReportsLocked();
      }
    }
  }
  return result;
}

void GcsSubmissionAdapter::handleCommand(const rx_msgs::msg::Command::SharedPtr msg)
{
  if (!msg) {
    return;
  }
  if (msg->target_vehicle != rx_msgs::msg::VehicleType::TYPE_UUV &&
    msg->target_vehicle != rx_msgs::msg::VehicleType::TYPE_UNKNOWN)
  {
    RCLCPP_DEBUG(
      get_logger(),
      "Ignoring raw command (seq=%u, type=%u) addressed to other vehicle type=%u",
      msg->command_seq, msg->command_type, msg->target_vehicle);
    return;
  }
  RCLCPP_INFO(
    get_logger(),
    "Observed command seq=%u, type=%u, run_id=%u for target=%u (diagnostic only; "
    "run ownership governed by mission RunLifecycle)",
    msg->command_seq, msg->command_type, msg->run_id, msg->target_vehicle);
}

void GcsSubmissionAdapter::handleLifecycle(const banyu_msgs::msg::RunLifecycle::SharedPtr msg)
{
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (!msg) {
    return;
  }
  if ((msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ||
    msg->header.stamp.nanosec >= 1000000000U)
  {
    RCLCPP_WARN(get_logger(), "Rejected RunLifecycle with invalid header timestamp");
    return;
  }

  const double source_age = (now() - rclcpp::Time(msg->header.stamp,
    get_clock()->get_clock_type())).seconds();
  const bool is_terminal = (msg->phase == Lifecycle::INTERRUPTED ||
    msg->phase == Lifecycle::FAILED ||
    msg->phase == Lifecycle::SUCCEEDED ||
    msg->event == Lifecycle::TERMINAL);

  if (source_age < -1.0 || (!is_terminal && source_age > lifecycle_timeout_s_)) {
    RCLCPP_WARN(
      get_logger(),
      "Rejected RunLifecycle with out-of-window source age: %.3fs (timeout: %.3fs)",
      source_age, lifecycle_timeout_s_);
    return;
  }

  if (msg->authority_instance.empty() || msg->revision == 0 ||
    msg->phase > Lifecycle::INTERRUPTED ||
    (msg->phase >= Lifecycle::INITIALIZING && (msg->run_id == 0 || msg->epoch == 0)))
  {
    RCLCPP_WARN(
      get_logger(),
      "Rejected RunLifecycle with empty authority, invalid phase, or zero run/epoch in active phase");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (retired_authorities_.count(msg->authority_instance) != 0) {
    RCLCPP_WARN(
      get_logger(), "Rejected RunLifecycle from retired authority instance '%s'",
      msg->authority_instance.c_str());
    return;
  }

  if (!authority_instance_.empty() && msg->authority_instance != authority_instance_) {
    const bool live = (phase_ >= Lifecycle::INITIALIZING && phase_ <= Lifecycle::SURFACED);
    const bool fresh = lifecycle_received_ && (std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_lifecycle_time_).count() <= lifecycle_timeout_s_);
    if (live && fresh && msg->epoch == active_epoch_) {
      RCLCPP_WARN(
        get_logger(),
        "Rejected RunLifecycle authority replacement while previous authority is active and fresh");
      return;
    }
    retired_authorities_.insert(authority_instance_);
  }

  if (active_epoch_ != 0 && msg->epoch < active_epoch_) {
    RCLCPP_WARN(
      get_logger(),
      "Rejected RunLifecycle with lower epoch=%lu than active_epoch=%lu",
      msg->epoch, active_epoch_);
    return;
  }
  if (active_epoch_ != 0 && msg->epoch == active_epoch_ && active_run_id_ != 0 &&
    msg->run_id != active_run_id_)
  {
    RCLCPP_WARN(
      get_logger(),
      "Rejected RunLifecycle with conflicting run_id=%u for current active epoch=%lu",
      msg->run_id, msg->epoch);
    return;
  }

  if (lifecycle_received_) {
    if (!authority_instance_.empty() && msg->authority_instance == authority_instance_ &&
      msg->revision <= lifecycle_revision_)
    {
      return;
    }
  } else {
    if (!authority_instance_.empty() && msg->authority_instance == authority_instance_ &&
      msg->revision < lifecycle_revision_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Rejected RunLifecycle with stale revision=%lu compared to persisted revision=%lu",
        msg->revision, lifecycle_revision_);
      return;
    }
  }

  if (msg->event == Lifecycle::START_REJECTED) {
    RCLCPP_INFO(
      get_logger(),
      "RunLifecycle reported START_REJECTED for requested_run_id=%u; accepted run_id=%u preserved",
      msg->requested_run_id, msg->run_id);
  }

  const bool is_start_accepted = (msg->event == Lifecycle::START_ACCEPTED);
  const bool is_active_or_terminal_context = (msg->phase >= Lifecycle::INITIALIZING &&
    msg->phase <= Lifecycle::INTERRUPTED);

  if ((is_start_accepted || is_active_or_terminal_context) && msg->run_id != 0 && msg->epoch != 0) {
    const bool is_new_run = (msg->run_id != active_run_id_ || msg->epoch > active_epoch_);
    if (is_new_run) {
      std::string run_err;
      if (!appendActiveContext(msg->run_id, msg->epoch, msg->authority_instance,
          msg->revision, run_err))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to persist active context (run_id=%u, epoch=%lu) to journal '%s': %s; "
          "in-memory authority preserved without mutation",
          msg->run_id, msg->epoch, journal_path_.c_str(), run_err.c_str());
        return;
      }

      const uint32_t prev_run = active_run_id_;
      active_run_id_ = msg->run_id;
      active_epoch_ = msg->epoch;

      for (auto & pair : accepted_by_key_) {
        auto & report = pair.second;
        if ((report.run_id != active_run_id_ || report.epoch < active_epoch_) &&
          report.pub_state == PublicationState::PENDING)
        {
          report.pub_state = PublicationState::SUPPRESSED_NEW_RUN;
          std::string sup_err;
          if (!appendPubSuppressed(report, sup_err)) {
            RCLCPP_ERROR(
              get_logger(),
              "Failed to persist suppressed report marker for '%s' to journal '%s': %s; "
              "preserving pending state in memory",
              report.report_id.c_str(), journal_path_.c_str(), sup_err.c_str());
            report.pub_state = PublicationState::PENDING;
          } else {
            RCLCPP_WARN(
              get_logger(),
              "Unresolved report '%s' (run_id=%u, epoch=%lu) suppressed under new accepted run_id=%u, epoch=%lu",
              report.report_id.c_str(), report.run_id, report.epoch, active_run_id_, active_epoch_);
          }
        }
      }

      RCLCPP_INFO(
        get_logger(),
        "Authoritative run accepted: run_id=%u, epoch=%lu (previous run_id=%u)",
        active_run_id_, active_epoch_, prev_run);
    }
  }

  authority_instance_ = msg->authority_instance;
  lifecycle_revision_ = msg->revision;
  last_lifecycle_time_ = std::chrono::steady_clock::now();
  phase_ = msg->phase;
  lifecycle_received_ = true;

  if (is_terminal) {
    RCLCPP_INFO(
      get_logger(),
      "RunLifecycle reported terminal phase=%u (preserving durable accepted reports)",
      msg->phase);
  }

  reconcilePendingReportsLocked();
}

GcsSubmissionAdapter::PublishOutcome GcsSubmissionAdapter::publishPublicReport(
  const banyu_msgs::msg::Task2Submission & msg)
{
  if (!forward_to_rx_topics_) {
    return PublishOutcome::Disabled;
  }

  if (msg.kind == banyu_msgs::msg::Task2Submission::SURVEY) {
    if (!survey_pub_) {
      RCLCPP_ERROR(get_logger(), "Survey publisher is uninitialized");
      return PublishOutcome::Failed;
    }
    rx_msgs::msg::PipelineSurvey survey;
    survey.header = msg.header;
    if (survey.header.stamp.sec == 0 && survey.header.stamp.nanosec == 0) {
      survey.header.stamp = now();
    }
    if (survey.header.frame_id.empty()) {
      survey.header.frame_id = "uuv";
    }
    survey.active_buoy_position.latitude = msg.buoy_latitude;
    survey.active_buoy_position.longitude = msg.buoy_longitude;
    survey.segments.assign(
      msg.original_conditions.begin(),
      msg.original_conditions.end());
    try {
      survey_pub_->publish(survey);
      return PublishOutcome::Attempted;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Exception while publishing pipeline survey: %s", e.what());
      return PublishOutcome::Failed;
    }
  } else if (msg.kind == banyu_msgs::msg::Task2Submission::DELIVERY) {
    if (!delivery_pub_) {
      RCLCPP_ERROR(get_logger(), "Delivery publisher is uninitialized");
      return PublishOutcome::Failed;
    }
    rx_msgs::msg::ResourceDelivery delivery;
    delivery.header = msg.header;
    if (delivery.header.stamp.sec == 0 && delivery.header.stamp.nanosec == 0) {
      delivery.header.stamp = now();
    }
    if (delivery.header.frame_id.empty()) {
      delivery.header.frame_id = "uuv";
    }
    delivery.task = rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR;
    delivery.resource_color = msg.resource_color;
    delivery.delivery_circle_color = msg.delivery_circle_color;
    try {
      delivery_pub_->publish(delivery);
      return PublishOutcome::Attempted;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Exception while publishing resource delivery: %s", e.what());
      return PublishOutcome::Failed;
    }
  }

  RCLCPP_ERROR(get_logger(), "Unsupported Task2Submission kind=%u", msg.kind);
  return PublishOutcome::Failed;
}

void GcsSubmissionAdapter::reconcilePendingReportsLocked()
{
  if (!forward_to_rx_topics_ || active_run_id_ == 0) {
    return;
  }

  for (auto & pair : accepted_by_key_) {
    auto & report = pair.second;
    if (report.pub_state == PublicationState::PENDING &&
      hasValidAuthorityLocked(report.run_id))
    {
      banyu_msgs::msg::Task2Submission sub;
      if (!banyu_msgs::task2::parseSubmission(report.payload, sub)) {
        RCLCPP_ERROR(
          get_logger(),
          "Failed to reconstruct Task2Submission payload for pending report '%s'",
          report.report_id.c_str());
        continue;
      }
      PublishOutcome outcome = publishPublicReport(sub);
      if (outcome == PublishOutcome::Attempted) {
        std::string err;
        if (!appendPubAttempted(report, err)) {
          RCLCPP_ERROR(
            get_logger(),
            "Publication issued for recovered report '%s', but persisting ATTEMPTED marker failed: %s. "
            "Durable journal remains PENDING; replay window acknowledged.",
            report.report_id.c_str(), err.c_str());
          report.pub_state = PublicationState::PENDING;
        } else {
          report.pub_state = PublicationState::ATTEMPTED;
          RCLCPP_INFO(
            get_logger(),
            "Recovered and forwarded pending report '%s' (run_id=%u, epoch=%lu)",
            report.report_id.c_str(), report.run_id, report.epoch);
        }
      }
    }
  }
}

void GcsSubmissionAdapter::handleSubmission(const banyu_msgs::msg::Task2Submission::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }

  if (msg->run_id == 0 || msg->epoch == 0 || msg->report_id.empty() ||
    msg->authorization_revision == 0 ||
    !banyu_msgs::task2::validSubmission(*msg))
  {
    ++rejected_count_;
    RCLCPP_WARN(
      get_logger(),
      "Rejected invalid Task2Submission (run_id=%u, epoch=%lu, report_id='%s', auth_rev=%lu)",
      msg->run_id, msg->epoch, msg->report_id.c_str(), msg->authorization_revision);
    return;
  }

  if (active_run_id_ != 0 && msg->run_id != active_run_id_) {
    ++rejected_count_;
    RCLCPP_WARN(
      get_logger(),
      "Rejected Task2Submission (report_id='%s', run_id=%u) mismatched with active run_id=%u",
      msg->report_id.c_str(), msg->run_id, active_run_id_);
    return;
  }

  if (active_epoch_ != 0 && msg->epoch < active_epoch_) {
    ++rejected_count_;
    RCLCPP_WARN(
      get_logger(),
      "Rejected Task2Submission (report_id='%s', epoch=%lu) with stale epoch compared to active epoch=%lu",
      msg->report_id.c_str(), msg->epoch, active_epoch_);
    return;
  }

  const std::string payload = banyu_msgs::task2::canonicalPayload(*msg);
  const auto key = std::make_tuple(msg->run_id, msg->epoch, msg->kind);

  const auto key_it = accepted_by_key_.find(key);
  if (key_it != accepted_by_key_.end()) {
    if (key_it->second.report_id == msg->report_id && key_it->second.payload == payload) {
      if (key_it->second.pub_state == PublicationState::PENDING) {
        if (!forward_to_rx_topics_) {
          RCLCPP_INFO(
            get_logger(),
            "Retransmitted Task2Submission (report_id='%s') remains PENDING (forwarding disabled)",
            msg->report_id.c_str());
          return;
        }
        if (hasValidAuthorityLocked(msg->run_id)) {
          RCLCPP_INFO(
            get_logger(),
            "Publishing pending Task2Submission (report_id='%s')",
            msg->report_id.c_str());
          PublishOutcome outcome = publishPublicReport(*msg);
          if (outcome == PublishOutcome::Attempted) {
            std::string err;
            if (!appendPubAttempted(key_it->second, err)) {
              RCLCPP_ERROR(
                get_logger(),
                "Publication issued for report '%s', but persisting ATTEMPTED marker failed: %s. "
                "Durable journal remains PENDING; replay window acknowledged.",
                msg->report_id.c_str(), err.c_str());
              key_it->second.pub_state = PublicationState::PENDING;
            } else {
              key_it->second.pub_state = PublicationState::ATTEMPTED;
            }
          }
        } else {
          RCLCPP_WARN(
            get_logger(),
            "Retransmitted Task2Submission (report_id='%s') remains PENDING (awaiting validated mission RunLifecycle for run_id=%u, epoch=%lu)",
            msg->report_id.c_str(), msg->run_id, msg->epoch);
        }
        return;
      }
      if (key_it->second.pub_state == PublicationState::ATTEMPTED) {
        ++duplicate_count_;
        RCLCPP_INFO(
          get_logger(),
          "Deduplicated retransmitted Task2Submission (report_id='%s')",
          msg->report_id.c_str());
        return;
      }
      ++rejected_count_;
      RCLCPP_WARN(
        get_logger(),
        "Rejected suppressed Task2Submission (report_id='%s')",
        msg->report_id.c_str());
      return;
    }
    ++rejected_count_;
    RCLCPP_WARN(
      get_logger(),
      "Rejected conflicting Task2Submission for existing logical run_id=%u, epoch=%lu, kind=%u",
      msg->run_id, msg->epoch, msg->kind);
    return;
  }

  const auto id_it = report_id_to_key_.find(msg->report_id);
  if (id_it != report_id_to_key_.end()) {
    ++rejected_count_;
    RCLCPP_WARN(
      get_logger(),
      "Rejected Task2Submission with reused report_id='%s'",
      msg->report_id.c_str());
    return;
  }

  AcceptedReport report{
    msg->run_id, msg->epoch, msg->kind, msg->report_id, payload, msg->authorization_revision,
    PublicationState::PENDING};

  std::string err;
  if (!appendAcceptance(report, err)) {
    ++rejected_count_;
    RCLCPP_ERROR(
      get_logger(),
      "Failed to persist accepted report to journal '%s': %s",
      journal_path_.c_str(), err.c_str());
    return;
  }

  accepted_by_key_.emplace(key, report);
  report_id_to_key_.emplace(msg->report_id, key);
  ++accepted_count_;

  RCLCPP_INFO(
    get_logger(),
    "Accepted Task2Submission: report_id='%s', run_id=%u, epoch=%lu, kind=%u, tier=%u, auth_rev=%lu",
    msg->report_id.c_str(), msg->run_id, msg->epoch, msg->kind, msg->tier,
    msg->authorization_revision);

  if (forward_to_rx_topics_ && hasValidAuthorityLocked(msg->run_id)) {
    PublishOutcome outcome = publishPublicReport(*msg);
    if (outcome == PublishOutcome::Attempted) {
      auto & stored = accepted_by_key_[key];
      if (!appendPubAttempted(stored, err)) {
        RCLCPP_ERROR(
          get_logger(),
          "Publication issued for report '%s', but persisting ATTEMPTED marker failed: %s. "
          "Durable journal remains PENDING; replay window acknowledged.",
          stored.report_id.c_str(), err.c_str());
        stored.pub_state = PublicationState::PENDING;
      } else {
        stored.pub_state = PublicationState::ATTEMPTED;
      }
    }
  } else if (!forward_to_rx_topics_) {
    RCLCPP_INFO(
      get_logger(),
      "Task2Submission '%s' accepted and retained PENDING (forwarding disabled)",
      msg->report_id.c_str());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "Task2Submission '%s' accepted and retained PENDING (awaiting validated mission RunLifecycle for run_id=%u, epoch=%lu)",
      msg->report_id.c_str(), msg->run_id, msg->epoch);
  }
}

void GcsSubmissionAdapter::loadJournal()
{
  if (journal_path_.empty()) {
    return;
  }
  std::ifstream file(journal_path_);
  if (!file.is_open()) {
    if (std::filesystem::exists(journal_path_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to open existing deduplication journal '%s'",
        journal_path_.c_str());
    }
    return;
  }
  std::string line;
  std::size_t line_num = 0;
  while (std::getline(file, line)) {
    ++line_num;
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    iss.imbue(std::locale::classic());
    std::string tag;
    if (!(iss >> tag)) {
      continue;
    }
    bool parsed = false;
    if (tag == "BANYU_GCS_ADAPTER_ENTRY_V3") {
      AcceptedReport report;
      unsigned kind_val = 0;
      std::string state_str;
      if (iss >> report.run_id >> report.epoch >> kind_val >> report.report_id >>
        report.authorization_revision >> state_str >> std::quoted(report.payload))
      {
        report.kind = static_cast<uint8_t>(kind_val);
        if (state_str == "ATTEMPTED") {
          report.pub_state = PublicationState::ATTEMPTED;
        } else if (state_str == "SUPPRESSED_NEW_RUN") {
          report.pub_state = PublicationState::SUPPRESSED_NEW_RUN;
        } else {
          report.pub_state = PublicationState::PENDING;
        }
        const auto key = std::make_tuple(report.run_id, report.epoch, report.kind);
        accepted_by_key_[key] = report;
        report_id_to_key_[report.report_id] = key;
        parsed = true;
      }
    } else if (tag == "BANYU_GCS_ADAPTER_PUB_ATTEMPTED") {
      uint32_t run_id = 0;
      uint64_t epoch = 0;
      unsigned kind_val = 0;
      std::string report_id;
      if (iss >> run_id >> epoch >> kind_val >> report_id) {
        const auto key = std::make_tuple(run_id, epoch, static_cast<uint8_t>(kind_val));
        auto it = accepted_by_key_.find(key);
        if (it != accepted_by_key_.end()) {
          it->second.pub_state = PublicationState::ATTEMPTED;
        } else {
          auto id_it = report_id_to_key_.find(report_id);
          if (id_it != report_id_to_key_.end()) {
            accepted_by_key_[id_it->second].pub_state = PublicationState::ATTEMPTED;
          }
        }
        parsed = true;
      }
    } else if (tag == "BANYU_GCS_ADAPTER_PUB_SUPPRESSED") {
      uint32_t run_id = 0;
      uint64_t epoch = 0;
      unsigned kind_val = 0;
      std::string report_id;
      if (iss >> run_id >> epoch >> kind_val >> report_id) {
        const auto key = std::make_tuple(run_id, epoch, static_cast<uint8_t>(kind_val));
        auto it = accepted_by_key_.find(key);
        if (it != accepted_by_key_.end()) {
          it->second.pub_state = PublicationState::SUPPRESSED_NEW_RUN;
        } else {
          auto id_it = report_id_to_key_.find(report_id);
          if (id_it != report_id_to_key_.end()) {
            accepted_by_key_[id_it->second].pub_state = PublicationState::SUPPRESSED_NEW_RUN;
          }
        }
        parsed = true;
      }
    } else if (tag == "BANYU_GCS_ADAPTER_ACTIVE_CONTEXT") {
      uint32_t run_id = 0;
      uint64_t epoch = 0;
      uint64_t rev = 0;
      std::string auth;
      if (iss >> run_id >> epoch >> rev >> auth) {
        active_run_id_ = run_id;
        active_epoch_ = epoch;
        lifecycle_revision_ = rev;
        authority_instance_ = auth;
        parsed = true;
      }
    } else if (tag == "BANYU_GCS_ADAPTER_ACTIVE_RUN") {
      uint32_t run_id = 0;
      if (iss >> run_id) {
        historical_hint_run_id_ = run_id;
        RCLCPP_INFO(
          get_logger(),
          "Observed historical ACTIVE_RUN hint run_id=%u in journal '%s' (not authoritative; "
          "awaiting mission RunLifecycle)",
          run_id, journal_path_.c_str());
        parsed = true;
      }
    } else if (tag == "BANYU_GCS_ADAPTER_ENTRY_V2") {
      AcceptedReport report;
      unsigned kind_val = 0;
      if (iss >> report.run_id >> report.epoch >> kind_val >> report.report_id >>
        report.authorization_revision >> std::quoted(report.payload))
      {
        report.kind = static_cast<uint8_t>(kind_val);
        report.pub_state = PublicationState::PENDING;
        const auto key = std::make_tuple(report.run_id, report.epoch, report.kind);
        accepted_by_key_[key] = report;
        report_id_to_key_[report.report_id] = key;
        parsed = true;
      }
    } else if (tag == "BANYU_GCS_ADAPTER_ENTRY_V1") {
      std::string report_id, payload;
      if (iss >> report_id >> std::quoted(payload)) {
        AcceptedReport report;
        report.report_id = report_id;
        report.payload = payload;
        report.pub_state = PublicationState::ATTEMPTED;
        banyu_msgs::msg::Task2Submission sub;
        if (banyu_msgs::task2::parseSubmission(payload, sub)) {
          report.run_id = sub.run_id;
          report.epoch = sub.epoch;
          report.kind = sub.kind;
          report.authorization_revision = sub.authorization_revision;
          const auto key = std::make_tuple(sub.run_id, sub.epoch, sub.kind);
          accepted_by_key_[key] = report;
          report_id_to_key_[report_id] = key;
        } else {
          const auto key = std::make_tuple(
            0u, 0ul, static_cast<uint8_t>(accepted_by_key_.size() + 1));
          accepted_by_key_[key] = report;
          report_id_to_key_[report_id] = key;
        }
        parsed = true;
      }
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Unknown journal tag '%s' on line %zu in '%s'",
        tag.c_str(), line_num, journal_path_.c_str());
      continue;
    }
    if (!parsed) {
      ++corrupt_entry_count_;
      RCLCPP_ERROR(
        get_logger(),
        "Corrupt or truncated journal line %zu in '%s': %s",
        line_num, journal_path_.c_str(), line.c_str());
    }
  }

  accepted_count_ = accepted_by_key_.size();
  std::size_t pending_n = 0;
  for (const auto & pair : accepted_by_key_) {
    if (pair.second.pub_state == PublicationState::PENDING) {
      ++pending_n;
    }
  }
  RCLCPP_INFO(
    get_logger(),
    "Loaded %zu accepted reports (%zu pending publication, active_run_id=%u) from journal '%s'",
    accepted_by_key_.size(), pending_n, active_run_id_, journal_path_.c_str());
}

bool GcsSubmissionAdapter::appendJournalLine(const std::string & line, std::string & error)
{
  if (journal_path_.empty()) {
    error = "Journal path is empty";
    return false;
  }
  std::error_code ec;
  const auto parent = std::filesystem::path(journal_path_).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      error = "create_directories failed: " + ec.message();
      return false;
    }
  }

  const int fd = ::open(journal_path_.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
  if (fd == -1) {
    error = "open journal failed: " + std::string(std::strerror(errno));
    return false;
  }

  std::size_t offset = 0;
  while (offset < line.size()) {
    const ssize_t written = ::write(fd, line.data() + offset, line.size() - offset);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      error = "write journal failed: " + std::string(std::strerror(errno));
      ::close(fd);
      return false;
    }
    offset += static_cast<std::size_t>(written);
  }

  if (::fsync(fd) != 0) {
    error = "fsync journal failed: " + std::string(std::strerror(errno));
    ::close(fd);
    return false;
  }

  if (::close(fd) != 0) {
    error = "close journal failed: " + std::string(std::strerror(errno));
    return false;
  }

  if (!parent.empty()) {
    const int dir_fd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (dir_fd != -1) {
      ::fsync(dir_fd);
      ::close(dir_fd);
    }
  }

  return true;
}

bool GcsSubmissionAdapter::appendAcceptance(const AcceptedReport & report, std::string & error)
{
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << "BANYU_GCS_ADAPTER_ENTRY_V3 " << report.run_id << " " << report.epoch << " "
      << static_cast<unsigned>(report.kind) << " " << report.report_id << " "
      << report.authorization_revision << " PENDING " << std::quoted(report.payload) << "\n";
  return appendJournalLine(oss.str(), error);
}

bool GcsSubmissionAdapter::appendPubAttempted(const AcceptedReport & report, std::string & error)
{
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << "BANYU_GCS_ADAPTER_PUB_ATTEMPTED " << report.run_id << " " << report.epoch << " "
      << static_cast<unsigned>(report.kind) << " " << report.report_id << "\n";
  return appendJournalLine(oss.str(), error);
}

bool GcsSubmissionAdapter::appendPubSuppressed(const AcceptedReport & report, std::string & error)
{
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << "BANYU_GCS_ADAPTER_PUB_SUPPRESSED " << report.run_id << " " << report.epoch << " "
      << static_cast<unsigned>(report.kind) << " " << report.report_id << "\n";
  return appendJournalLine(oss.str(), error);
}

bool GcsSubmissionAdapter::appendActiveRun(uint32_t run_id, std::string & error)
{
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << "BANYU_GCS_ADAPTER_ACTIVE_RUN " << run_id << "\n";
  return appendJournalLine(oss.str(), error);
}

bool GcsSubmissionAdapter::appendActiveContext(
  uint32_t run_id, uint64_t epoch, const std::string & authority_instance,
  uint64_t revision, std::string & error)
{
  std::ostringstream oss;
  oss.imbue(std::locale::classic());
  oss << "BANYU_GCS_ADAPTER_ACTIVE_CONTEXT " << run_id << " " << epoch << " "
      << revision << " " << authority_instance << "\n";
  return appendJournalLine(oss.str(), error);
}

}  // namespace banyu_communication

#ifndef BANYU_COMMUNICATION_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<banyu_communication::GcsSubmissionAdapter>());
  rclcpp::shutdown();
  return 0;
}
#endif
