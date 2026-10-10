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
  const auto submission_topic = declare_parameter<std::string>(
    "topics.task2_submission", "/banyu/communication/task2_submission");
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

  command_sub_ = create_subscription<rx_msgs::msg::Command>(
    command_topic, rclcpp::QoS(10).reliable(),
    std::bind(&GcsSubmissionAdapter::handleCommand, this, std::placeholders::_1));

  survey_pub_ = create_publisher<rx_msgs::msg::PipelineSurvey>(
    survey_topic, rclcpp::QoS(10).reliable().transient_local());

  delivery_pub_ = create_publisher<rx_msgs::msg::ResourceDelivery>(
    delivery_topic, rclcpp::QoS(10).reliable().transient_local());

  RCLCPP_INFO(
    get_logger(),
    "GcsSubmissionAdapter initialized: forward_to_rx=%d, submission_topic='%s', journal='%s'",
    forward_to_rx_topics_, submission_topic.c_str(), journal_path_.c_str());
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

void GcsSubmissionAdapter::handleCommand(const rx_msgs::msg::Command::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!msg) {
    return;
  }
  if (msg->command_type == rx_msgs::msg::Command::CMD_RUN_START) {
    if (msg->run_id != 0 && msg->run_id != active_run_id_) {
      active_run_id_ = msg->run_id;
      for (auto & pair : accepted_by_key_) {
        auto & report = pair.second;
        if (report.run_id != active_run_id_ && report.pub_state == PublicationState::PENDING) {
          report.pub_state = PublicationState::SUPPRESSED_NEW_RUN;
          RCLCPP_WARN(
            get_logger(),
            "Unresolved report '%s' (run_id=%u) suppressed from publishing under new run_id=%u",
            report.report_id.c_str(), report.run_id, active_run_id_);
        }
      }
    }
    RCLCPP_INFO(
      get_logger(),
      "Run start detected (run_id=%u, seq=%u); retaining historical accepted report count=%zu",
      msg->run_id, msg->command_seq, accepted_by_key_.size());
  }
}

void GcsSubmissionAdapter::publishPublicReport(const banyu_msgs::msg::Task2Submission & msg)
{
  if (!forward_to_rx_topics_) {
    return;
  }

  if (msg.kind == banyu_msgs::msg::Task2Submission::SURVEY) {
    rx_msgs::msg::PipelineSurvey survey;
    survey.header = msg.header;
    survey.active_buoy_position.latitude = msg.buoy_latitude;
    survey.active_buoy_position.longitude = msg.buoy_longitude;
    survey.segments.assign(
      msg.original_conditions.begin(),
      msg.original_conditions.end());
    survey_pub_->publish(survey);
  } else if (msg.kind == banyu_msgs::msg::Task2Submission::DELIVERY) {
    rx_msgs::msg::ResourceDelivery delivery;
    delivery.header = msg.header;
    delivery.task = rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR;
    delivery.resource_color = msg.resource_color;
    delivery.delivery_circle_color = msg.delivery_circle_color;
    delivery_pub_->publish(delivery);
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

  const std::string payload = banyu_msgs::task2::canonicalPayload(*msg);
  const auto key = std::make_tuple(msg->run_id, msg->epoch, msg->kind);

  const auto key_it = accepted_by_key_.find(key);
  if (key_it != accepted_by_key_.end()) {
    if (key_it->second.report_id == msg->report_id && key_it->second.payload == payload) {
      if (key_it->second.pub_state == PublicationState::PENDING) {
        RCLCPP_INFO(
          get_logger(),
          "Recovering and publishing pending Task2Submission (report_id='%s')",
          msg->report_id.c_str());
        publishPublicReport(*msg);
        key_it->second.pub_state = PublicationState::ATTEMPTED;
        std::string err;
        appendPubAttempted(key_it->second, err);
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

  publishPublicReport(*msg);

  auto & stored = accepted_by_key_[key];
  stored.pub_state = PublicationState::ATTEMPTED;
  appendPubAttempted(stored, err);
}

void GcsSubmissionAdapter::loadJournal()
{
  if (journal_path_.empty()) {
    return;
  }
  std::ifstream file(journal_path_);
  if (!file.is_open()) {
    return;
  }
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream iss(line);
    iss.imbue(std::locale::classic());
    std::string tag;
    if (!(iss >> tag)) {
      continue;
    }
    if (tag == "BANYU_GCS_ADAPTER_ENTRY_V3") {
      AcceptedReport report;
      unsigned kind_val = 0;
      std::string state_str;
      if (iss >> report.run_id >> report.epoch >> kind_val >> report.report_id >>
        report.authorization_revision >> state_str >> std::quoted(report.payload))
      {
        report.kind = static_cast<uint8_t>(kind_val);
        report.pub_state = (state_str == "ATTEMPTED") ?
          PublicationState::ATTEMPTED : PublicationState::PENDING;
        const auto key = std::make_tuple(report.run_id, report.epoch, report.kind);
        accepted_by_key_[key] = report;
        report_id_to_key_[report.report_id] = key;
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
      }
    } else if (tag == "BANYU_GCS_ADAPTER_ENTRY_V1") {
      std::string report_id, payload;
      if (iss >> report_id >> std::quoted(payload)) {
        AcceptedReport report;
        report.report_id = report_id;
        report.payload = payload;
        report.pub_state = PublicationState::ATTEMPTED;
        const auto key = std::make_tuple(0u, 0ul, 0u);
        accepted_by_key_[key] = report;
        report_id_to_key_[report_id] = key;
      }
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
    "Loaded %zu accepted reports (%zu pending publication) from deduplication journal '%s'",
    accepted_by_key_.size(), pending_n, journal_path_.c_str());
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
