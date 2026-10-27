#include "banyu_mission/banyu_mission.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <cstdlib>
#include <filesystem>
#include <random>
#include <sstream>
#include "rx_msgs/msg/rx_task.hpp"
#include "rx_msgs/msg/vehicle_type.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "banyu_mission/action/register_actions.hpp"
#include "banyu_mission/condition/register_conditions.hpp"

namespace banyu_mission
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
// Under sim time this node's clock can lag a fresh source stamp while its executor is busy;
// tolerate that much skew, matching VehicleStatusTracker, without accepting real future stamps.
constexpr double kMaxFutureStampS = 0.5;

bool finite(double value)
{
  return std::isfinite(value);
}

class WaitSurveyComplete : public BT::StatefulActionNode
{
public:
  WaitSurveyComplete(
    const std::string & name, const BT::NodeConfig & config,
    std::function<BT::NodeStatus()> check)
  : StatefulActionNode(name, config), check_(std::move(check)) {}
  static BT::PortsList providedPorts() {return {};}
  BT::NodeStatus onStart() override {return check_();}
  BT::NodeStatus onRunning() override {return check_();}
  void onHalted() override {}

private:
  std::function<BT::NodeStatus()> check_;
};

class MissionTreeLogger : public BT::StatusChangeLogger
{
public:
  MissionTreeLogger(const BT::Tree & tree, rclcpp::Logger logger)
  : BT::StatusChangeLogger(tree.rootNode()), logger_(logger)
  {
  }

  void callback(
    BT::Duration /*timestamp*/, const BT::TreeNode & node,
    BT::NodeStatus prev_status, BT::NodeStatus status) override
  {
    if (prev_status == status) {
      return;
    }

    const auto type = node.type();
    if (type != BT::NodeType::ACTION && type != BT::NodeType::SUBTREE) {
      return;
    }

    const auto & name = node.name();
    const auto & reg_name = node.registrationName();
    const std::string label = (name == reg_name || name.empty()) ?
      reg_name : (name + " (" + reg_name + ")");

    if (status == BT::NodeStatus::RUNNING && prev_status != BT::NodeStatus::RUNNING) {
      RCLCPP_INFO(logger_, "[BT] Executing: %s", label.c_str());
    } else if (status == BT::NodeStatus::SUCCESS && prev_status != BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(logger_, "[BT] Completed: %s -> SUCCESS", label.c_str());
    } else if (status == BT::NodeStatus::FAILURE && prev_status != BT::NodeStatus::FAILURE) {
      RCLCPP_WARN(logger_, "[BT] Completed: %s -> FAILURE", label.c_str());
    }
  }

  void flush() override {}

private:
  rclcpp::Logger logger_;
};
}  // namespace

void registerMissionNodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<MissionState> & state,
  std::function<void()> on_abort,
  std::function<BT::NodeStatus()> on_survey_complete,
  std::function<BT::NodeStatus()> on_surface_success,
  std::function<BT::NodeStatus()> on_lock_buoy_target)
{
  registerConditionNodes(factory, state);
  registerActionNodes(factory, node, state, std::move(on_abort));
  if (!on_survey_complete) {on_survey_complete = []() {return BT::NodeStatus::FAILURE;};}
  if (!on_surface_success) {on_surface_success = []() {return BT::NodeStatus::FAILURE;};}
  factory.registerNodeType<WaitSurveyComplete>("WaitSurveyComplete", on_survey_complete);
  factory.registerSimpleAction("ConfirmSurfaceAction",
    [on_surface_success](BT::TreeNode &) {return on_surface_success();});
  if (!on_lock_buoy_target) {on_lock_buoy_target = []() {return BT::NodeStatus::SUCCESS;};}
  factory.registerSimpleAction("LockBuoyTarget",
    [on_lock_buoy_target](BT::TreeNode &) {return on_lock_buoy_target();});
}

BanyuMission::BanyuMission(const rclcpp::NodeOptions & options)
: rclcpp::Node("banyu_mission", options), state_(std::make_shared<MissionState>(this))
{
  const auto mission_mode = declare_parameter<std::string>("mission_mode", "task2");
  mission_mode_ = mission_mode;
  state_->requireRunContext(mission_mode == "task2");
  state_->configure(
    declare_parameter<double>("timing.gps_timeout_s", 5.0),
    declare_parameter<double>("timing.pipeline_timeout_s", 1.0),
    declare_parameter<double>("timing.state_timeout_s", 5.0),
    declare_parameter<int>("rc_interrupt_channel", 4),
    declare_parameter<int>("rc_interrupt_threshold", 1700),
    declare_parameter<std::string>("autonomy_mode", "GUIDED"),
    declare_parameter<std::string>("autonomy_authority", ""),
    mission_mode != "por");
  declare_parameter<int>("groot_port", 1666);
  fallback_mode_ = declare_parameter<std::string>("fallback_mode", "SURFACE");
  set_mode_service_ = declare_parameter<std::string>("services.set_mode", "/mavros/set_mode");
  set_mode_client_ = create_client<mavros_msgs::srv::SetMode>(set_mode_service_);
  arming_service_ = declare_parameter<std::string>("services.arming", "/mavros/cmd/arming");
  arming_client_ = create_client<mavros_msgs::srv::CommandBool>(arming_service_);

  preparation_timeout_s_ = declare_parameter<double>("timing.preparation_timeout_s", 30.0);
  reset_timeout_s_ = declare_parameter<double>("timing.run_reset_timeout_s", 3.0);
  execution_readiness_timeout_s_ = declare_parameter<double>(
    "timing.execution_readiness_timeout_s", 1.0);
  const auto lifecycle_topic = declare_parameter<std::string>(
    "topics.run_lifecycle", "/banyu/mission/run_lifecycle");
  const auto command_topic = declare_parameter<std::string>(
    "topics.run_command", "/system/mission/command");
  const auto readiness_pose_topic = declare_parameter<std::string>(
    "topics.readiness_pose", "/mavros/local_position/pose");
  const auto execution_readiness_topic = declare_parameter<std::string>(
    "topics.execution_readiness", "/banyu/execution/readiness");
  const auto survey_progress_topic = declare_parameter<std::string>(
    "topics.survey_progress", "/banyu/communication/survey_progress");
  const auto tier = declare_parameter<int>("mission.task2_tier", 2);
  if (tier < 2 || tier > 4) {throw std::invalid_argument("Invalid declared Task 2 tier");}
  task2_tier_ = static_cast<uint8_t>(tier);
  const auto reset_services = declare_parameter<std::vector<std::string>>(
    "run.reset_services", {"/banyu/pipeline/reset_tracker", "/banyu/pipeline/reset_end"});
  if (mission_mode_ == "task2") {
    if (!finite(preparation_timeout_s_) || preparation_timeout_s_ <= 0.0 ||
      !finite(reset_timeout_s_) || reset_timeout_s_ <= 0.0 || reset_services.empty() ||
      !finite(execution_readiness_timeout_s_) || execution_readiness_timeout_s_ <= 0.0)
    {
      throw std::invalid_argument(
          "Task 2 requires reset consumers and positive deadlines");
    }
    run_ = std::make_unique<RunCoordinator>();
    std::random_device random;
    std::ostringstream identity;
    identity << std::hex << random() << random() << random() << random();
    authority_instance_ = identity.str();
    lifecycle_pub_ = create_publisher<banyu_msgs::msg::RunLifecycle>(
      lifecycle_topic, rclcpp::QoS(1).reliable().transient_local());
    command_sub_ = create_subscription<rx_msgs::msg::Command>(
      command_topic, rclcpp::QoS(10).reliable(),
      std::bind(&BanyuMission::commandCallback, this, std::placeholders::_1));
    execution_readiness_sub_ = create_subscription<banyu_msgs::msg::ExecutionReadiness>(
      execution_readiness_topic, rclcpp::QoS(1).reliable(),
      [this](banyu_msgs::msg::ExecutionReadiness::SharedPtr evidence) {
        std::lock_guard<std::recursive_mutex> lock(control_mutex_);
        const auto context = run_->snapshot();
        if (evidence->authority_instance != authority_instance_ ||
        evidence->run_id != context.run_id || evidence->epoch != context.epoch ||
        evidence->lifecycle_revision == 0 || evidence->lifecycle_revision > lifecycle_revision_ ||
        evidence->header.stamp.nanosec >= 1000000000u ||
        (evidence->header.stamp.sec == 0 && evidence->header.stamp.nanosec == 0)) {return;}
        const double age = (now() - rclcpp::Time(evidence->header.stamp,
          get_clock()->get_clock_type())).seconds();
        if (age < -kMaxFutureStampS || age > execution_readiness_timeout_s_ ||
        evidence->lifecycle_revision < execution_readiness_.lifecycle_revision) {return;}
        execution_readiness_ = *evidence;
        execution_received_at_ = std::chrono::steady_clock::now();
        if (evidence->holding && context.phase == RunCoordinator::Phase::Ready) {
          preparation_hold_seen_ = true;
        }
      });
    survey_progress_sub_ = create_subscription<banyu_msgs::msg::Task2SurveyProgress>(
      survey_progress_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](banyu_msgs::msg::Task2SurveyProgress::SharedPtr progress) {
        std::lock_guard<std::recursive_mutex> lock(control_mutex_);
        const auto context = run_->snapshot();
        if (progress->authority_instance != authority_instance_ ||
        progress->run_id != context.run_id || progress->epoch != context.epoch ||
        progress->header.stamp.nanosec >= 1000000000u ||
        (progress->header.stamp.sec == 0 && progress->header.stamp.nanosec == 0)) {return;}
        const double age = (now() - rclcpp::Time(progress->header.stamp,
          get_clock()->get_clock_type())).seconds();
        if (age < -kMaxFutureStampS || age > 2.0) {return;}
        if (progress->evidence_revision < survey_progress_.evidence_revision) {return;}
        survey_progress_ = *progress;
        survey_progress_received_at_ = std::chrono::steady_clock::now();
      });

    buoy_target_sub_ = create_subscription<banyu_msgs::msg::Task2BuoyTarget>(
      declare_parameter<std::string>("topics.buoy_target", "/banyu/task2/buoy_target"),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](banyu_msgs::msg::Task2BuoyTarget::SharedPtr target) {
        std::lock_guard<std::recursive_mutex> lock(control_mutex_);
        if (!finite(target->latitude) || !finite(target->longitude)) {return;}
        buoy_target_ = *target;
        if (!buoy_target_locked_) {applyBuoyTarget();}
      });

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      readiness_pose_topic, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr pose) {
        std::lock_guard<std::recursive_mutex> lock(control_mutex_);
        pose_valid_ = pose->header.frame_id == "map" && finite(pose->pose.position.x) &&
        finite(pose->pose.position.y) && finite(pose->pose.position.z) &&
        (pose->header.stamp.sec != 0 || pose->header.stamp.nanosec != 0);
        const double source_age = (now() - rclcpp::Time(pose->header.stamp,
          get_clock()->get_clock_type())).seconds();
        pose_valid_ = pose_valid_ && source_age >= -kMaxFutureStampS && source_age <= 1.0;
        pose_received_at_ = std::chrono::steady_clock::now();
      });
    for (const auto & service : reset_services) {
      reset_consumers_.push_back({create_client<banyu_msgs::srv::ResetTask2>(service), {}, false});
    }
    preparation_service_ = create_service<std_srvs::srv::Trigger>(
      "/banyu/mission/prepare_autonomy",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::recursive_mutex> lock(control_mutex_);
        const auto vehicle = state_->snapshot();
        response->success = initialized_ && mission_config_valid_ &&
        vehicle.fcu_connected && vehicle.fcu_status_fresh && vehicle.rc_fresh &&
        !vehicle.operator_takeover && vehicle.gps_available && run_->beginPreparation();
        response->message =
        response->success ? "Autonomy preparation accepted; position hold only" :
        "Preparation rejected: configuration, safety, GPS, journal or active run";
        if (response->success) {
          start_commands_.clear();
          last_stop_sequence_ = 0;
          execution_readiness_ = banyu_msgs::msg::ExecutionReadiness{};
          execution_received_at_ = {};
          preparation_hold_seen_ = false;
          state_->prepareAutonomy();
          preparation_tree_->haltTree();
          preparation_started_ = std::chrono::steady_clock::now();
          publishLifecycle();
        }
      });
  }

  start_service_ = create_service<std_srvs::srv::Trigger>(
    "/banyu/mission/start",
    [this](const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::lock_guard<std::recursive_mutex> lock(control_mutex_);
      if (mission_mode_ == "task2") {
        response->success = false;
        response->message = "GCS_START_REQUIRED: use a validated CMD_RUN_START";
        return;
      }
      const bool config_ok = mission_config_valid_.load();
      if (!config_ok) {
        response->success = false;
        response->message = "mission config invalid";
        RCLCPP_INFO(get_logger(), "Mission start service: %s", response->message.c_str());
        return;
      }
      last_fallback_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      const bool started = state_->requestStart();
      response->success = started;
      response->message = response->success ?
      "mission start accepted" :
      "mission cannot be started (already active or succeeded)";
      RCLCPP_INFO(get_logger(), "Mission start service: %s", response->message.c_str());
    });
  abort_service_ = create_service<std_srvs::srv::Trigger>(
    "/banyu/mission/abort",
    [this](const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      std::lock_guard<std::recursive_mutex> lock(control_mutex_);
      if (run_) {
        terminateRun(false, "MISSION_ABORT_REQUESTED", !operatorHasControl());
        response->success = true;
        response->message = "mission abort accepted";
        return;
      }
      state_->requestAbort();
      fallbackToSurface();
      response->success = true;
      response->message = "mission abort accepted";
      RCLCPP_WARN(
        get_logger(), "Mission abort accepted; fallback to %s commanded",
        fallback_mode_.c_str());
    });
}

BanyuMission::~BanyuMission()
{
  stop_requested_.store(true);
  if (tick_thread_.joinable()) {tick_thread_.join();}
  {
    std::lock_guard<std::mutex> lock(tree_mutex_);
    tree_logger_.reset();
    groot_publisher_.reset();
    if (tree_) {tree_->haltTree();}
    if (preparation_tree_) {preparation_tree_->haltTree();}
  }
}

MissionState::Lifecycle BanyuMission::lifecycle() const
{
  return state_->lifecycle();
}

bool BanyuMission::autonomousReady() const
{
  const auto vehicle = state_->snapshot();
  const auto phase = run_ ? run_->snapshot().phase : RunCoordinator::Phase::Idle;
  const bool underway = phase == RunCoordinator::Phase::Active ||
    phase == RunCoordinator::Phase::SurveyComplete || phase == RunCoordinator::Phase::Surfaced;
  return mission_config_valid_ && vehicle.fcu_connected && vehicle.fcu_status_fresh &&
         vehicle.rc_fresh && vehicle.armed && !vehicle.operator_takeover &&
         (underway || vehicle.gps_available) && state_->autonomyModeAllowed(vehicle.mode) && pose_valid_ &&
         std::chrono::steady_clock::now() - pose_received_at_ < std::chrono::seconds(1);
}

std::string BanyuMission::autonomyNotReadyReason() const
{
  const auto vehicle = state_->snapshot();
  const auto phase = run_ ? run_->snapshot().phase : RunCoordinator::Phase::Idle;
  const bool underway = phase == RunCoordinator::Phase::Active ||
    phase == RunCoordinator::Phase::SurveyComplete || phase == RunCoordinator::Phase::Surfaced;
  if (!mission_config_valid_) {return "mission config invalid";}
  if (!vehicle.fcu_connected || !vehicle.fcu_status_fresh) {return "FCU disconnected or stale";}
  if (!vehicle.rc_fresh) {return "RC/deadman stale";}
  if (!vehicle.armed) {return "disarmed";}
  if (vehicle.operator_takeover) {return "operator takeover";}
  if (!underway && !vehicle.gps_available) {return "GPS unavailable";}
  if (!state_->autonomyModeAllowed(vehicle.mode)) {return "mode " + vehicle.mode;}
  if (!pose_valid_) {return "pose invalid or stamp older than 1 s";}
  if (std::chrono::steady_clock::now() - pose_received_at_ >= std::chrono::seconds(1)) {
    return "pose not received for 1 s";
  }
  return "ready";
}

bool BanyuMission::operatorHasControl() const
{
  const auto vehicle = state_->snapshot();
  return vehicle.operator_takeover || (vehicle.fcu_connected && vehicle.fcu_status_fresh &&
         !vehicle.mode.empty() && !state_->autonomyModeAllowed(vehicle.mode));
}

bool BanyuMission::executionHolding() const
{
  const auto context = run_->snapshot();
  return execution_readiness_.holding &&
         execution_readiness_.authority_instance == authority_instance_ &&
         execution_readiness_.run_id == context.run_id &&
         execution_readiness_.epoch == context.epoch &&
         execution_received_at_.time_since_epoch().count() != 0 &&
         std::chrono::duration<double>(std::chrono::steady_clock::now() -
         execution_received_at_).count() <= execution_readiness_timeout_s_;
}

bool BanyuMission::surveyEvidenceReady() const
{
  const auto context = run_->snapshot();
  return survey_progress_.survey_complete &&
         survey_progress_.authority_instance == authority_instance_ &&
         survey_progress_.run_id == context.run_id && survey_progress_.epoch == context.epoch &&
         survey_progress_received_at_.time_since_epoch().count() != 0 &&
         std::chrono::steady_clock::now() - survey_progress_received_at_ < std::chrono::seconds(2);
}

void BanyuMission::publishLifecycle(
  uint8_t event, const std::string & reason, uint32_t requested_run_id)
{
  if (!run_ || !lifecycle_pub_) {return;}
  const auto context = run_->snapshot();
  banyu_msgs::msg::RunLifecycle message;
  message.header.stamp = now();
  message.accepted_at = accepted_run_at_;
  message.header.frame_id = "uuv";
  message.run_id = context.run_id;
  message.epoch = context.epoch;
  message.command_seq = context.command_seq;
  message.phase = static_cast<uint8_t>(context.phase);
  const bool active = context.phase == RunCoordinator::Phase::Active ||
    context.phase == RunCoordinator::Phase::SurveyComplete ||
    context.phase == RunCoordinator::Phase::Surfaced;
  message.current_task = active ? rx_msgs::msg::RxTask::TASK_INFRA_SURVEY_REPAIR :
    rx_msgs::msg::RxTask::TASK_NONE;
  message.reason = reason.empty() ? context.reason : reason;
  message.event = event;
  message.requested_run_id = requested_run_id;
  message.revision = ++lifecycle_revision_;
  message.authority_instance = authority_instance_;
  message.autonomous_ready = autonomousReady() &&
    (active || (context.phase == RunCoordinator::Phase::Ready && executionHolding()) ||
    context.phase == RunCoordinator::Phase::Initializing);
  lifecycle_pub_->publish(message);
  lifecycle_published_at_ = std::chrono::steady_clock::now();
}

void BanyuMission::commandCallback(const rx_msgs::msg::Command::SharedPtr command)
{
  std::lock_guard<std::recursive_mutex> lock(control_mutex_);
  using Command = rx_msgs::msg::Command;
  using Lifecycle = banyu_msgs::msg::RunLifecycle;
  if (!run_ || (command->target_vehicle != rx_msgs::msg::VehicleType::TYPE_UUV &&
    command->target_vehicle != rx_msgs::msg::VehicleType::TYPE_UNKNOWN)) {return;}
  if (command->command_type == Command::CMD_MISSION_DONE) {
    const auto context = run_->snapshot();
    const bool live = context.phase >= RunCoordinator::Phase::Initializing &&
      context.phase <= RunCoordinator::Phase::Surfaced;
    if (!live || command->run_id != context.run_id ||
      command->command_seq <= context.command_seq || command->command_seq <= last_stop_sequence_)
    {
      publishLifecycle(Lifecycle::SNAPSHOT, "STOP_REJECTED: stale sequence or run identity");
      return;
    }
    last_stop_sequence_ = command->command_seq;
    terminateRun(false, "GCS_STOP_REQUESTED", !operatorHasControl());
    return;
  }
  if (command->command_type != Command::CMD_RUN_START) {return;}
  const auto seen = start_commands_.find(command->command_seq);
  if (seen != start_commands_.end() && seen->second != command->run_id) {
    publishLifecycle(Lifecycle::START_REJECTED, "COMMAND_REPLAY: conflicting run identity",
      command->run_id);
    return;
  }
  if (command->command_seq != 0) {start_commands_.emplace(command->command_seq, command->run_id);}
  const auto result = run_->requestStart(
    command->run_id, command->command_seq,
    initialized_ && autonomousReady() && executionHolding());
  if (result.disposition == RunCoordinator::Disposition::Rejected) {
    publishLifecycle(Lifecycle::START_REJECTED, result.reason, command->run_id);
  } else if (result.disposition == RunCoordinator::Disposition::Duplicate) {
    publishLifecycle(Lifecycle::START_DUPLICATE, result.reason, command->run_id);
  } else {
    initialization_started_ = std::chrono::steady_clock::now();
    accepted_run_at_ = now();
    survey_progress_ = banyu_msgs::msg::Task2SurveyProgress{};
    survey_progress_received_at_ = {};
    for (auto & consumer : reset_consumers_) {
      consumer.client->prune_pending_requests();
      consumer.future = {};
      consumer.acknowledged = false;
    }
    last_fallback_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    buoy_target_locked_ = false;
    applyBuoyTarget();
    publishLifecycle(Lifecycle::START_ACCEPTED, result.reason, command->run_id);
  }
}

void BanyuMission::tickTask2Preparation()
{
  const auto phase = run_->snapshot().phase;
  if (phase == RunCoordinator::Phase::Ready &&
    (!autonomousReady() || (preparation_hold_seen_ && !executionHolding())))
  {
    run_->cancelPreparation("READINESS_LOST: explicit preparation required");
    preparation_tree_->haltTree();
    publishLifecycle(banyu_msgs::msg::RunLifecycle::TERMINAL);
    return;
  }
  if (phase != RunCoordinator::Phase::Preparing) {return;}
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
    preparation_started_).count() > preparation_timeout_s_ || state_->snapshot().operator_takeover)
  {
    run_->cancelPreparation("PREPARATION_TIMEOUT_OR_TAKEOVER");
    preparation_tree_->haltTree();
    publishLifecycle(banyu_msgs::msg::RunLifecycle::TERMINAL);
    return;
  }
  const auto result = preparation_tree_->tickExactlyOnce();
  if (result == BT::NodeStatus::FAILURE) {
    run_->cancelPreparation("PREPARATION_FAILED");
    preparation_tree_->haltTree();
    publishLifecycle(banyu_msgs::msg::RunLifecycle::TERMINAL);
  } else if (result == BT::NodeStatus::SUCCESS && autonomousReady()) {
    run_->markReady();
    publishLifecycle();
  }
}

void BanyuMission::tickRunInitialization()
{
  const auto context = run_->snapshot();
  if (context.phase != RunCoordinator::Phase::Initializing) {return;}
  if (!autonomousReady()) {
    terminateRun(true, "READINESS_LOST_DURING_INITIALIZATION", false);
    return;
  }
  if (std::chrono::duration<double>(std::chrono::steady_clock::now() -
    initialization_started_).count() > reset_timeout_s_)
  {
    terminateRun(true, "RUN_RESET_TIMEOUT", false);
    return;
  }
  bool complete = true;
  for (auto & consumer : reset_consumers_) {
    if (consumer.acknowledged) {continue;}
    complete = false;
    if (!consumer.future.valid()) {
      if (consumer.client->service_is_ready()) {
        auto request = std::make_shared<banyu_msgs::srv::ResetTask2::Request>();
        request->run_id = context.run_id;
        request->epoch = context.epoch;
        consumer.future = consumer.client->async_send_request(request).future.share();
      }
    } else if (consumer.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      try {
        const auto response = consumer.future.get();
        if (!response->success) {
          terminateRun(true, "RUN_RESET_REJECTED: " + response->message, false);
          return;
        }
        consumer.acknowledged = true;
      } catch (const std::exception & error) {
        terminateRun(true, "RUN_RESET_FAILED: " + std::string(error.what()), false);
        return;
      }
    }
  }
  if (complete) {
    if (!state_->acceptRun(context.run_id, context.epoch) ||
      !run_->completeInitialization(context.epoch))
    {terminateRun(true, "RUN_INITIALIZATION_FAILED", false); return;}
    preparation_tree_->haltTree();
    tree_->haltTree();
    publishLifecycle();
  }
}

void BanyuMission::terminateRun(bool failure, const std::string & reason, bool recover)
{
  const auto context = run_->snapshot();
  if (context.phase == RunCoordinator::Phase::Preparing ||
    context.phase == RunCoordinator::Phase::Ready)
  {run_->cancelPreparation(reason);} else {
    run_->transition(context.epoch, failure ? RunCoordinator::Phase::Failed :
      RunCoordinator::Phase::Interrupted, reason);
  }
  state_->requestAbort();
  if (failure) {state_->markFailed();} else {state_->markInterrupted();}
  if (tree_) {tree_->haltTree();}
  if (preparation_tree_) {preparation_tree_->haltTree();}
  for (auto & consumer : reset_consumers_) {
    consumer.client->prune_pending_requests();
    consumer.future = {};
  }
  publishLifecycle(banyu_msgs::msg::RunLifecycle::TERMINAL, reason);
  if (recover && !operatorHasControl() &&
    context.phase != RunCoordinator::Phase::Idle)
  {fallbackToSurface();}
}

void BanyuMission::applyBuoyTarget()
{
  if (!blackboard_ || !buoy_target_) {return;}
  const auto & target = *buoy_target_;
  blackboard_->set("target_latitude", target.latitude);
  blackboard_->set("target_longitude", target.longitude);
  const bool use_heading = target.heading_valid && finite(target.heading_enu_rad) &&
    !explicit_heading_;
  if (use_heading) {
    const double heading = std::atan2(
      std::sin(target.heading_enu_rad), std::cos(target.heading_enu_rad));
    blackboard_->set("heading", heading);
    blackboard_->set("heading_prior", heading);
  }
  const auto heading_text = use_heading ?
    std::to_string(target.heading_enu_rad) + " rad" : std::string("unchanged");
  RCLCPP_INFO(
    get_logger(), "Task 2 target from USV PipelineBuoys seq=%u: %.7f, %.7f, heading %s",
    target.source_seq, target.latitude, target.longitude, heading_text.c_str());
}

std::string BanyuMission::xmlPath()
{
  std::string mission_mode = "task2";
  get_parameter("mission_mode", mission_mode);
  std::string por_tree = "por_timed_mission.xml";
  if (por_use_vision_) {
    por_tree = "por_mission.xml";
  } else if (por_use_vision_assist_) {
    por_tree = "por_vision_assisted_mission.xml";
  } else if (por_use_vision_return_) {
    por_tree = "por_hybrid_mission.xml";
  }
  const auto default_tree = ament_index_cpp::get_package_share_directory(
    "banyu_mission") + "/bt/" +
    (mission_mode == "por" ? por_tree : std::string("banyu_mission.xml"));
  return declare_parameter<std::string>(
    "bt_xml", default_tree);
}

void BanyuMission::configurePorBlackboard(const BT::Blackboard::Ptr & blackboard)
{
  try {
    const auto config = PorMissionConfig::load(*this);
    config.validate();
    por_use_vision_ = config.use_vision;
    por_use_vision_return_ = config.use_vision_return;
    por_use_vision_assist_ = config.use_vision_assist;
    config.apply(blackboard);
  } catch (const std::exception & error) {
    mission_config_valid_ = false;
    RCLCPP_ERROR(get_logger(), "Invalid POR mission configuration: %s", error.what());
  }
}

void BanyuMission::initialize()
{
  bool expected = false;
  if (!initialized_.compare_exchange_strong(expected, true)) {return;}

  std::string mission_mode = "task2";
  get_parameter("mission_mode", mission_mode);
  const auto target_latitude = declare_parameter<double>("mission.target_latitude", 0.0);
  const auto target_longitude = declare_parameter<double>("mission.target_longitude", 0.0);
  const auto start_latitude = declare_parameter<double>("mission.start_latitude", 0.0);
  const auto start_longitude = declare_parameter<double>("mission.start_longitude", 0.0);
  const auto end_latitude = declare_parameter<double>(
    "mission.end_latitude", std::numeric_limits<double>::quiet_NaN());
  const auto end_longitude = declare_parameter<double>(
    "mission.end_longitude", std::numeric_limits<double>::quiet_NaN());
  mission_config_valid_ = state_->configureMissionCoordinates(
    start_latitude, start_longitude, target_latitude, target_longitude, mission_mode != "por");
  if (mission_mode == "por") {
    mission_config_valid_ = true;
  }
  const auto positive = [this](double value) {return finite(value) && value > 0.0;};
  const auto nonnegative = [this](double value) {return finite(value) && value >= 0.0;};
  const auto gps_tolerance = declare_parameter<double>("mission.gps_tolerance_m", 3.0);
  const auto heading_tolerance = declare_parameter<double>("mission.heading_tolerance_rad", 0.1);
  const auto inspection_depth = declare_parameter<double>("mission.inspection_depth_m", 2.0);
  const auto depth_tolerance = declare_parameter<double>("mission.depth_tolerance_m", 0.2);
  const auto heading_rad = declare_parameter<double>(
    "mission.heading_rad", std::numeric_limits<double>::quiet_NaN());
  const auto acquire_timeout_s = declare_parameter<double>("mission.acquire_timeout_s", 15.0);
  const auto traverse_timeout_s =
    declare_parameter<double>("mission.traverse_timeout_s", 600.0);
  const auto search_span = declare_parameter<double>("mission.search_span_rad", 2.0 * kPi);
  const auto sweep_rate = declare_parameter<double>("mission.sweep_rate_rad_s", 0.2);
  const auto ring_tolerance = declare_parameter<double>("mission.ring_tolerance", 0.08);
  const auto hold_duration = declare_parameter<double>("mission.hold_duration_s", 3.0);
  // Advanced and Disruptive: stay on the repaired ring while the flash code is read (5 s steady green,
  // then the code for 30 s, plus the probe hold before it).
  const auto decode_window = declare_parameter<double>("mission.decode_window_s", 40.0);
  // Depth for the probe hold (Advanced and Disruptive): the probe tip hangs below the camera and the magnetic
  // switch only reacts at a few centimetres. Defaults to the inspection depth (no extra descent).
  const auto repair_depth_param = declare_parameter<double>("mission.repair_depth_m", 0.0);
  // Looser alignment for the probe step (the tip sits ahead of the camera) and a time limit per repair step.
  const auto repair_ring_tolerance = declare_parameter<double>("mission.repair_ring_tolerance", 0.25);
  const auto repair_step_timeout_s = declare_parameter<double>("mission.repair_step_timeout_s", 60.0);
  // Flight mode for the dive and everything under water; must be the second entry of autonomy_mode.
  const auto underwater_mode = declare_parameter<std::string>("mission.underwater_mode", "ALT_HOLD");
  const auto surface_depth = declare_parameter<double>("mission.surface_depth_m", 0.0);
  const auto surface_tolerance = declare_parameter<double>("mission.surface_tolerance_m", 0.2);
  const auto retries = declare_parameter<int>("mission.max_retries", 3);
  const auto crc = declare_parameter<int64_t>("mission.crc32", 0);
  const auto camera = declare_parameter<std::string>("mission.camera_name", "front_camera");
  const auto id = declare_parameter<std::string>("mission.id", "banyu-mission");
  std::vector<uint8_t> ring_order;
  for (const auto value : declare_parameter<std::vector<int64_t>>(
      "mission.ring_order", std::vector<int64_t>{}))
  {
    if (value < 0 || value > 255) {mission_config_valid_ = false;} else {
      ring_order.push_back(value);
    }
  }
  mission_config_valid_ = mission_config_valid_ && positive(gps_tolerance) &&
    positive(heading_tolerance) && nonnegative(inspection_depth) && positive(depth_tolerance) &&
    positive(acquire_timeout_s) && positive(traverse_timeout_s) &&
    (std::isnan(heading_rad) || finite(heading_rad)) &&
    (std::isnan(end_latitude) || finite(end_latitude)) &&
    (std::isnan(end_longitude) || finite(end_longitude)) &&
    positive(search_span) && search_span <= 2.0 * kPi && positive(sweep_rate) &&
    positive(ring_tolerance) && ring_tolerance <= 1.0 && positive(hold_duration) &&
    nonnegative(surface_depth) && positive(surface_tolerance) && retries >= 0 && retries <= 255 &&
    crc >= 0 && crc <= std::numeric_limits<uint32_t>::max() && !camera.empty() && !id.empty();

  double buoy_heading = 0.0;
  bool buoy_heading_valid = false;
  if (!std::isnan(end_latitude) && !std::isnan(end_longitude)) {
    buoy_heading_valid = MissionState::deriveEnuInitialBearing(
      target_latitude, target_longitude,
      end_latitude, end_longitude,
      buoy_heading);
    if (buoy_heading_valid) {
      buoy_heading = std::atan2(std::sin(buoy_heading), std::cos(buoy_heading));
      RCLCPP_INFO(
        get_logger(),
        "Computed pipeline heading from start/end buoys: %.4f rad (%.1f deg)",
        buoy_heading, buoy_heading * 180.0 / kPi);
    }
  }

  double effective_heading = 0.0;
  if (!std::isnan(heading_rad)) {
    effective_heading = heading_rad;
  } else if (buoy_heading_valid) {
    effective_heading = buoy_heading;
  } else {
    effective_heading = state_->missionHeading();
  }

  auto self = rclcpp::Node::SharedPtr(this, [](rclcpp::Node *) {});
  factory_ = std::make_unique<BT::BehaviorTreeFactory>();
  registerMissionNodes(*factory_, self, state_, [this]() {
      if (!run_ && !operatorHasControl()) {fallbackToSurface();}
    }, [this]() {
      if (!run_ || state_->lifecycle() != MissionState::Lifecycle::Active) {
        return BT::NodeStatus::FAILURE;
      }
      const auto context = run_->snapshot();
      if (context.phase == RunCoordinator::Phase::SurveyComplete) {return BT::NodeStatus::SUCCESS;}
      if (context.phase != RunCoordinator::Phase::Active) {return BT::NodeStatus::FAILURE;}
      if (!state_->pipelineEnded() || !surveyEvidenceReady()) {return BT::NodeStatus::RUNNING;}
      if (!run_->transition(context.epoch, RunCoordinator::Phase::SurveyComplete,
        "ORIGINAL_SURVEY_AND_TIER_EVIDENCE_COMPLETE")) {return BT::NodeStatus::FAILURE;}
      publishLifecycle();
      return BT::NodeStatus::SUCCESS;
    }, [this]() {
      if (!run_ || state_->lifecycle() != MissionState::Lifecycle::Active || !autonomousReady()) {
        return BT::NodeStatus::FAILURE;
      }
      const auto context = run_->snapshot();
      if (!run_->transition(context.epoch, RunCoordinator::Phase::Surfaced,
        "CURRENT_SURFACE_ACTION_SUCCEEDED")) {return BT::NodeStatus::FAILURE;}
      publishLifecycle();
      return BT::NodeStatus::SUCCESS;
    }, [this]() {
      std::lock_guard<std::recursive_mutex> lock(control_mutex_);
      if (!buoy_target_locked_) {
        buoy_target_locked_ = true;
        RCLCPP_INFO(
          get_logger(), "Buoy target locked at %.7f, %.7f (%s)",
          blackboard_->get<double>("target_latitude"), blackboard_->get<double>("target_longitude"),
          buoy_target_ ? "USV PipelineBuoys" : "configured");
      }
      return BT::NodeStatus::SUCCESS;
  });
  auto blackboard = BT::Blackboard::create();
  blackboard_ = blackboard;
  explicit_heading_ = !std::isnan(heading_rad);
  blackboard->set("target_latitude", target_latitude);
  blackboard->set("target_longitude", target_longitude);
  blackboard->set("start_latitude", start_latitude);
  blackboard->set("start_longitude", start_longitude);
  blackboard->set("gps_tolerance", gps_tolerance);
  blackboard->set("heading", effective_heading);
  blackboard->set("heading_tolerance", heading_tolerance);
  blackboard->set("inspection_depth", inspection_depth);
  blackboard->set("depth_tolerance", depth_tolerance);
  blackboard->set("camera_name", camera);
  blackboard->set("heading_prior", effective_heading);
  blackboard->set("acquire_timeout_ms", static_cast<unsigned>(acquire_timeout_s * 1000.0));
  blackboard->set("traverse_timeout_ms", static_cast<unsigned>(traverse_timeout_s * 1000.0));
  blackboard->set("search_span", search_span);
  blackboard->set("sweep_rate", sweep_rate);
  blackboard->set("ring_tolerance", ring_tolerance);
  blackboard->set("hold_duration", hold_duration);
  blackboard->set("underwater_mode", underwater_mode);
  blackboard->set("repair_depth", repair_depth_param > 0.0 ? repair_depth_param : inspection_depth);
  blackboard->set("repair_ring_tolerance", repair_ring_tolerance);
  blackboard->set("repair_step_timeout_ms", static_cast<unsigned>(repair_step_timeout_s * 1000.0));
  blackboard->set("decode_enabled", task2_tier_ >= 3);
  blackboard->set("decode_window", decode_window);
  blackboard->set("hold_total", hold_duration + decode_window);
  blackboard->set("mission_id", id);
  blackboard->set("crc32", static_cast<uint32_t>(crc));
  blackboard->set("max_retries", static_cast<uint8_t>(retries));
  blackboard->set("surface_depth", surface_depth);
  blackboard->set("surface_tolerance", surface_tolerance);
  blackboard->set("ring_order", ring_order);

  if (mission_mode == "por") {
    configurePorBlackboard(blackboard);
  } else {
    std::lock_guard<std::recursive_mutex> lock(control_mutex_);
    applyBuoyTarget();
  }
  factory_->registerBehaviorTreeFromFile(xmlPath());
  tree_ = std::make_shared<BT::Tree>(factory_->createTree("Main", blackboard));
  if (run_) {
    preparation_tree_ = std::make_shared<BT::Tree>(factory_->createTreeFromText(
      "<root BTCPP_format=\"4\" main_tree_to_execute=\"PrepareAutonomy\">"
      "<BehaviorTree ID=\"PrepareAutonomy\"><ArmAndSetGuided /></BehaviorTree></root>"));
    publishLifecycle();
  }
  tree_logger_ = std::make_unique<MissionTreeLogger>(*tree_, get_logger());
  int groot_port = 1666;
  get_parameter("groot_port", groot_port);
  groot_publisher_ =
    std::make_unique<BT::Groot2Publisher>(*tree_, static_cast<unsigned>(std::max(1, groot_port)));
  const int tick_period_ms = declare_parameter<int>("timing.tick_period_ms", 50);
  tick_thread_ = std::thread([this, tick_period_ms]() {tickLoop(std::max(1, tick_period_ms));});
}

void BanyuMission::tickLoop(int tick_period_ms)
{
  bool started_log = false;
  while (rclcpp::ok(get_node_base_interface()->get_context()) && !stop_requested_.load()) {
    std::unique_lock<std::recursive_mutex> control_lock(control_mutex_);
    if (run_) {
      tickTask2Preparation();
      tickRunInitialization();
      const auto phase = run_->snapshot().phase;
      if ((phase == RunCoordinator::Phase::Active ||
        phase == RunCoordinator::Phase::SurveyComplete ||
        phase == RunCoordinator::Phase::Surfaced) && !autonomousReady())
      {
        const auto cause = autonomyNotReadyReason();
        RCLCPP_WARN(get_logger(), "Autonomy readiness lost: %s", cause.c_str());
        terminateRun(false, "AUTONOMY_READINESS_LOST: " + cause, !operatorHasControl());
      }
      if (std::chrono::steady_clock::now() - lifecycle_published_at_ >=
        std::chrono::milliseconds(250)) {publishLifecycle();}
    }
    if (state_->lifecycle() != MissionState::Lifecycle::Active) {
      started_log = false;
      control_lock.unlock();
      std::this_thread::sleep_for(std::chrono::milliseconds(tick_period_ms));
      continue;
    }
    if (!started_log) {
      RCLCPP_INFO(get_logger(), "Mission active: starting BehaviorTree execution");
      started_log = true;
    }
    BT::NodeStatus status;
    {
      std::lock_guard<std::mutex> lock(tree_mutex_);
      if (!tree_) {return;}
      status = tree_->tickExactlyOnce();
    }
    if (status != BT::NodeStatus::RUNNING) {
      const bool interrupted = state_->autonomyInterrupted();
      RCLCPP_INFO(
        get_logger(),
        "BehaviorTree execution finished: status=%s, interrupted=%s",
        BT::toStr(status).c_str(), interrupted ? "true" : "false");
      {
        std::lock_guard<std::mutex> lock(tree_mutex_);
        tree_->haltTree();
      }
      if (interrupted) {
        state_->markInterrupted();
        RCLCPP_WARN(
          get_logger(),
          "Mission aborted/interrupted: fallback mode -> %s",
          fallback_mode_.c_str());
        if (run_) {
          terminateRun(false, "AUTONOMY_INTERRUPTED", !operatorHasControl());
        } else if (!operatorHasControl()) {fallbackToSurface();}
      } else if (status == BT::NodeStatus::SUCCESS) {
        if (run_ && !run_->transition(run_->snapshot().epoch, RunCoordinator::Phase::Succeeded,
          "MISSION_COMPLETED"))
        {terminateRun(true, "MISSION_COMPLETION_TRANSITION_FAILED", false);} else {
          state_->markSucceeded();
          if (run_) {publishLifecycle(banyu_msgs::msg::RunLifecycle::TERMINAL);}
          RCLCPP_INFO(
            get_logger(),
            "Mission completed successfully: disarming vehicle to protect thrusters");
          disarmVehicle();
          setFlightMode("MANUAL");
        }
      } else {
        state_->markFailed();
        RCLCPP_ERROR(
          get_logger(),
          "Mission failed: fallback mode -> %s",
          fallback_mode_.c_str());
        if (run_) {terminateRun(true, "EXECUTION_FAILED", true);} else {fallbackToSurface();}
      }
      started_log = false;
    }
    control_lock.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_period_ms));
  }
}

void BanyuMission::fallbackToSurface()
{
  const auto current_time = now();
  if (last_fallback_time_.nanoseconds() > 0 &&
    (current_time - last_fallback_time_).seconds() < 1.0)
  {
    return;
  }
  last_fallback_time_ = current_time;

  if (fallback_mode_ == "DISARM" || fallback_mode_ == "disarm") {
    RCLCPP_WARN(get_logger(), "Fallback mode is DISARM: disarming vehicle and switching to MANUAL");
    disarmVehicle();
    setFlightMode("MANUAL");
  } else {
    setFlightMode(fallback_mode_);
  }
}

void BanyuMission::disarmVehicle()
{
  if (!arming_client_) {
    return;
  }
  if (!arming_client_->service_is_ready()) {
    if (!arming_client_->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_WARN(
        get_logger(),
        "Arming service '%s' not ready; could not disarm vehicle",
        arming_service_.c_str());
      return;
    }
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  request->value = false;
  RCLCPP_WARN(get_logger(), "Commanding vehicle DISARM via %s", arming_service_.c_str());

  arming_client_->async_send_request(
    request,
    [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->success) {
          RCLCPP_INFO(get_logger(), "Vehicle successfully DISARMED");
        } else {
          RCLCPP_WARN(
            get_logger(), "Failed to disarm vehicle (result: %d)",
            response->result);
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Disarming service call threw an exception: %s", e.what());
      }
    });
}

void BanyuMission::setFlightMode(const std::string & mode)
{
  if (!set_mode_client_) {
    return;
  }
  if (!set_mode_client_->service_is_ready()) {
    RCLCPP_WARN(
      get_logger(),
      "SetMode service '%s' not ready; waiting briefly to set fallback mode %s",
      set_mode_service_.c_str(), mode.c_str());
    if (!set_mode_client_->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_ERROR(
        get_logger(),
        "SetMode service '%s' unavailable; could not switch to fallback mode %s",
        set_mode_service_.c_str(), mode.c_str());
      return;
    }
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->custom_mode = mode;
  RCLCPP_WARN(
    get_logger(),
    "Commanding fallback flight mode '%s' via %s",
    mode.c_str(), set_mode_service_.c_str());

  set_mode_client_->async_send_request(
    request,
    [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
      try {
        auto response = future.get();
        if (response->mode_sent) {
          RCLCPP_INFO(get_logger(), "Flight mode successfully switched to %s", mode.c_str());
        } else {
          RCLCPP_ERROR(
            get_logger(), "Failed to switch flight mode to %s (rejected by FCU)",
            mode.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_logger(), "Exception calling SetMode service for %s: %s", mode.c_str(),
          e.what());
      }
    });
}

}  // namespace banyu_mission
