#include "banyu_mission/banyu_mission.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "banyu_mission/action/register_actions.hpp"
#include "banyu_mission/condition/register_conditions.hpp"

namespace banyu_mission
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

bool finite(double value)
{
  return std::isfinite(value);
}
}  // namespace

void registerMissionNodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<MissionState> & state)
{
  registerConditionNodes(factory, state);
  registerActionNodes(factory, node, state);
}

BanyuMission::BanyuMission()
: rclcpp::Node("banyu_mission"), state_(std::make_shared<MissionState>(this))
{
  state_->configure(
    declare_parameter<double>("timing.gps_timeout_s", 2.0),
    declare_parameter<double>("timing.pipeline_timeout_s", 1.0),
    declare_parameter<double>("timing.state_timeout_s", 2.0),
    declare_parameter<int>("rc_interrupt_channel", 4),
    declare_parameter<int>("rc_interrupt_threshold", 1700),
    declare_parameter<std::string>("autonomy_mode", "GUIDED"),
    declare_parameter<std::string>("autonomy_authority", ""));
  declare_parameter<int>("groot_port", 1666);

  start_service_ = create_service<std_srvs::srv::Trigger>(
    "/banyu/mission/start",
    [this](const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      const bool config_ok = mission_config_valid_.load();
      const bool started = state_->requestStart();
      response->success = config_ok && started;
      response->message = response->success ?
      "mission start accepted" :
      (config_ok ? "mission cannot be started (already active or succeeded)" :
      "mission config invalid");
      RCLCPP_INFO(get_logger(), "Mission start service: %s", response->message.c_str());
    });
  abort_service_ = create_service<std_srvs::srv::Trigger>(
    "/banyu/mission/abort",
    [state = state_](const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      state->requestAbort();
      response->success = true;
      response->message = "mission abort accepted";
    });
}

BanyuMission::~BanyuMission()
{
  stop_requested_.store(true);
  {
    std::lock_guard<std::mutex> lock(tree_mutex_);
    if (tree_) {tree_->haltTree();}
  }
  if (tick_thread_.joinable()) {tick_thread_.join();}
}

std::string BanyuMission::xmlPath()
{
  return declare_parameter<std::string>(
    "bt_xml", ament_index_cpp::get_package_share_directory(
      "banyu_mission") + "/bt/banyu_mission.xml");
}

void BanyuMission::initialize()
{
  bool expected = false;
  if (!initialized_.compare_exchange_strong(expected, true)) {return;}

  const auto target_latitude = declare_parameter<double>("mission.target_latitude", 0.0);
  const auto target_longitude = declare_parameter<double>("mission.target_longitude", 0.0);
  const auto start_latitude = declare_parameter<double>("mission.start_latitude", 0.0);
  const auto start_longitude = declare_parameter<double>("mission.start_longitude", 0.0);
  mission_config_valid_ = state_->configureMissionCoordinates(
    start_latitude, start_longitude, target_latitude, target_longitude);
  const auto positive = [this](double value) {return finite(value) && value > 0.0;};
  const auto nonnegative = [this](double value) {return finite(value) && value >= 0.0;};
  const auto gps_tolerance = declare_parameter<double>("mission.gps_tolerance_m", 3.0);
  const auto heading_tolerance = declare_parameter<double>("mission.heading_tolerance_rad", 0.1);
  const auto inspection_depth = declare_parameter<double>("mission.inspection_depth_m", 2.0);
  const auto depth_tolerance = declare_parameter<double>("mission.depth_tolerance_m", 0.2);
  const auto search_span = declare_parameter<double>("mission.search_span_rad", 2.0 * kPi);
  const auto sweep_rate = declare_parameter<double>("mission.sweep_rate_rad_s", 0.2);
  const auto ring_tolerance = declare_parameter<double>("mission.ring_tolerance", 0.08);
  const auto hold_duration = declare_parameter<double>("mission.hold_duration_s", 3.0);
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
    positive(search_span) && search_span <= 2.0 * kPi && positive(sweep_rate) &&
    positive(ring_tolerance) && ring_tolerance <= 1.0 && positive(hold_duration) &&
    nonnegative(surface_depth) && positive(surface_tolerance) && retries >= 0 && retries <= 255 &&
    crc >= 0 && crc <= std::numeric_limits<uint32_t>::max() && !camera.empty() && !id.empty();

  auto self = shared_from_this();
  factory_ = std::make_unique<BT::BehaviorTreeFactory>();
  registerMissionNodes(*factory_, self, state_);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("target_latitude", target_latitude);
  blackboard->set("target_longitude", target_longitude);
  blackboard->set("start_latitude", start_latitude);
  blackboard->set("start_longitude", start_longitude);
  blackboard->set("gps_tolerance", gps_tolerance);
  blackboard->set("heading", state_->missionHeading());
  blackboard->set("heading_tolerance", heading_tolerance);
  blackboard->set("inspection_depth", inspection_depth);
  blackboard->set("depth_tolerance", depth_tolerance);
  blackboard->set("camera_name", camera);
  blackboard->set("heading_prior", state_->missionHeading());
  blackboard->set("search_span", search_span);
  blackboard->set("sweep_rate", sweep_rate);
  blackboard->set("ring_tolerance", ring_tolerance);
  blackboard->set("hold_duration", hold_duration);
  blackboard->set("mission_id", id);
  blackboard->set("crc32", static_cast<uint32_t>(crc));
  blackboard->set("max_retries", static_cast<uint8_t>(retries));
  blackboard->set("surface_depth", surface_depth);
  blackboard->set("surface_tolerance", surface_tolerance);
  blackboard->set("ring_order", ring_order);
  factory_->registerBehaviorTreeFromFile(xmlPath());
  tree_ = std::make_shared<BT::Tree>(factory_->createTree("Main", blackboard));
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
  while (rclcpp::ok() && !stop_requested_.load()) {
    if (state_->lifecycle() != MissionState::Lifecycle::Active) {
      started_log = false;
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
      if (interrupted) {state_->markInterrupted();} else if (status == BT::NodeStatus::SUCCESS) {
        state_->markSucceeded();
      } else {state_->markFailed();}
      started_log = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_period_ms));
  }
}

}  // namespace banyu_mission
