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
  std::function<void()> on_abort)
{
  registerConditionNodes(factory, state);
  registerActionNodes(factory, node, state, std::move(on_abort));
}

BanyuMission::BanyuMission(const rclcpp::NodeOptions & options)
: rclcpp::Node("banyu_mission", options), state_(std::make_shared<MissionState>(this))
{
  const auto mission_mode = declare_parameter<std::string>("mission_mode", "task2");
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

  start_service_ = create_service<std_srvs::srv::Trigger>(
    "/banyu/mission/start",
    [this](const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
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
  }
}

MissionState::Lifecycle BanyuMission::lifecycle() const
{
  return state_->lifecycle();
}

std::string BanyuMission::xmlPath()
{
  std::string mission_mode = "task2";
  get_parameter("mission_mode", mission_mode);
  const auto default_tree = ament_index_cpp::get_package_share_directory(
    "banyu_mission") + "/bt/" +
    (mission_mode == "por" ? "por_mission.xml" : "banyu_mission.xml");
  return declare_parameter<std::string>(
    "bt_xml", default_tree);
}

void BanyuMission::configurePorBlackboard(const BT::Blackboard::Ptr & blackboard)
{
  try {
    const auto config = PorMissionConfig::load(*this);
    config.validate();
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
    positive(acquire_timeout_s) && (std::isnan(heading_rad) || finite(heading_rad)) &&
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

  auto self = shared_from_this();
  factory_ = std::make_unique<BT::BehaviorTreeFactory>();
  registerMissionNodes(*factory_, self, state_, [this]() {fallbackToSurface();});
  auto blackboard = BT::Blackboard::create();
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

  if (mission_mode == "por") {
    configurePorBlackboard(blackboard);
  }
  factory_->registerBehaviorTreeFromFile(xmlPath());
  tree_ = std::make_shared<BT::Tree>(factory_->createTree("Main", blackboard));
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
      if (interrupted) {
        state_->markInterrupted();
        RCLCPP_WARN(
          get_logger(),
          "Mission aborted/interrupted: fallback mode -> %s",
          fallback_mode_.c_str());
        fallbackToSurface();
      } else if (status == BT::NodeStatus::SUCCESS) {
        state_->markSucceeded();
      } else {
        state_->markFailed();
        RCLCPP_ERROR(
          get_logger(),
          "Mission failed: fallback mode -> %s",
          fallback_mode_.c_str());
        fallbackToSurface();
      }
      started_log = false;
    }
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
  setFlightMode(fallback_mode_);
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
