#include "banyu_mission/banyu_mission.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "banyu_mission/action_nodes.hpp"

namespace banyu_mission
{

namespace
{
constexpr uint8_t kRed = banyu_msgs::msg::PipelineRingObservation::COLOR_RED;
constexpr double kPi = 3.14159265358979323846;

bool finite(double value)
{
  return std::isfinite(value);
}

void registerConditions(
  BT::BehaviorTreeFactory & factory,
  const std::shared_ptr<MissionState> & state)
{
  factory.registerSimpleCondition(
    "AutonomyInterrupted", [state](BT::TreeNode &) {
      return state->autonomyInterrupted() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleCondition(
    "RequireVehicleReady", [state](BT::TreeNode &) {
      return state->vehicleReady() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleCondition(
    "RequireGpsAvailable", [state](BT::TreeNode &) {
      return state->gpsAvailable() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleCondition(
    "RequireMissionStarted", [state](BT::TreeNode &) {
      return state->snapshot().mission_started ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleCondition(
    "PipelineLost", [state](BT::TreeNode &) {
      return state->pipelineLost() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleCondition(
    "PipelineActive", [state](BT::TreeNode &) {
      return state->pipelineEnded() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    });
  factory.registerSimpleCondition(
    "PipelineEnded", [state](BT::TreeNode &) {
      return state->pipelineEnded() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleCondition(
    "RedRingPending", [state](BT::TreeNode &) {
      return state->redPending() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    });
  factory.registerSimpleAction(
    "HandleAbortOrTeleop", [state](BT::TreeNode &) {
      state->requestAbort();
      return BT::NodeStatus::SUCCESS;
    });
}

}  // namespace

MissionState::MissionState(rclcpp::Node * node, bool subscribe)
: node_(node), last_pipeline_(node->now()), last_state_(node->now())
{
  if (!subscribe) {
    return;
  }
  const auto vehicle_status_topic = node_->declare_parameter<std::string>(
    "topics.vehicle_status", "/banyu/vehicle_status");
  const auto tracker_state_topic = node_->declare_parameter<std::string>(
    "topics.tracker_state", "/banyu/pipeline/ring_state");
  const bool tracker_state_transient_local = node_->declare_parameter<bool>(
    "topics.tracker_state_transient_local", false);
  const auto pipeline_topic = node_->declare_parameter<std::string>(
    "topics.pipeline_observation", "/banyu/pipeline/observation");
  const auto ring_topic = node_->declare_parameter<std::string>(
    "topics.ring_observation", "/banyu/pipeline/ring_observation");
  const auto end_topic = node_->declare_parameter<std::string>(
    "topics.pipeline_end", "/banyu/pipeline/end");

  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::VehicleStatus>(
      vehicle_status_topic, rclcpp::QoS(5),
      std::bind(&MissionState::stateCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineObservation>(
      pipeline_topic, rclcpp::QoS(5),
      std::bind(&MissionState::pipelineCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineRingObservation>(
      ring_topic, rclcpp::QoS(5),
      std::bind(&MissionState::ringCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<banyu_msgs::msg::PipelineRingState>(
      tracker_state_topic,
      tracker_state_transient_local ? rclcpp::QoS(5).reliable().transient_local() :
      rclcpp::QoS(5).reliable(),
      std::bind(&MissionState::ringStateCallback, this, std::placeholders::_1)));
  subscriptions_.push_back(
    node_->create_subscription<std_msgs::msg::Bool>(
      end_topic, rclcpp::QoS(5),
      std::bind(&MissionState::endCallback, this, std::placeholders::_1)));
}

void MissionState::configure(
  double gps_timeout_s, double pipeline_timeout_s, double state_timeout_s,
  int rc_interrupt_channel, int rc_interrupt_threshold,
  std::string autonomy_mode, std::string autonomy_authority)
{
  std::lock_guard<std::mutex> lock(mutex_);
  gps_timeout_s_ = finite(gps_timeout_s) && gps_timeout_s > 0.0 ? gps_timeout_s : 0.01;
  pipeline_timeout_s_ = finite(pipeline_timeout_s) &&
    pipeline_timeout_s > 0.0 ? pipeline_timeout_s : 0.01;
  state_timeout_s_ = finite(state_timeout_s) && state_timeout_s > 0.0 ? state_timeout_s : 0.01;
  rc_interrupt_channel_ = std::max(0, rc_interrupt_channel);
  rc_interrupt_threshold_ = rc_interrupt_threshold;
  autonomy_mode_ = std::move(autonomy_mode);
  autonomy_authority_ = std::move(autonomy_authority);
}

bool MissionState::validateMissionCoordinates(
  double start_latitude, double start_longitude,
  double end_latitude, double end_longitude)
{
  return finite(start_latitude) && finite(start_longitude) && finite(end_latitude) &&
         finite(end_longitude) && start_latitude >= -90.0 && start_latitude <= 90.0 &&
         end_latitude >= -90.0 && end_latitude <= 90.0 && start_longitude >= -180.0 &&
         start_longitude <= 180.0 && end_longitude >= -180.0 && end_longitude <= 180.0 &&
         (start_latitude != end_latitude || start_longitude != end_longitude);
}

bool MissionState::deriveEnuInitialBearing(
  double start_latitude, double start_longitude,
  double end_latitude, double end_longitude, double & bearing)
{
  if (!validateMissionCoordinates(start_latitude, start_longitude, end_latitude, end_longitude)) {
    return false;
  }
  const double lat0 = start_latitude * kPi / 180.0;
  const double lat1 = end_latitude * kPi / 180.0;
  double dlon = (end_longitude - start_longitude) * kPi / 180.0;
  while (dlon > kPi) {dlon -= 2.0 * kPi;}
  while (dlon < -kPi) {dlon += 2.0 * kPi;}
  const double east = dlon * std::cos((lat0 + lat1) * 0.5);
  const double north = (lat1 - lat0);
  bearing = std::atan2(north, east);
  if (bearing < 0.0) {bearing += 2.0 * kPi;}
  return finite(bearing);
}

bool MissionState::configureMissionCoordinates(
  double start_latitude, double start_longitude,
  double end_latitude, double end_longitude)
{
  std::lock_guard<std::mutex> lock(mutex_);
  start_latitude_ = start_latitude;
  start_longitude_ = start_longitude;
  end_latitude_ = end_latitude;
  end_longitude_ = end_longitude;
  mission_coordinates_valid_ = deriveEnuInitialBearing(
    start_latitude, start_longitude, end_latitude, end_longitude, mission_heading_);
  return mission_coordinates_valid_;
}

bool MissionState::resetForStart()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ != Lifecycle::Idle || !mission_coordinates_valid_) {
    return false;
  }
  mission_coordinates_valid_ = deriveEnuInitialBearing(
    start_latitude_, start_longitude_, end_latitude_, end_longitude_, mission_heading_);
  if (!mission_coordinates_valid_) {
    return false;
  }
  mission_started_ = true;
  lifecycle_ = Lifecycle::Active;
  abort_requested_ = false;
  pipeline_ended_ = false;
  pipeline_seen_ = false;
  red_pending_ = false;
  ring_visible_ = false;
  ring_color_ = banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
  return true;
}

void MissionState::requestAbort()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {
    abort_requested_ = true;
  }
}

bool MissionState::requestStart()
{
  return resetForStart();
}

void MissionState::markSucceeded()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {lifecycle_ = Lifecycle::Succeeded;}
}

void MissionState::markFailed()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {lifecycle_ = Lifecycle::Failed;}
}

void MissionState::markInterrupted()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (lifecycle_ == Lifecycle::Active) {lifecycle_ = Lifecycle::Interrupted;}
}

MissionState::Lifecycle MissionState::lifecycle() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return lifecycle_;
}

double MissionState::missionHeading() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mission_heading_;
}

MissionState::Snapshot MissionState::snapshot() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto now = node_->now();
  const bool state_fresh = fcu_status_fresh_ && (now - last_state_).seconds() <= state_timeout_s_;
  const bool pipeline_fresh = pipeline_seen_ &&
    (now - last_pipeline_).seconds() <= pipeline_timeout_s_;
  Snapshot value;
  value.fcu_connected = fcu_connected_;
  value.fcu_status_fresh = state_fresh;
  value.rc_fresh = rc_fresh_;
  value.operator_takeover = operator_takeover_;
  value.manual_interrupt = manual_interrupt_;
  value.abort_requested = abort_requested_;
  value.mission_started = mission_started_;
  value.gps_available = gps_valid_;
  value.pipeline_fresh = pipeline_fresh && pipeline_tracked_;
  value.pipeline_lost = !value.pipeline_fresh;
  value.pipeline_ended = pipeline_ended_;
  value.red_pending = red_pending_;
  value.red_record_index = red_record_index_;
  value.ring_visible = ring_visible_;
  value.ring_color = ring_color_;
  value.mode = mode_;
  value.authority = authority_;
  return value;
}

bool MissionState::autonomyInterrupted() const
{
  const auto value = snapshot();
  if (!value.mission_started || lifecycle() != Lifecycle::Active) {return false;}
  const bool ready = value.fcu_connected && value.fcu_status_fresh && value.rc_fresh &&
    !value.operator_takeover && value.mode == autonomy_mode_ &&
    (autonomy_authority_.empty() || value.authority == autonomy_authority_);
  return value.abort_requested || !ready;
}

bool MissionState::vehicleReady() const
{
  const auto value = snapshot();
  return value.fcu_connected && value.fcu_status_fresh && value.rc_fresh &&
         !value.operator_takeover && value.mode == autonomy_mode_ &&
         (autonomy_authority_.empty() || value.authority == autonomy_authority_);
}

bool MissionState::gpsAvailable() const {return snapshot().gps_available;}
bool MissionState::pipelineLost() const {return snapshot().pipeline_lost;}
bool MissionState::pipelineEnded() const {return snapshot().pipeline_ended;}
bool MissionState::redPending() const {return snapshot().red_pending;}
uint32_t MissionState::pendingRedIndex() const {return snapshot().red_record_index;}

bool MissionState::acknowledgeRed(uint32_t index)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!red_pending_ || red_record_index_ != index) {return false;}
  red_pending_ = false;
  return true;
}

void MissionState::stateCallback(const banyu_msgs::msg::VehicleStatus::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  fcu_connected_ = msg->fcu_connected;
  fcu_status_fresh_ = msg->fcu_status_fresh;
  rc_fresh_ = msg->rc_fresh;
  operator_takeover_ = msg->operator_takeover;
  mode_ = msg->mode;
  authority_ = msg->authority;
  gps_valid_ = msg->gps_valid && msg->gps_fresh;
  manual_interrupt_ = msg->operator_takeover;
  last_state_ = node_->now();
}

void MissionState::pipelineCallback(const banyu_msgs::msg::PipelineObservation::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_seen_ = true;
  pipeline_tracked_ = msg->status == banyu_msgs::msg::PipelineObservation::TRACKED &&
    finite(msg->offset_x) && finite(msg->offset_y) && finite(msg->heading_error_rad) &&
    finite(msg->confidence) && msg->confidence >= 0.0F;
  last_pipeline_ = node_->now();
}

void MissionState::ringCallback(const banyu_msgs::msg::PipelineRingObservation::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ring_visible_ = msg->visible;
  ring_color_ = msg->visible ? msg->color : banyu_msgs::msg::PipelineRingObservation::COLOR_UNKNOWN;
}

void MissionState::ringStateCallback(const banyu_msgs::msg::PipelineRingState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  ring_color_ = msg->color;
  if (msg->color == banyu_msgs::msg::PipelineRingState::COLOR_RED && finite(msg->confidence) &&
    msg->confidence > 0.0F)
  {
    red_pending_ = true;
    red_record_index_ = msg->index;
  }
}

void MissionState::endCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  pipeline_ended_ = msg->data;
}

void registerMissionNodes(
  BT::BehaviorTreeFactory & factory,
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<MissionState> & state)
{
  registerConditions(factory, state);
  const auto ack_service = node->declare_parameter<std::string>(
    "services.acknowledge_ring_repair", "/banyu/pipeline/acknowledge_ring_repair");
  factory.registerNodeType<AcknowledgeRedRingNode>(
    "AcknowledgeRedRing", node, state, ack_service);
  const auto action_name = [node](const std::string & suffix) {
      return node->declare_parameter<std::string>("actions." + suffix, "/banyu/actions/" + suffix);
    };
  factory.registerNodeType<NavigateSurfaceGpsNode>(
    "NavigateSurfaceGps", node, action_name("navigate_surface_gps"));
  factory.registerNodeType<AlignHeadingNode>("AlignHeading", node, action_name("align_heading"));
  factory.registerNodeType<DiveToDepthNode>("DiveToDepth", node, action_name("dive_to_depth"));
  factory.registerNodeType<AcquirePipelineNode>(
    "AcquirePipeline", node, action_name("acquire_pipeline"));
  factory.registerNodeType<SearchPipelineNode>(
    "SearchPipeline", node, action_name("search_pipeline"));
  factory.registerNodeType<FollowPipelineNode>(
    "FollowPipeline", node, action_name("follow_pipeline"));
  factory.registerNodeType<CenterOnRingNode>("CenterOnRing", node, action_name("center_on_ring"));
  factory.registerNodeType<HoldProbeOnRingNode>(
    "HoldProbeOnRing", node, action_name("hold_probe_on_ring"));
  factory.registerNodeType<SurfaceNode>("Surface", node, action_name("surface"));
  factory.registerNodeType<SendMissionReportNode>(
    "SendMissionReport", node, action_name("send_mission_report"));
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
      response->success = mission_config_valid_ && state_->requestStart();
      response->message = response->success ? "mission start accepted" : "mission cannot be started";
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
  while (rclcpp::ok() && !stop_requested_.load()) {
    if (state_->lifecycle() != MissionState::Lifecycle::Active) {
      std::this_thread::sleep_for(std::chrono::milliseconds(tick_period_ms));
      continue;
    }
    BT::NodeStatus status;
    {
      std::lock_guard<std::mutex> lock(tree_mutex_);
      if (!tree_) {return;}
      status = tree_->tickExactlyOnce();
    }
    if (status != BT::NodeStatus::RUNNING) {
      const bool interrupted = state_->autonomyInterrupted();
      {
        std::lock_guard<std::mutex> lock(tree_mutex_);
        tree_->haltTree();
      }
      if (interrupted) {state_->markInterrupted();} else if (status == BT::NodeStatus::SUCCESS) {
        state_->markSucceeded();
      } else {state_->markFailed();}
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(tick_period_ms));
  }
}

}  // namespace banyu_mission
