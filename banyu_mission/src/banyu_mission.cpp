#include "banyu_mission/banyu_mission.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <filesystem>

#include "banyu_mission/bt_nodes.hpp"

namespace banyu_mission {

BanyuMission::BanyuMission(const rclcpp::NodeOptions& options)
    : Node("banyu_mission", options), ctx_(std::make_shared<MissionContext>()) {
    declareParameters();
    ctx_->setpoint_pub = setpoint_pub_;
    ctx_->robocommand_pub = robocommand_pub_;
}

void BanyuMission::initialize() {
    setupBehaviorTree();
    RCLCPP_INFO(this->get_logger(), "BanyuMission node has been initialized.");
}

BanyuMission::~BanyuMission() {
    shutting_down_.store(true);
    ctx_->aborted.store(true);
    ctx_->mission_active.store(false);
    handoff_active_.store(false);
    {
        std::lock_guard<std::mutex> lock(handoff_mutex_);
        handoff_request_in_flight_.store(false);
        ++handoff_request_generation_;
    }
    if (handoff_timer_) {
        handoff_timer_->cancel();
    }
    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }
    if (tree_.rootNode() != nullptr) {
        tree_.haltTree();
    }
}

void BanyuMission::declareParameters() {
    // Offsets are NED: east/north/down.
    this->declare_parameter<double>("gate_offset_north", 0.0);
    this->declare_parameter<double>("gate_offset_east", 0.0);
    this->declare_parameter<double>("gate_offset_down", 1.9);
    this->declare_parameter<double>("arrival_tolerance", 0.5);
    this->declare_parameter<double>("nav_timeout", 60.0);
    this->declare_parameter<double>("nav_kp_h", 0.5);
    this->declare_parameter<double>("nav_kp_v", 0.5);
    this->declare_parameter<double>("nav_max_speed_h", 0.5);
    this->declare_parameter<double>("nav_max_speed_v", 0.3);
    this->declare_parameter<double>("return_timeout", 60.0);
    detection_timeout_s_ = this->declare_parameter<double>("detection_timeout_s", 0.5);
    hydrophone_heading_timeout_s_ =
        this->declare_parameter<double>("hydrophone_heading_timeout_s", 0.5);
    pipeline_ring_state_timeout_s_ =
        this->declare_parameter<double>("pipeline_ring_state_timeout_s", 0.5);
    active_buoy_pose_timeout_s_ =
        this->declare_parameter<double>("active_buoy_pose_timeout_s", 0.5);
    pipeline_route_timeout_s_ =
        this->declare_parameter<double>("pipeline_route_timeout_s", 0.5);

    // GateAlign
    this->declare_parameter<double>("gate_align_timeout", 60.0);
    this->declare_parameter<double>("gate_align_deadband_x", 0.08);
    this->declare_parameter<double>("gate_align_deadband_y", 0.08);
    this->declare_parameter<double>("gate_align_gain_x", 0.6);
    this->declare_parameter<double>("gate_align_gain_y", 0.4);
    this->declare_parameter<double>("gate_align_max_speed_y", 0.3);
    this->declare_parameter<double>("gate_align_max_speed_z", 0.2);
    this->declare_parameter<double>("gate_align_hold_time", 0.5);

    // GatePass (forward) and ReturnGatePass (return leg)
    this->declare_parameter<double>("gate_pass_speed", 0.4);
    this->declare_parameter<double>("gate_pass_distance", 3.0);
    this->declare_parameter<double>("gate_pass_min_distance", 1.5);
    this->declare_parameter<double>("gate_pass_timeout", 60.0);

    // SearchMarker / SearchGate
    this->declare_parameter<double>("search_speed", 0.4);
    this->declare_parameter<double>("search_approach_distance", 2.5);
    this->declare_parameter<double>("search_min_distance", 1.8);
    this->declare_parameter<double>("search_timeout", 60.0);

    // MarkerAlign
    this->declare_parameter<double>("marker_align_timeout", 60.0);
    this->declare_parameter<double>("marker_align_deadband_x", 0.08);
    this->declare_parameter<double>("marker_align_deadband_y", 0.08);
    this->declare_parameter<double>("marker_align_gain_x", 0.6);
    this->declare_parameter<double>("marker_align_gain_y", 0.4);
    this->declare_parameter<double>("marker_align_max_speed_y", 0.3);
    this->declare_parameter<double>("marker_align_max_speed_z", 0.2);
    this->declare_parameter<double>("marker_align_hold_time", 0.5);

    // MarkerOrbit (strafe orbit: body-y circles, body-x locks range, yaw rate
    // keeps the buoy centered; completes after a full revolution by yaw)
    this->declare_parameter<double>("orbit_speed", 0.3);
    this->declare_parameter<double>("orbit_radius_m", 3.0);
    this->declare_parameter<double>("orbit_kp_range", 0.5);
    this->declare_parameter<double>("orbit_max_range_speed", 0.3);
    this->declare_parameter<double>("orbit_kp_yaw", 0.6);
    this->declare_parameter<double>("orbit_max_yaw_rate", 0.6);
    this->declare_parameter<double>("orbit_kp_vertical", 0.4);
    this->declare_parameter<double>("orbit_max_vertical", 0.2);
    this->declare_parameter<double>("orbit_angle_rad", 6.28318530718);
    this->declare_parameter<double>("orbit_timeout", 120.0);

    // ReturnToStart leg: turn, search gate, align, pass through, surface
    this->declare_parameter<double>("turn_yaw_rate", 0.3);
    this->declare_parameter<double>("turn_capture_gain", 0.5);
    this->declare_parameter<double>("turn_capture_deadband", 0.2);
    this->declare_parameter<double>("turn_angle_rad", 3.14159265359);
    this->declare_parameter<double>("turn_timeout", 30.0);
    this->declare_parameter<double>("gate_pass_lost_distance", 4.0);
    this->declare_parameter<double>("gate_pass_clearance_m", 0.6);
    this->declare_parameter<double>("surface_depth", -0.4);
    this->declare_parameter<double>("surface_tolerance", 0.2);
    this->declare_parameter<double>("surface_timeout", 30.0);

    this->declare_parameter<std::string>("abort_mode", "STABILIZE");
    this->declare_parameter<int>("groot2_port", 1666);
    mission_mode_ = this->declare_parameter<std::string>("mission_mode", "por");
    handoff_request_timeout_s_ =
        this->declare_parameter<double>("handoff_request_timeout_s", 1.0);
    this->declare_parameter<bool>("task2_fixture_mode", false);
    this->declare_parameter<double>("task2_preflight_timeout_s", 30.0);
    pose_state_timeout_s_ = this->declare_parameter<double>("task2_pose_state_timeout_s", 0.5);
    task2_dvl_timeout_s_ = this->declare_parameter<double>("task2_dvl_timeout_s", 0.5);
    this->declare_parameter<double>("task2_ring_confidence_min", 0.5);
    this->declare_parameter<double>("task2_start_capture_distance", 1.0);
    this->declare_parameter<double>("task2_waypoint_horizontal_tolerance", 0.4);
    this->declare_parameter<double>("task2_waypoint_vertical_tolerance", 0.3);
    this->declare_parameter<double>("task2_waypoint_hold_s", 0.5);
    this->declare_parameter<double>("task2_max_waypoint_spacing", 5.0);
    this->declare_parameter<double>("task2_route_min_z", -16.0);
    this->declare_parameter<double>("task2_route_max_z", -11.0);
    this->declare_parameter<double>("task2_min_segment_spacing", 0.3);
    this->declare_parameter<double>("task2_min_path_length", 4.0);
    this->declare_parameter<double>("task2_max_path_length", 8.0);
    this->declare_parameter<double>("task2_segment_timeout_s", 60.0);
    this->declare_parameter<double>("task2_total_timeout_s", 360.0);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "mavros/local_position/pose", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onPose, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, std::bind(&BanyuMission::onState, this, std::placeholders::_1));
    detection_sub_ = this->create_subscription<banyu_msgs::msg::Detection>(
        "banyu/detection", rclcpp::SensorDataQoS(), std::bind(&BanyuMission::onDetection, this, std::placeholders::_1));
    hydrophone_heading_sub_ = this->create_subscription<banyu_msgs::msg::HydrophoneHeading>(
        "/banyu/hydrophone/relative_heading", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onHydrophoneHeading, this, std::placeholders::_1));
    pipeline_ring_state_sub_ = this->create_subscription<banyu_msgs::msg::PipelineRingState>(
        "/banyu/pipeline/ring_state", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onPipelineRingState, this, std::placeholders::_1));
    active_buoy_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/banyu/active_buoy_pose", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onActiveBuoyPose, this, std::placeholders::_1));
    pipeline_route_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/banyu/pipeline/route", rclcpp::SensorDataQoS(),
        std::bind(&BanyuMission::onPipelineRoute, this, std::placeholders::_1));
    dvl_live_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/banyu/dvl/live", 10, std::bind(&BanyuMission::onDvlLive, this, std::placeholders::_1));

    detection_watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
        std::lock_guard<std::mutex> lock(ctx_->data_mutex);
        const rclcpp::Time now = this->now();
        if (ctx_->has_detection && ctx_->last_detection_received.nanoseconds() != 0 &&
            (now - ctx_->last_detection_received).seconds() > detection_timeout_s_) {
            ctx_->has_detection = false;
        }
        if (ctx_->has_pose && ctx_->last_pose_received.nanoseconds() != 0 &&
            (now - ctx_->last_pose_received).seconds() > pose_state_timeout_s_) {
            ctx_->has_pose = false;
        }
        if (ctx_->has_state && ctx_->last_state_received.nanoseconds() != 0 &&
            (now - ctx_->last_state_received).seconds() > pose_state_timeout_s_) {
            ctx_->has_state = false;
        }
        if (ctx_->has_hydrophone_heading &&
            ctx_->last_hydrophone_heading_received.nanoseconds() != 0 &&
            (now - ctx_->last_hydrophone_heading_received).seconds() >
                hydrophone_heading_timeout_s_) {
            ctx_->has_hydrophone_heading = false;
        }
        if (!ctx_->pipeline_ring_states.empty()) {
            for (auto ring_it = ctx_->pipeline_ring_states.begin();
                 ring_it != ctx_->pipeline_ring_states.end();) {
                const auto receipt_it = ctx_->pipeline_ring_state_receipt_times.find(ring_it->first);
                if (receipt_it == ctx_->pipeline_ring_state_receipt_times.end() ||
                    receipt_it->second.nanoseconds() == 0 ||
                    (now - receipt_it->second).seconds() > pipeline_ring_state_timeout_s_) {
                    ctx_->pipeline_ring_state_receipt_times.erase(ring_it->first);
                    ring_it = ctx_->pipeline_ring_states.erase(ring_it);
                } else {
                    ++ring_it;
                }
            }
        }
        ctx_->has_pipeline_ring_state = !ctx_->pipeline_ring_states.empty();
        if (ctx_->has_active_buoy_pose && ctx_->last_active_buoy_pose_received.nanoseconds() != 0 &&
            (now - ctx_->last_active_buoy_pose_received).seconds() > active_buoy_pose_timeout_s_) {
            ctx_->has_active_buoy_pose = false;
        }
        if (ctx_->has_pipeline_route && ctx_->last_pipeline_route_received.nanoseconds() != 0 &&
            (now - ctx_->last_pipeline_route_received).seconds() > pipeline_route_timeout_s_) {
            ctx_->has_pipeline_route = false;
        }
        if (ctx_->has_dvl_live && ctx_->last_dvl_live_received.nanoseconds() != 0 &&
            (now - ctx_->last_dvl_live_received).seconds() > task2_dvl_timeout_s_) {
            ctx_->has_dvl_live = false;
        }
    });

    setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);
    robocommand_pub_ = this->create_publisher<std_msgs::msg::String>("/banyu/robocommand", 10);

    start_service_ = this->create_service<std_srvs::srv::SetBool>(
        "banyu/mission/start", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            (void)req;
            std::string failure_message;
            res->success = startMission(failure_message);
            res->message = res->success ? "mission started" : failure_message;
        });

    abort_service_ = this->create_service<std_srvs::srv::SetBool>(
        "banyu/mission/abort", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            (void)req;
            abortMission();
            res->success = true;
            res->message = "mission aborted";
        });

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    handoff_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&BanyuMission::onHandoffTimer, this));
    handoff_timer_->cancel();
}

void BanyuMission::setupBehaviorTree() {
    factory_.registerNodeType<AbortedOrTeleop>("AbortedOrTeleop", ctx_);
    factory_.registerNodeType<MissionActive>("MissionActive", ctx_);
    factory_.registerNodeType<NavToGate>("NavToGate", this->shared_from_this(), ctx_);
    factory_.registerNodeType<GateAlign>("GateAlign", this->shared_from_this(), ctx_);
    factory_.registerNodeType<GatePass>("GatePass", this->shared_from_this(), ctx_);
    factory_.registerNodeType<SearchMarker>("SearchMarker", this->shared_from_this(), ctx_);
    factory_.registerNodeType<MarkerAlign>("MarkerAlign", this->shared_from_this(), ctx_);
    factory_.registerNodeType<MarkerOrbit>("MarkerOrbit", this->shared_from_this(), ctx_);
    factory_.registerNodeType<TurnAround>("TurnAround", this->shared_from_this(), ctx_);
    factory_.registerNodeType<SearchGate>("SearchGate", this->shared_from_this(), ctx_);
    factory_.registerNodeType<ReturnGatePass>("ReturnGatePass", this->shared_from_this(), ctx_);
    factory_.registerNodeType<Surface>("Surface", this->shared_from_this(), ctx_);
    factory_.registerNodeType<Task2Preflight>("Task2Preflight", this->shared_from_this(), ctx_);
    factory_.registerNodeType<FollowTask2Pipeline>("FollowTask2Pipeline", this->shared_from_this(), ctx_);

    const std::filesystem::path share_dir = ament_index_cpp::get_package_share_directory("banyu_mission");
    const std::filesystem::path tree_name =
        mission_mode_ == "task2" ? "mission_tree_task2.xml" : "mission_tree.xml";
    const std::filesystem::path tree_file = share_dir / "bt" / tree_name;

    factory_.registerBehaviorTreeFromFile(tree_file);
    tree_ = factory_.createTree("BanyuMission");
    tree_.initialize();

    auto blackboard = tree_.rootBlackboard();
    blackboard->set("mission_active", false);
    blackboard->set("aborted", false);
    blackboard->set("gate_offset_north", this->get_parameter("gate_offset_north").as_double());
    blackboard->set("gate_offset_east", this->get_parameter("gate_offset_east").as_double());
    blackboard->set("gate_offset_down", this->get_parameter("gate_offset_down").as_double());

    const int groot2_port = this->get_parameter("groot2_port").as_int();
    groot2_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, groot2_port);
    RCLCPP_INFO(this->get_logger(), "BehaviorTree 'BanyuMission' loaded, Groot2 on port %d", groot2_port);
}

bool BanyuMission::startMission(std::string& failure_message) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (shutting_down_.load()) {
        failure_message = "node is shutting down";
        return false;
    }
    if (handoff_active_.load()) {
        failure_message = "handoff in progress";
        return false;
    }
    if (ctx_->mission_active.load()) {
        failure_message = "mission already active";
        return false;
    }
    if (mission_mode_ == "task2" && !this->get_parameter("task2_fixture_mode").as_bool()) {
        failure_message = "task2 fixture mode is disabled";
        return false;
    }

    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }

    ctx_->mission_active.store(false);
    ctx_->aborted.store(false);
    {
        std::lock_guard<std::mutex> lock(ctx_->data_mutex);
        ctx_->mission_origin.reset();
        ctx_->gate_target.reset();
    }

    armAndSetGuided();

    ctx_->mission_active.store(true);
    tick_thread_ = std::thread([this]() {
        const BT::NodeStatus status = tree_.tickWhileRunning(std::chrono::milliseconds(20));
        if (status == BT::NodeStatus::FAILURE) {
            ctx_->aborted.store(true);
            ctx_->mission_active.store(false);
            beginHandoff();
        }
        if (status == BT::NodeStatus::SUCCESS && mission_mode_ == "task2") {
            ctx_->mission_active.store(false);
            beginHandoff();
        }
        ctx_->mission_active.store(false);
        ctx_->aborted.store(false);
        if (status == BT::NodeStatus::SUCCESS) {
            if (mission_mode_ == "task2") {
                RCLCPP_INFO(this->get_logger(),
                            "Task 2 mission finished (SUCCESS); handoff command published.");
            } else {
                RCLCPP_INFO(this->get_logger(), "Mission tree finished (SUCCESS). Handing control back.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Mission tree finished (FAILURE). Handing control back.");
        }
    });
    return true;
}

void BanyuMission::abortMission() {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (!ctx_->mission_active.load() && !handoff_active_.load()) {
        return;
    }

    ctx_->aborted.store(true);
    ctx_->mission_active.store(false);
    RCLCPP_WARN(this->get_logger(), "Mission aborted by request.");

    beginHandoff();

    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }
}

void BanyuMission::beginHandoff() {
    if (shutting_down_.load()) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(handoff_mutex_);
        handoff_active_.store(true);
        handoff_request_in_flight_.store(false);
        ++handoff_request_generation_;
    }

    if (ctx_->setpoint_pub) {
        ctx_->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, 0.0, this->now()));
    }
    if (handoff_timer_) {
        handoff_timer_->reset();
    }
}

void BanyuMission::onHandoffTimer() {
    if (shutting_down_.load() || !handoff_active_.load()) {
        return;
    }

    const rclcpp::Time now = this->now();
    std::string current_mode;
    {
        std::lock_guard<std::mutex> lock(ctx_->data_mutex);
        current_mode = ctx_->state.mode;
    }

    if (ctx_->setpoint_pub) {
        ctx_->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, 0.0, now));
    }

    const std::string target_mode = this->get_parameter("abort_mode").as_string();
    if (current_mode == target_mode) {
        {
            std::lock_guard<std::mutex> lock(handoff_mutex_);
            handoff_active_.store(false);
            handoff_request_in_flight_.store(false);
            ++handoff_request_generation_;
        }
        handoff_timer_->cancel();
        return;
    }

    const auto wall_now = std::chrono::steady_clock::now();
    uint64_t request_generation = 0;
    {
        std::lock_guard<std::mutex> lock(handoff_mutex_);
        if (!handoff_active_.load()) {
            return;
        }
        if (handoff_request_in_flight_.load() && wall_now < handoff_request_deadline_) {
            return;
        }
        if (handoff_request_in_flight_.load()) {
            handoff_request_in_flight_.store(false);
            ++handoff_request_generation_;
        }
        if (!set_mode_client_->service_is_ready()) {
            return;
        }

        request_generation = ++handoff_request_generation_;
        handoff_request_in_flight_.store(true);
        const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(std::max(0.05, handoff_request_timeout_s_)));
        handoff_request_deadline_ = wall_now + timeout;
    }

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = target_mode;
    set_mode_client_->async_send_request(
        request, [this, request_generation](
                     rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
            (void)future;
            if (shutting_down_.load()) {
                return;
            }
            std::lock_guard<std::mutex> lock(handoff_mutex_);
            if (request_generation == handoff_request_generation_) {
                handoff_request_in_flight_.store(false);
            }
        });
}

void BanyuMission::armAndSetGuided() {
    auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req->value = true;
    if (arming_client_->wait_for_service(std::chrono::seconds(1))) {
        arming_client_->async_send_request(
            arm_req, [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
                const auto res = future.get();
                if (res->success) {
                    RCLCPP_INFO(this->get_logger(), "Vehicle armed");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Arm command failed (result=%d)", res->result);
                }
            });
    } else {
        RCLCPP_WARN(this->get_logger(), "Arming service not available");
    }

    auto mode_req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    mode_req->custom_mode = "GUIDED";
    if (set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        set_mode_client_->async_send_request(mode_req,
                                             [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
                                                 const auto res = future.get();
                                                 if (res->mode_sent) {
                                                     RCLCPP_INFO(this->get_logger(), "Mode set to GUIDED");
                                                 } else {
                                                     RCLCPP_ERROR(this->get_logger(), "Failed to set GUIDED mode");
                                                 }
                                             });
    } else {
        RCLCPP_WARN(this->get_logger(), "SetMode service not available");
    }
}

void BanyuMission::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->current_pose = *msg;
    ctx_->has_pose = true;
    ctx_->last_pose_received = this->now();
}

void BanyuMission::onState(const mavros_msgs::msg::State::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->state = *msg;
    ctx_->has_state = true;
    ctx_->last_state_received = this->now();
}

void BanyuMission::onDetection(const banyu_msgs::msg::Detection::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->latest_detection = *msg;
    ctx_->has_detection = true;
    ctx_->last_detection_received = this->now();
}

void BanyuMission::onHydrophoneHeading(const banyu_msgs::msg::HydrophoneHeading::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->latest_hydrophone_heading = *msg;
    ctx_->has_hydrophone_heading = true;
    ctx_->last_hydrophone_heading_received = this->now();
}

void BanyuMission::onPipelineRingState(const banyu_msgs::msg::PipelineRingState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    const rclcpp::Time receipt_time = this->now();
    ctx_->latest_pipeline_ring_state = *msg;
    ctx_->has_pipeline_ring_state = true;
    ctx_->last_pipeline_ring_state_received = receipt_time;
    ctx_->pipeline_ring_states[msg->index] = *msg;
    ctx_->pipeline_ring_state_receipt_times[msg->index] = receipt_time;
}

void BanyuMission::onActiveBuoyPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->active_buoy_pose = *msg;
    ctx_->has_active_buoy_pose = true;
    ctx_->last_active_buoy_pose_received = this->now();
}

void BanyuMission::onPipelineRoute(const nav_msgs::msg::Path::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->latest_pipeline_route = *msg;
    ctx_->has_pipeline_route = true;
    ctx_->last_pipeline_route_received = this->now();
}

void BanyuMission::onDvlLive(const std_msgs::msg::Bool::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->dvl_live = msg->data;
    ctx_->has_dvl_live = true;
    ctx_->last_dvl_live_received = this->now();
}

}  // namespace banyu_mission

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<banyu_mission::BanyuMission>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
