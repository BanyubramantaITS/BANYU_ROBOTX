#include "banyu_mission/banyu_mission.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "banyu_mission/bt_nodes/aborted_or_teleop.hpp"
#include "banyu_mission/bt_nodes/gate_align.hpp"
#include "banyu_mission/bt_nodes/gate_pass.hpp"
#include "banyu_mission/bt_nodes/marker_align.hpp"
#include "banyu_mission/bt_nodes/marker_orbit.hpp"
#include "banyu_mission/bt_nodes/mission_active.hpp"
#include "banyu_mission/bt_nodes/nav_to_gate.hpp"
#include "banyu_mission/bt_nodes/return_gate_pass.hpp"
#include "banyu_mission/bt_nodes/search_gate.hpp"
#include "banyu_mission/bt_nodes/search_marker.hpp"
#include "banyu_mission/bt_nodes/surface.hpp"
#include "banyu_mission/bt_nodes/turn_around.hpp"

namespace banyu_mission {

// Declare Params
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
}

// Setup BT
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

    const std::filesystem::path share_dir = ament_index_cpp::get_package_share_directory("banyu_mission");
    const std::filesystem::path tree_file = share_dir / "bt" / "mission_tree.xml";

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

// Interfaces
BanyuMission::BanyuMission(const rclcpp::NodeOptions& options) : Node("banyu_mission", options), ctx_(std::make_shared<MissionContext>()) {
    declareParameters();

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("mavros/local_position/pose", rclcpp::SensorDataQoS(),
                                                                           std::bind(&BanyuMission::onPose, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("mavros/state", 10, std::bind(&BanyuMission::onState, this, std::placeholders::_1));
    detection_sub_ = this->create_subscription<banyu_msgs::msg::Detection>("banyu/detection", rclcpp::SensorDataQoS(),
                                                                           std::bind(&BanyuMission::onDetection, this, std::placeholders::_1));

    detection_watchdog_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), [this]() {
        std::lock_guard<std::mutex> lock(ctx_->data_mutex);
        if (!ctx_->has_detection || ctx_->last_detection_received.nanoseconds() == 0) {
            return;
        }

        if ((this->now() - ctx_->last_detection_received).seconds() > detection_timeout_s_) {
            ctx_->has_detection = false;
        }
    });

    setpoint_pub_ = this->create_publisher<mavros_msgs::msg::PositionTarget>("mavros/setpoint_raw/local", 10);

    start_service_ = this->create_service<std_srvs::srv::SetBool>(
        "banyu/mission/start", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            (void)req;
            if (ctx_->mission_active.load()) {
                res->success = false;
                res->message = "mission already active";
                return;
            }
            startMission();
            res->success = true;
            res->message = "mission started";
        });

    abort_service_ = this->create_service<std_srvs::srv::SetBool>(
        "banyu/mission/abort", [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req, std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
            (void)req;
            abortMission();
            res->success = true;
            res->message = "mission aborted";
        });

    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    ctx_->setpoint_pub = setpoint_pub_;
}

void BanyuMission::setAbortMode() {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = this->get_parameter("abort_mode").as_string();
    if (set_mode_client_->wait_for_service(std::chrono::seconds(1))) {
        set_mode_client_->async_send_request(request);
    } else {
        RCLCPP_WARN(this->get_logger(), "SetMode service unavailable while handing control back");
    }
}

void BanyuMission::armAndSetGuided() {
    auto arm_req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    arm_req->value = true;
    if (arming_client_->wait_for_service(std::chrono::seconds(1))) {
        arming_client_->async_send_request(arm_req, [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
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
        set_mode_client_->async_send_request(mode_req, [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
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
}

void BanyuMission::onState(const mavros_msgs::msg::State::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->state = *msg;
}

void BanyuMission::onDetection(const banyu_msgs::msg::Detection::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(ctx_->data_mutex);
    ctx_->latest_detection = *msg;
    ctx_->has_detection = true;
    ctx_->last_detection_received = this->now();
}

// Lifecycle
void BanyuMission::initialize() {
    setupBehaviorTree();
    RCLCPP_INFO(this->get_logger(), "BanyuMission node has been initialized.");
}

BanyuMission::~BanyuMission() {
    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }
    if (tree_.rootNode() != nullptr) {
        tree_.haltTree();
    }
}

void BanyuMission::startMission() {
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
            setAbortMode();
        }
        ctx_->mission_active.store(false);
        ctx_->aborted.store(false);
        if (status == BT::NodeStatus::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Mission tree finished (SUCCESS). Handing control back.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Mission tree finished (FAILURE). Handing control back.");
        }
    });
}

void BanyuMission::abortMission() {
    if (!ctx_->mission_active.load()) {
        return;
    }

    ctx_->aborted.store(true);
    ctx_->mission_active.store(false);
    RCLCPP_WARN(this->get_logger(), "Mission aborted by request.");

    if (ctx_->setpoint_pub) {
        ctx_->setpoint_pub->publish(velocitySetpoint(0.0, 0.0, 0.0, this->now()));
    }

    setAbortMode();

    if (tick_thread_.joinable()) {
        tick_thread_.join();
    }
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
