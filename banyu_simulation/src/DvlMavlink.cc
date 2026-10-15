// Gazebo DVL -> ArduSub SITL as MAVLink VISION_POSITION_DELTA. Sim only; real
// hardware runs the A50's own driver and everything downstream is unchanged.
//
//   <plugin filename="BanyuDvlMavlink" name="banyu::DvlMavlink">
//     <topic>/dvl/velocity</topic>   (optional, this default)
//     <port>14552</port>             (optional; sim_vehicle.py --out=udp:127.0.0.1:<port>)
//   </plugin>
//
// Why MAVLink straight to SITL instead of ros_gz_bridge + MAVROS:
// - MAVROS cannot emit VISION_POSITION_DELTA (none of its 40 plugins does), and
//   that is the *only* message that bootstraps EKF3 aiding from a DVL alone:
//       VISION_POSITION_DELTA -> writeBodyFrameOdom() -> readyToUseBodyOdm()
//                             -> AID_RELATIVE
//   VISION_SPEED_ESTIMATE and ODOMETRY-twist reach writeExtNavVelData(), which
//   is in neither AID_NONE exit branch (AP_NavEKF3_Control.cpp:284-299).
// - ros_gz_bridge on Humble cannot carry gz.msgs.DVLVelocityTracking.
//
// SITL streams to <port>; we bind it, learn SITL's address from whatever it
// sends, and reply there. Nothing is sent until SITL has been heard from.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <string>

#include <ardupilotmega/mavlink.h>

#include <gz/common/Console.hh>
#include <gz/msgs/dvl_velocity_tracking.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace banyu
{
// Frame mapping, established EMPIRICALLY (do not "fix" from first principles).
// Driving an unrotated probe along each world axis and reading the sensor gave:
//     world +X 0.5 -> dvl reports y = +0.500
//     world +Y 0.5 -> dvl reports x = -0.500
//     world +Z 0.5 -> dvl reports z = +0.490
// i.e. the sensor frame is yawed +90 deg from the body frame, which is the
// <reference_frame>0 0 0 0 0 -1.570796</reference_frame> in the DVL model.
//
// Gazebo body frame is z-UP (x fwd, y left, z up).
// ArduPilot body frame is FRD (x fwd, y starboard, z down).
//   body_x =  dvl_y      FRD_x =  body_x =  dvl_y
//   body_y = -dvl_x  =>  FRD_y = -body_y =  dvl_x
//   body_z =  dvl_z      FRD_z = -body_z = -dvl_z
static void DvlToFrd(const gz::msgs::Vector3d &_v, float _out[3])
{
  _out[0] = static_cast<float>(_v.y());
  _out[1] = static_cast<float>(_v.x());
  _out[2] = static_cast<float>(-_v.z());
}

// Confidence -> the EKF's velocity error, via
//   velErr = EK3_VIS_VERR_MIN + (MAX-MIN)*(1 - quality/100)
//          = 0.1 + 0.8*(1 - quality/100)          (AP_NavEKF3_Measurements.cpp:138)
// Defaults put velErr in 0.1..0.9, always under readyToUseBodyOdm()'s < 1.0 gate,
// so confidence cannot *block* fusion -- it only scales how far the EKF trusts
// us. Send something honest anyway.
static constexpr float kCovFullDistrust = 0.5f;  // m/s stddev at which we report zero confidence

static float ConfidenceFromCov(const google::protobuf::RepeatedField<double> &_cov)
{
  if (_cov.size() < 9)
    return 0.0f;
  const double worst = std::max({_cov[0], _cov[4], _cov[8]});
  if (worst <= 0.0 || !std::isfinite(worst))
    return 0.0f;
  return std::clamp(100.0f * (1.0f - static_cast<float>(std::sqrt(worst)) / kCovFullDistrust), 0.0f, 100.0f);
}

class DvlMavlink : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  ~DvlMavlink() override
  {
    if (sock_ >= 0)
      close(sock_);
  }

  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    const auto topic = _sdf->Get<std::string>("topic", "/dvl/velocity").first;
    const auto port = _sdf->Get<int>("port", 14552).first;

    sock_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (sock_ < 0 || bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      gzerr << "[DvlMavlink] cannot bind UDP 127.0.0.1:" << port << "\n";
      return;
    }
    if (!node_.Subscribe(topic, &DvlMavlink::OnDvl, this)) {
      gzerr << "[DvlMavlink] cannot subscribe to " << topic << "\n";
      return;
    }
    gzmsg << "[DvlMavlink] " << topic << " -> VISION_POSITION_DELTA via UDP 127.0.0.1:" << port
          << "\n";
  }

private:
  void OnDvl(const gz::msgs::DVLVelocityTracking &_msg)
  {
    // Drain SITL's stream: keeps the peer address fresh and the socket empty.
    char buf[512];
    sockaddr_in from{};
    socklen_t len = sizeof(from);
    while (recvfrom(sock_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&from), &len) > 0) {
      peer_ = from;
      len = sizeof(from);
    }
    if (peer_.sin_port == 0)
      return;  // SITL not heard from yet
    if (!greeted_) {
      gzmsg << "[DvlMavlink] SITL heard on port " << ntohs(peer_.sin_port) << ", forwarding\n";
      greeted_ = true;
    }

    // Bottom lock only. Water-track velocity is relative to a moving medium
    // and would poison a dead-reckoned position delta.
    if (_msg.target().type() != gz::msgs::DVLTrackingTarget::DVL_TARGET_BOTTOM) {
      havePrev_ = false;  // gap: next delta would span the dropout
      return;
    }

    const uint64_t stamp = static_cast<uint64_t>(_msg.header().stamp().sec()) * 1000000u +
                           static_cast<uint64_t>(_msg.header().stamp().nsec()) / 1000u;
    const bool hadPrev = havePrev_;
    const uint64_t prev = prevStampUs_;
    prevStampUs_ = stamp;
    havePrev_ = true;
    if (!hadPrev || stamp <= prev)
      return;  // need two samples to form a delta

    const uint64_t dtUs = stamp - prev;
    const float dt = static_cast<float>(dtUs) * 1e-6f;
    float v[3];
    DvlToFrd(_msg.velocity().mean(), v);
    const float delta[3] = {v[0] * dt, v[1] * dt, v[2] * dt};
    // No angle delta: there is no attitude source here. The EKF then assumes
    // zero body rotation over the interval, which costs a small lever-arm error
    // while yawing (VISO_POS_Z is 0.10 m). Revisit if orbit-phase drift shows up.
    const float angle[3] = {0.0f, 0.0f, 0.0f};

    mavlink_message_t m;
    mavlink_msg_vision_position_delta_pack(1, 197, &m, stamp, dtUs, angle, delta,
                                           ConfidenceFromCov(_msg.velocity().covariance()));
    uint8_t out[MAVLINK_MAX_PACKET_LEN];
    const auto n = mavlink_msg_to_send_buffer(out, &m);
    sendto(sock_, out, n, 0, reinterpret_cast<sockaddr *>(&peer_), sizeof(peer_));
  }

  gz::transport::Node node_;
  int sock_ = -1;
  sockaddr_in peer_{};
  bool greeted_ = false;
  bool havePrev_ = false;
  uint64_t prevStampUs_ = 0;
};
}  // namespace banyu

GZ_ADD_PLUGIN(banyu::DvlMavlink, gz::sim::System, banyu::DvlMavlink::ISystemConfigure)
