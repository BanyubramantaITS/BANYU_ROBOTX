// Sim stand-in for the UUV's hydrophone array (Gazebo has no acoustics). Attached to the vehicle
// model; once per ping it publishes the first-arrival time at each hydrophone from the true
// vehicle and pinger poses, as gz.msgs.Float_V (bridged to ros_gz_interfaces/Float32Array).
//
//   <plugin filename="BanyuHydrophoneArray" name="banyu::HydrophoneArray">
//     <pinger_model>pinger_green</pinger_model>   (only the active buoy's pinger pings)
//     <acoustic_offset>0 0 -0.186</acoustic_offset> (acoustic centre in the pinger model frame)
//     <hydrophone>0.15 0 -0.1</hydrophone>        (body frame, one per hydrophone)
//     <ping_rate>1.0</ping_rate>                  (Hz; handbook range 0.5-2)
//     <jitter>2e-6</jitter>                       (s, first-arrival pick noise per hydrophone)
//     <topic>/banyu/pinger/arrivals</topic>
//   </plugin>
//
// First arrival = direct path, so multipath is deliberately left out. Nothing is published
// while any hydrophone is out of the water.

#include <random>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/float_v.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace banyu
{
class HydrophoneArray : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    vehicle_ = _entity;
    pinger_name_ = _sdf->Get<std::string>("pinger_model", "pinger_green").first;
    offset_ = _sdf->Get<gz::math::Vector3d>("acoustic_offset", {0, 0, -0.186}).first;
    period_s_ = 1.0 / _sdf->Get<double>("ping_rate", 1.0).first;
    jitter_ = std::normal_distribution<double>(0.0, _sdf->Get<double>("jitter", 2e-6).first);
    for (auto e = _sdf->FindElement("hydrophone"); e; e = e->GetNextElement("hydrophone"))
      hydrophones_.push_back(e->Get<gz::math::Vector3d>());
    pub_ = node_.Advertise<gz::msgs::Float_V>(_sdf->Get<std::string>("topic", "/banyu/pinger/arrivals").first);
    if (hydrophones_.size() < 3)
      gzerr << "[HydrophoneArray] need at least 3 <hydrophone> elements\n";
  }

  void PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || hydrophones_.size() < 3)
      return;
    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (t < next_ping_s_)
      return;
    next_ping_s_ = t + period_s_;

    if (pinger_ == gz::sim::kNullEntity) {
      pinger_ = _ecm.EntityByComponents(gz::sim::components::Name(pinger_name_), gz::sim::components::Model());
      if (pinger_ == gz::sim::kNullEntity)
        return;  // world still loading
    }
    const gz::math::Pose3d vehicle = gz::sim::worldPose(vehicle_, _ecm);
    const gz::math::Vector3d source = gz::sim::worldPose(pinger_, _ecm).CoordPositionAdd(offset_);

    std::vector<double> arrivals;
    for (const auto &h : hydrophones_) {
      const gz::math::Vector3d p = vehicle.CoordPositionAdd(h);
      if (p.Z() >= 0)
        return;  // a dry hydrophone hears nothing
      arrivals.push_back(p.Distance(source) / kSoundSpeed + jitter_(rng_));
    }
    double mean = 0;
    for (double a : arrivals) mean += a / arrivals.size();

    gz::msgs::Float_V msg;
    for (double a : arrivals)
      msg.add_data(static_cast<float>(a - mean));  // relative times: float keeps the us precision
    pub_.Publish(msg);
  }

private:
  static constexpr double kSoundSpeed = 1500.0;
  gz::sim::Entity vehicle_{gz::sim::kNullEntity};
  gz::sim::Entity pinger_{gz::sim::kNullEntity};
  std::string pinger_name_;
  gz::math::Vector3d offset_;
  std::vector<gz::math::Vector3d> hydrophones_;
  double period_s_{1.0};
  double next_ping_s_{0.0};
  std::mt19937 rng_{std::random_device{}()};
  std::normal_distribution<double> jitter_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
};
}  // namespace banyu

GZ_ADD_PLUGIN(banyu::HydrophoneArray, gz::sim::System, banyu::HydrophoneArray::ISystemConfigure,
              banyu::HydrophoneArray::ISystemPostUpdate)
