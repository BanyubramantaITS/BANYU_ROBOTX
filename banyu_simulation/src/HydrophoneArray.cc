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
//     <blocker>-0.15 -0.057 0.003 0.19 0.057 0.117</blocker>  (solid hull box, body frame:
//                                                   min xyz, max xyz; any number)
//     <shadow_pick_delay>30e-6</shadow_pick_delay> (s, sigma of the half-normal lateness of a
//                                                   shadowed pick)
//     <topic>/banyu/pinger/arrivals</topic>
//   </plugin>
//
// First arrival = direct path, or the shortest path around the hull when a <blocker> is in the
// way (acoustic_shadow.hpp). A shadowed hydrophone also hears a weaker, diffracted wave, so its
// threshold pick lands late: half-normal, sigma shadow_pick_delay, about one carrier cycle.
// ponytail: shadow_pick_delay is uncalibrated; measure pick lateness behind the real hull.
// Multipath is deliberately left out. Nothing is published while any hydrophone is out of the water.

#include <cmath>
#include <random>
#include <sstream>
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

#include "acoustic_shadow.hpp"

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
    std::vector<Box> boxes;
    for (auto e = _sdf->FindElement("blocker"); e; e = e->GetNextElement("blocker")) {
      std::istringstream in(e->Get<std::string>());
      Box b;
      in >> b.lo.X() >> b.lo.Y() >> b.lo.Z() >> b.hi.X() >> b.hi.Y() >> b.hi.Z();
      if (in.fail())
        gzerr << "[HydrophoneArray] <blocker> needs 6 numbers: min xyz, max xyz\n";
      else
        boxes.push_back(b);
    }
    shadow_ = ShadowPaths(boxes, hydrophones_);
    shadow_pick_ = std::normal_distribution<double>(0.0, _sdf->Get<double>("shadow_pick_delay", 30e-6).first);
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

    const gz::math::Vector3d source_body = vehicle.Rot().RotateVectorReverse(source - vehicle.Pos());

    std::vector<double> arrivals;
    for (size_t i = 0; i < hydrophones_.size(); ++i) {
      if (vehicle.CoordPositionAdd(hydrophones_[i]).Z() >= 0)
        return;  // a dry hydrophone hears nothing
      bool shadowed = false;
      double t_i = shadow_.Length(source_body, i, &shadowed) / kSoundSpeed + jitter_(rng_);
      if (shadowed)
        t_i += std::abs(shadow_pick_(rng_));
      arrivals.push_back(t_i);
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
  std::normal_distribution<double> shadow_pick_;
  ShadowPaths shadow_;
  gz::transport::Node node_;
  gz::transport::Node::Publisher pub_;
};
}  // namespace banyu

GZ_ADD_PLUGIN(banyu::HydrophoneArray, gz::sim::System, banyu::HydrophoneArray::ISystemConfigure,
              banyu::HydrophoneArray::ISystemPostUpdate)
