// RobotX 2026 Task 2 pipeline lights (handbook p.65): on activation one light
// node turns RED and the rest GREEN. This system picks the RED one and
// recolours the "light" visual of each listed light node model.
//
// Repair (Core extra / Advanced and Disruptive tiers, p.66-67): once the probe
// tip stays within switch_radius of the RED node's magnetic switch for
// repair_hold, the node turns GREEN for 5 s, then for 30 s flashes
//   advanced:   the circle colour, 1 s on / 1 s off
//   disruptive: the tin colour 1 s, off 1 s, the circle colour 1 s, off 2 s
// and then goes dark.
//
//   <plugin filename="BanyuPipelineLights" name="banyu::PipelineLights">
//     <light_node>light_node_1</light_node>   (one per node, reporting order)
//     <seed>42</seed>                         (optional; env BANYU_LIGHT_SEED wins)
//     <probe_model>bluerov2_heavy</probe_model>
//     <probe_link>probe_tip</probe_link>      (link origin = the magnet)
//     <switch_offset>0 0 0.0414</switch_offset> (light node frame; its magnet_target)
//     <switch_radius>0.02</switch_radius>     (m)
//     <repair_hold>3</repair_hold>            (s)
//     <flash_colour>blue</flash_colour>       (optional red|green|blue; else from the seed)
//     <flash_colour2>red</flash_colour2>      (disruptive circle colour; same)
//     <tier>advanced</tier>                   (advanced|disruptive; env BANYU_TIER wins)
//   </plugin>
//
// ponytail: switch_radius and repair_hold are guesses: the handbook puts the
// switch at the ring's centre and asks for the probe "held in place long
// enough", no numbers. Measure them on the real light node.

#include <cmath>
#include <cstdlib>
#include <chrono>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/visual.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/VisualCmd.hh>

namespace banyu
{
class PipelineLights : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    for (auto e = _sdf->FindElement("light_node"); e; e = e->GetNextElement("light_node"))
      names_.push_back(e->Get<std::string>());
    if (names_.empty()) {
      gzerr << "[PipelineLights] no <light_node> given\n";
      return;
    }

    unsigned seed = std::random_device{}();
    if (_sdf->HasElement("seed"))
      seed = _sdf->Get<unsigned>("seed");
    if (const char *env = std::getenv("BANYU_LIGHT_SEED"); env && *env)
      seed = static_cast<unsigned>(std::stoul(env));

    std::mt19937 rng(seed);
    red_ = std::uniform_int_distribution<size_t>(0, names_.size() - 1)(rng);
    const std::string names[] = {"red", "green", "blue"};
    auto pick = [&](const char *element) {
      int c = std::uniform_int_distribution<int>(0, 2)(rng);
      const std::string want = _sdf->Get<std::string>(element, "").first;
      for (int i = 0; i < 3; ++i)
        if (want == names[i])
          c = i;
      return c;
    };
    flash_ = pick("flash_colour");
    flash2_ = pick("flash_colour2");
    std::string tier = _sdf->Get<std::string>("tier", "advanced").first;
    if (const char *env = std::getenv("BANYU_TIER"); env && *env)
      tier = env;
    disruptive_ = tier == "disruptive";

    probe_model_ = _sdf->Get<std::string>("probe_model", "bluerov2_heavy").first;
    probe_link_ = _sdf->Get<std::string>("probe_link", "probe_tip").first;
    switch_offset_ = _sdf->Get<gz::math::Vector3d>("switch_offset", {0, 0, 0.0414}).first;
    switch_radius_ = _sdf->Get<double>("switch_radius", 0.02).first;
    repair_hold_ = _sdf->Get<double>("repair_hold", 3.0).first;

    std::string pattern;
    for (size_t i = 0; i < names_.size(); ++i)
      pattern += (i == red_ ? " RED" : " green");
    gzmsg << "[PipelineLights] seed " << seed << " -> damaged section " << red_ + 1 << " of "
          << names_.size() << " [" << pattern << " ], repair flash " << names[flash_]
          << (disruptive_ ? " then " + names[flash2_] + " (disruptive)" : std::string()) << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info, gz::sim::EntityComponentManager &_ecm) override
  {
    if (names_.empty())
      return;

    // Colours applied on the first update, once every included model exists in the ECM.
    if (!started_) {
      for (size_t i = 0; i < names_.size(); ++i) {
        gz::sim::Model model(_ecm.EntityByComponents(gz::sim::components::Name(names_[i]),
                                                     gz::sim::components::Model()));
        auto visual = gz::sim::Link(model.LinkByName(_ecm, "link")).VisualByName(_ecm, "light");
        if (visual == gz::sim::kNullEntity) {
          gzerr << "[PipelineLights] " << names_[i] << " has no link/light visual\n";
          continue;
        }
        SetColor(_ecm, visual, i == red_ ? kRed : kGreen);
        if (i == red_) {
          red_visual_ = visual;
          red_switch_ = gz::sim::worldPose(model.Entity(), _ecm).CoordPositionAdd(switch_offset_);
        }
      }
      started_ = true;
    }
    if (_info.paused || red_visual_ == gz::sim::kNullEntity)
      return;

    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (repaired_at_ < 0) {
      // The vehicle is spawned after the world loads: look for its probe until it exists.
      if (probe_ == gz::sim::kNullEntity) {
        gz::sim::Model vehicle(_ecm.EntityByComponents(gz::sim::components::Name(probe_model_),
                                                       gz::sim::components::Model()));
        probe_ = vehicle.LinkByName(_ecm, probe_link_);
        if (probe_ == gz::sim::kNullEntity)
          return;
      }
      if (gz::sim::worldPose(probe_, _ecm).Pos().Distance(red_switch_) > switch_radius_) {
        held_since_ = -1;
        return;
      }
      if (held_since_ < 0)
        held_since_ = t;
      if (t - held_since_ < repair_hold_)
        return;
      repaired_at_ = t;
      gzmsg << "[PipelineLights] section " << red_ + 1 << " repaired at t=" << t << " s\n";
    }

    // Repaired: GREEN 5 s, then the tier's flash pattern for 30 s, then dark.
    const double since = t - repaired_at_ - 5.0;
    const Rgb flash[] = {kRed, kGreen, kBlue};
    Rgb want = kOff;
    if (since < 0)
      want = kGreen;
    else if (since < 30.0 && !disruptive_ && std::fmod(since, 2.0) < 1.0)
      want = flash[flash_];
    else if (since < 30.0 && disruptive_ && std::fmod(since, 5.0) < 1.0)
      want = flash[flash_];
    else if (since < 30.0 && disruptive_ && std::fmod(since, 5.0) >= 2.0 && std::fmod(since, 5.0) < 3.0)
      want = flash[flash2_];
    if (!shown_ || shown_->r != want.r || shown_->g != want.g || shown_->b != want.b) {
      SetColor(_ecm, red_visual_, want);
      shown_ = want;
    }
  }

private:
  struct Rgb { float r, g, b; };
  // Same values the Blender export bakes into the red/green light node variants.
  static constexpr Rgb kRed{1.0f, 0.06f, 0.06f};
  static constexpr Rgb kGreen{0.06f, 1.0f, 0.12f};
  static constexpr Rgb kBlue{0.06f, 0.2f, 1.0f};
  static constexpr Rgb kOff{0.0f, 0.0f, 0.0f};

  // Material covers anything that builds its scene later (a GUI joining late);
  // VisualCmd is how UserCommands recolours visuals that already exist.
  static void SetColor(gz::sim::EntityComponentManager &_ecm, gz::sim::Entity _visual, Rgb _c)
  {
    const gz::math::Color color(_c.r, _c.g, _c.b, 1.0f);
    if (auto *comp = _ecm.Component<gz::sim::components::Material>(_visual)) {
      comp->Data().SetAmbient(color);
      comp->Data().SetDiffuse(color);
      comp->Data().SetEmissive(color);
      _ecm.SetChanged(_visual, gz::sim::components::Material::typeId,
                      gz::sim::ComponentState::OneTimeChange);
    }

    gz::msgs::Visual msg;
    auto *mat = msg.mutable_material();
    for (auto *col : {mat->mutable_ambient(), mat->mutable_diffuse(), mat->mutable_emissive()}) {
      col->set_r(_c.r);
      col->set_g(_c.g);
      col->set_b(_c.b);
      col->set_a(1.0f);
    }
    _ecm.SetComponentData<gz::sim::components::VisualCmd>(_visual, msg);
    _ecm.SetChanged(_visual, gz::sim::components::VisualCmd::typeId,
                    gz::sim::ComponentState::OneTimeChange);
  }

  std::vector<std::string> names_;
  size_t red_ = 0;
  int flash_ = 0, flash2_ = 0;  // indices into {red, green, blue}
  bool disruptive_ = false;
  bool started_ = false;

  std::string probe_model_, probe_link_;
  gz::math::Vector3d switch_offset_;
  double switch_radius_ = 0.02, repair_hold_ = 3.0;
  gz::sim::Entity probe_ = gz::sim::kNullEntity;
  gz::sim::Entity red_visual_ = gz::sim::kNullEntity;
  gz::math::Vector3d red_switch_;  // world; the pipeline is static
  double held_since_ = -1, repaired_at_ = -1;
  std::optional<Rgb> shown_;
};
}  // namespace banyu

GZ_ADD_PLUGIN(banyu::PipelineLights, gz::sim::System, banyu::PipelineLights::ISystemConfigure,
              banyu::PipelineLights::ISystemPreUpdate)
