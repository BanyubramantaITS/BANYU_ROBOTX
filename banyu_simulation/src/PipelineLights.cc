// RobotX 2026 Task 2 pipeline lights (handbook p.65): on activation one light
// node turns RED and the rest GREEN. This system picks the RED one and
// recolours the "light" visual of each listed light node model.
//
//   <plugin filename="BanyuPipelineLights" name="banyu::PipelineLights">
//     <light_node>light_node_1</light_node>   (one per node, reporting order)
//     <seed>42</seed>                         (optional; env BANYU_LIGHT_SEED wins)
//   </plugin>

#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/visual.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
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

    std::string pattern;
    for (size_t i = 0; i < names_.size(); ++i)
      pattern += (i == red_ ? " RED" : " green");
    gzmsg << "[PipelineLights] seed " << seed << " -> damaged section " << red_ + 1 << " of "
          << names_.size() << " [" << pattern << " ]\n";
  }

  // Applied on the first update, once every included model exists in the ECM.
  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &_ecm) override
  {
    if (done_ || names_.empty())
      return;

    for (size_t i = 0; i < names_.size(); ++i) {
      gz::sim::Model model(_ecm.EntityByComponents(gz::sim::components::Name(names_[i]),
                                                   gz::sim::components::Model()));
      auto visual = gz::sim::Link(model.LinkByName(_ecm, "link")).VisualByName(_ecm, "light");
      if (visual == gz::sim::kNullEntity) {
        gzerr << "[PipelineLights] " << names_[i] << " has no link/light visual\n";
        continue;
      }
      SetColor(_ecm, visual, i == red_ ? kRed : kGreen);
    }
    done_ = true;
  }

private:
  struct Rgb { float r, g, b; };
  // Same values the Blender export bakes into the red/green light node variants.
  static constexpr Rgb kRed{1.0f, 0.06f, 0.06f};
  static constexpr Rgb kGreen{0.06f, 1.0f, 0.12f};

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
  bool done_ = false;
};
}  // namespace banyu

GZ_ADD_PLUGIN(banyu::PipelineLights, gz::sim::System, banyu::PipelineLights::ISystemConfigure,
              banyu::PipelineLights::ISystemPreUpdate)
