// Underwater image formation for the listed server-side cameras (ogre2 only).
// A post-process render pass re-renders scene depth and applies, per channel,
//
//   I = J * exp(-beta * d) + B * (1 - exp(-beta * d)),  B = Binf * exp(-Kd * cam_depth)
//
// J rendered colour, d range to the surface seen by the pixel, beta beam
// attenuation, Binf veiling light just below the surface, Kd downwelling
// attenuation. Cameras above <surface_z> pass through untouched.
//
//   <plugin filename="BanyuUnderwaterTurbidity" name="banyu::UnderwaterTurbidity">
//     <camera>front_camera</camera>               sensor name, one per camera
//     <surface_z>0</surface_z>
//     <attenuation>0.45 0.12 0.18</attenuation>   beta, 1/m
//     <backscatter>0.06 0.26 0.30</backscatter>   Binf, linear RGB
//     <downwelling>0.15 0.04 0.06</downwelling>   Kd, 1/m
//     <min_range>0.2</min_range>   depth pass near clip, m: sees past the lens housing
//   </plugin>
//
// ponytail: sunlight reaching the object (exp(-Kd * object_depth)) is not
// applied, because rendered colour cannot tell sunlit surfaces from emissive
// ones and the task LEDs must not dim with depth. Split by material if the
// sunlit props ever need it.
// ponytail: GLSL only (GL3+, gz's default); Vulkan/Metal need their own shaders.
// ponytail: the GUI renders in its own process and stays clear; only sensor
// cameras (what the vehicle sees) get the effect.

#include <memory>
#include <set>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ogre2/Ogre2Camera.hh>
#include <gz/rendering/ogre2/Ogre2Scene.hh>
#include <gz/rendering/ogre2/Ogre2RenderPass.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/rendering/Events.hh>

#include <Compositor/OgreCompositorManager2.h>
#include <Compositor/OgreCompositorNodeDef.h>
#include <Compositor/Pass/PassQuad/OgreCompositorPassQuadDef.h>
#include <Compositor/Pass/PassScene/OgreCompositorPassSceneDef.h>
#include <OgreCamera.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreHlmsDatablock.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreTechnique.h>

namespace banyu
{
namespace
{
constexpr const char *kFragmentProgram = "BanyuUnderwaterFS";

// Pairs with Ogre's stock Ogre/Compositor/QuadCameraDir_vs, whose cameraDir
// is the frustum corner the quad pass feeds in (world axes, camera-relative).
constexpr const char *kFragmentSource = R"GLSL(
#version 330
in block
{
  vec2 uv0;
  vec3 cameraDir;
} inPs;

uniform sampler2D RT;
uniform sampler2D depthTexture;
uniform vec2 projectionParams;
uniform float cameraZ;
uniform float surfaceZ;
uniform vec3 attenuation;
uniform vec3 backscatter;
uniform vec3 downwelling;

out vec4 fragColour;

void main()
{
  vec4 colour = texture(RT, inPs.uv0);
  if (cameraZ > surfaceZ)
  {
    fragColour = colour;
    return;
  }
  // Linear depth as a fraction of the far plane, so cameraDir * it is the
  // camera-to-surface vector. Sky pixels land on the far plane.
  float linearDepth = projectionParams.y / (texture(depthTexture, inPs.uv0).x - projectionParams.x);
  float range = length(inPs.cameraDir * linearDepth);

  vec3 veil = backscatter * exp(-downwelling * (surfaceZ - cameraZ));
  vec3 t = exp(-attenuation * range);
  fragColour = vec4(colour.rgb * t + veil * (1.0 - t), colour.a);
}
)GLSL";

struct Params
{
  float surfaceZ = 0.0f;
  gz::math::Vector3d attenuation{0.45, 0.12, 0.18};
  gz::math::Vector3d backscatter{0.06, 0.26, 0.30};
  gz::math::Vector3d downwelling{0.15, 0.04, 0.06};
  // The BlueROV's semi-transparent front dome sits ~0.1 m ahead of the lens and
  // writes depth, which would read as "no water" across the middle of the frame.
  float minRange = 0.2f;
};

Ogre::Vector3 ToOgre(const gz::math::Vector3d &_v)
{
  return Ogre::Vector3(_v.X(), _v.Y(), _v.Z());
}

class TurbidityPass : public gz::rendering::Ogre2RenderPass
{
public:
  TurbidityPass(const Params &_p, Ogre::SceneManager *_sceneMgr) : params_(_p), sceneMgr_(_sceneMgr) {}

  void PreRender(const gz::rendering::CameraPtr &_camera) override
  {
    auto cam = std::dynamic_pointer_cast<gz::rendering::Ogre2Camera>(_camera);
    if (!material_ || !cam)
      return;

    // Depth camera shadows the sensor camera, differing only in near clip.
    const Ogre::Camera *ogreCam = cam->OgreCamera();
    depthCam_->setPosition(ogreCam->getDerivedPosition());
    depthCam_->setOrientation(ogreCam->getDerivedOrientation());
    depthCam_->setFOVy(ogreCam->getFOVy());
    depthCam_->setAspectRatio(ogreCam->getAspectRatio());
    depthCam_->setNearClipDistance(params_.minRange);
    depthCam_->setFarClipDistance(ogreCam->getFarClipDistance());

    Ogre::Vector2 ab = depthCam_->getProjectionParamsAB();
    ab.y /= depthCam_->getFarClipDistance();

    auto fs = material_->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
    fs->setNamedConstant("projectionParams", ab);
    fs->setNamedConstant("cameraZ", static_cast<float>(_camera->WorldPosition().Z()));
  }

  void CreateRenderPass() override
  {
    if (!this->ogreCompositorNodeDefName.empty())
      return;

    static int counter = 0;
    const std::string id = std::to_string(counter++);
    CreateMaterial("BanyuUnderwater_" + id);
    depthCam_ = sceneMgr_->createCamera("BanyuUnderwaterDepthCam_" + id);

    // Node chained between the camera's scene pass and its final copy:
    //   depthTex  <- depth-only re-render of the scene
    //   rt_output <- quad(rt_input, depthTex)
    auto *compMgr = Ogre::Root::getSingleton().getCompositorManager2();
    this->ogreCompositorNodeDefName = "BanyuUnderwaterNode_" + id;
    Ogre::CompositorNodeDef *node = compMgr->addNodeDefinition(this->ogreCompositorNodeDefName);
    node->addTextureSourceName("rt_input", 0, Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    node->addTextureSourceName("rt_output", 1, Ogre::TextureDefinitionBase::TEXTURE_INPUT);

    auto *depthDef = node->addTextureDefinition("depthTex");
    depthDef->format = Ogre::PFG_D32_FLOAT;
    depthDef->fsaa = "1";
    Ogre::RenderTargetViewDef *depthRtv = node->addRenderTextureView("depthRtv");
    depthRtv->depthAttachment.textureName = "depthTex";

    node->setNumTargetPass(2);
    Ogre::CompositorTargetDef *depthTarget = node->addTargetPass("depthRtv");
    depthTarget->setNumPasses(1);
    {
      auto *scene = static_cast<Ogre::CompositorPassSceneDef *>(depthTarget->addPass(Ogre::PASS_SCENE));
      scene->mCameraName = depthCam_->getName();
      scene->setAllLoadActions(Ogre::LoadAction::Clear);
      scene->mFirstRQ = 0u;
      scene->mLastRQ = 200u;  // gz: opaque items in RQ 10-11, transparent in 200+
      scene->mEnableForwardPlus = false;
      scene->mIncludeOverlays = false;
    }

    Ogre::CompositorTargetDef *outTarget = node->addTargetPass("rt_output");
    outTarget->setNumPasses(1);
    {
      auto *quad = static_cast<Ogre::CompositorPassQuadDef *>(outTarget->addPass(Ogre::PASS_QUAD));
      quad->mMaterialName = material_->getName();
      quad->addQuadTextureSource(0, "rt_input");
      quad->addQuadTextureSource(1, "depthTex");
      quad->mFrustumCorners = Ogre::CompositorPassQuadDef::WORLD_SPACE_CORNERS_CENTERED;
    }

    node->mapOutputChannel(0, "rt_output");
    node->mapOutputChannel(1, "rt_input");
  }

private:
  void CreateMaterial(const std::string &_name)
  {
    const std::string group = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
    auto &programs = Ogre::HighLevelGpuProgramManager::getSingleton();
    if (!programs.getByName(kFragmentProgram, group)) {
      auto fs = programs.createProgram(kFragmentProgram, group, "glsl", Ogre::GPT_FRAGMENT_PROGRAM);
      fs->setSource(kFragmentSource);
      fs->load();
    }

    material_ = Ogre::MaterialManager::getSingleton().create(_name, group);
    Ogre::Pass *pass = material_->getTechnique(0)->getPass(0);
    Ogre::HlmsMacroblock macro;
    macro.mDepthCheck = false;
    macro.mDepthWrite = false;
    macro.mCullMode = Ogre::CULL_NONE;
    pass->setMacroblock(macro);
    pass->setVertexProgram("Ogre/Compositor/QuadCameraDir_vs");
    pass->setFragmentProgram(kFragmentProgram);
    for (int i = 0; i < 2; ++i) {
      auto *tu = pass->createTextureUnitState();
      Ogre::HlmsSamplerblock sampler;
      sampler.setFiltering(Ogre::TFO_NONE);
      sampler.setAddressingMode(Ogre::TAM_CLAMP);
      tu->setSamplerblock(sampler);
    }

    auto fs = pass->getFragmentProgramParameters();
    fs->setNamedConstant("RT", 0);
    fs->setNamedConstant("depthTexture", 1);
    fs->setNamedConstant("surfaceZ", params_.surfaceZ);
    fs->setNamedConstant("attenuation", ToOgre(params_.attenuation));
    fs->setNamedConstant("backscatter", ToOgre(params_.backscatter));
    fs->setNamedConstant("downwelling", ToOgre(params_.downwelling));
    material_->load();
  }

  Params params_;
  Ogre::SceneManager *sceneMgr_;
  Ogre::Camera *depthCam_ = nullptr;
  Ogre::MaterialPtr material_;
};
}  // namespace

class UnderwaterTurbidity : public gz::sim::System, public gz::sim::ISystemConfigure
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &_events) override
  {
    // Explicit list: sensors like the DVL render through plain cameras too,
    // and hazing those would corrupt their output.
    for (auto e = _sdf->FindElement("camera"); e; e = e->GetNextElement("camera"))
      cameras_.insert(e->Get<std::string>());
    if (cameras_.empty()) {
      gzerr << "[UnderwaterTurbidity] no <camera> given\n";
      return;
    }
    params_.surfaceZ = _sdf->Get<float>("surface_z", params_.surfaceZ).first;
    params_.attenuation = _sdf->Get<gz::math::Vector3d>("attenuation", params_.attenuation).first;
    params_.backscatter = _sdf->Get<gz::math::Vector3d>("backscatter", params_.backscatter).first;
    params_.downwelling = _sdf->Get<gz::math::Vector3d>("downwelling", params_.downwelling).first;
    params_.minRange = _sdf->Get<float>("min_range", params_.minRange).first;

    // Emitted by the Sensors system on its render thread, where Ogre calls are legal.
    preRender_ = _events.Connect<gz::sim::events::PreRender>([this] { OnPreRender(); });
  }

private:
  void OnPreRender()
  {
    if (!scene_) {
      scene_ = gz::rendering::sceneFromFirstRenderEngine();
      if (!scene_)
        return;
      if (scene_->Engine()->Name() != "ogre2") {
        gzerr << "[UnderwaterTurbidity] needs the ogre2 render engine, got "
              << scene_->Engine()->Name() << "\n";
        preRender_.reset();
        return;
      }
    }

    // Cameras appear whenever a model with sensors spawns, so keep looking.
    // Rendering names are scoped (model::link::sensor); match the last part.
    for (unsigned i = 0; i < scene_->SensorCount(); ++i) {
      auto cam = std::dynamic_pointer_cast<gz::rendering::Ogre2Camera>(scene_->SensorByIndex(i));
      if (!cam)
        continue;
      const std::string &name = cam->Name();
      const auto sep = name.rfind("::");
      if (!cameras_.count(sep == std::string::npos ? name : name.substr(sep + 2)) ||
          !done_.insert(cam->Id()).second)
        continue;
      auto *sceneMgr = std::dynamic_pointer_cast<gz::rendering::Ogre2Scene>(scene_)->OgreSceneManager();
      cam->AddRenderPass(std::make_shared<TurbidityPass>(params_, sceneMgr));
      gzmsg << "[UnderwaterTurbidity] applied to camera " << cam->Name() << "\n";
    }
  }

  Params params_;
  std::set<std::string> cameras_;
  gz::rendering::ScenePtr scene_;
  std::set<unsigned> done_;
  gz::common::ConnectionPtr preRender_;
};
}  // namespace banyu

GZ_ADD_PLUGIN(banyu::UnderwaterTurbidity, gz::sim::System,
              banyu::UnderwaterTurbidity::ISystemConfigure)
