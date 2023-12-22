/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gz/msgs/param.pb.h>

#include <chrono>
#include <list>
#include <mutex>
#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/ShaderParams.hh>
#include <gz/rendering/Visual.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/SourceFilePath.hh"
#include "gz/sim/rendering/Events.hh"
#include "gz/sim/rendering/RenderUtil.hh"
#include "gz/sim/Util.hh"

#include "Wavefield.hh"
#include "WaveVisual.hh"

using namespace gz;
using namespace maritime;

class maritime::WaveVisual::Implementation
{
  /// \brief Path to vertex shader
  public: std::string vertexShaderUri;

  /// \brief Path to fragment shader
  public: std::string fragmentShaderUri;

  /// \brief Mutex to protect sim time updates.
  public: std::mutex mutex;

  /// \brief Connection to pre-render event callback
  public: common::ConnectionPtr connection{nullptr};

  /// \brief Name of visual this plugin is attached to
  public: std::string visualName;

  /// \brief Pointer to visual
  public: rendering::VisualPtr visual;

  /// \brief Material used by this visual
  public: rendering::MaterialPtr material;

  /// \brief Pointer to scene
  public: rendering::ScenePtr scene;

  /// \brief Entity id of the visual
  public: sim::Entity entity = sim::kNullEntity;

  /// \brief Current sim time
  public: std::chrono::steady_clock::duration currentSimTime;

  /// \brief Path to model
  public: std::string modelPath;

  /// \brief Wavefield for computing wave params
  public: Wavefield wavefield;

  /// \brief Indicate whether the shader params have been set or not
  public: bool paramsSet = false;

  /// \brief Shader param. Recale x and y components of normals.
  public: float rescale = 0.5f;

  /// \brief Shader param. Bump map scale.
  public: math::Vector2d bumpScale = math::Vector2d(25.0, 25.0);

  /// \brief Shader param. Bump map speed in x and y.
  public: math::Vector2d bumpSpeed = math::Vector2d(0.01f, 0.01f);

  /// \brief Shader param. HDR multiplier.
  public: float hdrMultiplier = 0.4f;

  /// \brief Shader param. Fresnel power.
  public: float fresnelPower = 5.0f;

  /// \brief Shader param. Color of shallow water.
  public: math::Color shallowColor = math::Color(0.0f, 0.1f, 0.3f, 1.0f);

  /// \brief Shader param. Color of deep water.
  public: math::Color deepColor = math::Color(0.0f, 0.05f, 0.2f, 1.0f);

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief All rendering operations must happen within this call
  public: void OnUpdate();

  /// \brief Callback for receiving wave field updates.
  /// \param[in] _msg The message containing all the wave field parameters.
  public: void OnWavefield(const msgs::Param &_msg);
};

/////////////////////////////////////////////////
WaveVisual::WaveVisual()
    : System(), dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

/////////////////////////////////////////////////
WaveVisual::~WaveVisual()
{
}

/////////////////////////////////////////////////
void WaveVisual::Configure(const sim::Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               sim::EntityComponentManager &_ecm,
               sim::EventManager &_eventMgr)
{
  GZ_PROFILE("WaveVisual::Configure");
  // Ugly, but needed because the sdf::Element::GetElement is not a const
  // function and _sdf is a const shared pointer to a const sdf::Element.
  auto sdf = const_cast<sdf::Element *>(_sdf.get());

  if (!sdf->HasElement("wavefield"))
  {
    gzerr << "<wavefield> parameter is missing " << std::endl;
    return;
  }
  if (!sdf->HasElement("shader"))
  {
    gzerr << "<shader> parameter is missing " << std::endl;
    return;
  }

  this->dataPtr->wavefield.Load(_sdf);

  if (this->dataPtr->modelPath.empty())
  {
    auto modelEntity = topLevelModel(_entity, _ecm);
    this->dataPtr->modelPath =
      _ecm.ComponentData<sim::components::SourceFilePath>(modelEntity).value();
  }

  // parse path to shaders
  sdf::ElementPtr shaderElem = sdf->GetElement("shader");
  if (!shaderElem->HasElement("vertex") ||
      !shaderElem->HasElement("fragment"))
  {
    gzerr << "<shader> must have <vertex> and <fragment> sdf elements"
          << std::endl;
    return;
  }
  else
  {
    sdf::ElementPtr vertexElem = shaderElem->GetElement("vertex");
    this->dataPtr->vertexShaderUri = common::findFile(sim::asFullPath(
        vertexElem->Get<std::string>(), this->dataPtr->modelPath));
    sdf::ElementPtr fragmentElem = shaderElem->GetElement("fragment");
    this->dataPtr->fragmentShaderUri = common::findFile(sim::asFullPath(
        fragmentElem->Get<std::string>(), this->dataPtr->modelPath));
  }

  // parse shader params
  if (shaderElem->HasElement("parameters"))
  {
    sdf::ElementPtr paramElem = shaderElem->GetElement("parameters");
    if (paramElem->HasElement("rescale"))
    {
      this->dataPtr->rescale = paramElem->GetElement("rescale")->Get<float>();
    }
    if (paramElem->HasElement("bumpScale"))
    {
      this->dataPtr->bumpScale =
          paramElem->GetElement("bumpScale")->Get<math::Vector2d>();
    }
    if (paramElem->HasElement("hdrMultiplier"))
    {
      this->dataPtr->hdrMultiplier =
          paramElem->GetElement("hdrMultiplier")->Get<float>();
    }
    if (paramElem->HasElement("fresnelPower"))
    {
      this->dataPtr->fresnelPower =
          paramElem->GetElement("fresnelPower")->Get<float>();
    }
    if (paramElem->HasElement("shallowColor"))
    {
      this->dataPtr->shallowColor =
          paramElem->GetElement("shallowColor")->Get<math::Color>();
    }
    if (paramElem->HasElement("deepColor"))
    {
      this->dataPtr->deepColor =
          paramElem->GetElement("deepColor")->Get<math::Color>();
    }
  }

  this->dataPtr->entity = _entity;
  auto nameComp = _ecm.Component<sim::components::Name>(_entity);
  this->dataPtr->visualName = nameComp->Data();

  // connect to the SceneUpdate event
  // the callback is executed in the rendering thread so do all
  // rendering operations in that thread
  this->dataPtr->connection =
      _eventMgr.Connect<sim::events::SceneUpdate>(
      std::bind(&WaveVisual::Implementation::OnUpdate, this->dataPtr.get()));

  // Subscribe to receive the wavefield parameters.
  this->dataPtr->node.Subscribe(this->dataPtr->wavefield.Topic(),
    &WaveVisual::Implementation::OnWavefield, this->dataPtr.get());
}

//////////////////////////////////////////////////
void WaveVisual::PreUpdate(
  const sim::UpdateInfo &_info,
  sim::EntityComponentManager &)
{
  GZ_PROFILE("WaveVisual::PreUpdate");
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->currentSimTime = _info.simTime;
}

//////////////////////////////////////////////////
void WaveVisual::Implementation::OnUpdate()
{
  if (this->visualName.empty())
    return;

  if (!this->scene)
    this->scene = rendering::sceneFromFirstRenderEngine();

  if (!this->scene)
    return;

  if (!this->visual)
  {
    // this does a breadth first search for visual with the entity id
    // \todo(anyone) provide a helper function in RenderUtil to search for
    // visual by entity id?
    auto rootVis = scene->RootVisual();
    std::list<rendering::NodePtr> nodes;
    nodes.push_back(rootVis);
    while (!nodes.empty())
    {
      auto n = nodes.front();
      nodes.pop_front();
      if (n && n->HasUserData("gazebo-entity"))
      {
        // RenderUti stores gazebo-entity user data as int
        // \todo(anyone) Change this to uint64_t in Gazebo H?
        auto variant = n->UserData("gazebo-entity");
        const uint64_t *value = std::get_if<uint64_t>(&variant);
        if (value && *value == static_cast<uint64_t>(this->entity))
        {
          this->visual = std::dynamic_pointer_cast<rendering::Visual>(n);
          break;
        }
      }
      for (unsigned int i = 0; i < n->ChildCount(); ++i)
        nodes.push_back(n->ChildByIndex(i));
    }
  }

  if (!this->visual)
    return;

  // get the material and set shaders
  if (!this->material)
  {
    auto mat = scene->CreateMaterial();
    mat->SetVertexShader(this->vertexShaderUri);
    mat->SetFragmentShader(this->fragmentShaderUri);
    this->visual->SetMaterial(mat);
    scene->DestroyMaterial(mat);
    this->material = this->visual->Material();
  }

  if (!this->material)
    return;

  std::lock_guard<std::mutex> lock(this->mutex);

  // params that only need to be set once
  if (!this->paramsSet)
  {
    auto vsParams = this->material->VertexShaderParams();

    (*vsParams)["worldviewproj_matrix"] = 1;

    // bump map parameters
    // rescale - scale x and y components of normals
    (*vsParams)["rescale"] = this->rescale;

    // size of bump map
    float bumpScaleV[2] = {
        static_cast<float>(this->bumpScale.X()),
        static_cast<float>(this->bumpScale.Y())};
    (*vsParams)["bumpScale"].InitializeBuffer(2);
    (*vsParams)["bumpScale"].UpdateBuffer(bumpScaleV);

    // bump map speed in x and y
    float bumpSpeedV[2] = {
        static_cast<float>(this->bumpSpeed.X()),
        static_cast<float>(this->bumpSpeed.Y())};
    (*vsParams)["bumpSpeed"].InitializeBuffer(2);
    (*vsParams)["bumpSpeed"].UpdateBuffer(bumpSpeedV);

    // wavefield parameters:
    (*vsParams)["Nwaves"] = static_cast<int>(this->wavefield.Number());
    float amplitudeV[3] = {
        static_cast<float>(this->wavefield.Amplitude_V()[0]),
        static_cast<float>(this->wavefield.Amplitude_V()[1]),
        static_cast<float>(this->wavefield.Amplitude_V()[2])};
    (*vsParams)["amplitude"].InitializeBuffer(3);
    (*vsParams)["amplitude"].UpdateBuffer(amplitudeV);

    float wavenumberV[3] = {
        static_cast<float>(this->wavefield.Wavenumber_V()[0]),
        static_cast<float>(this->wavefield.Wavenumber_V()[1]),
        static_cast<float>(this->wavefield.Wavenumber_V()[2])};
    (*vsParams)["wavenumber"].InitializeBuffer(3);
    (*vsParams)["wavenumber"].UpdateBuffer(wavenumberV);

    float omegaV[3] = {
        static_cast<float>(this->wavefield.AngularFrequency_V()[0]),
        static_cast<float>(this->wavefield.AngularFrequency_V()[1]),
        static_cast<float>(this->wavefield.AngularFrequency_V()[2])};
    (*vsParams)["omega"].InitializeBuffer(3);
    (*vsParams)["omega"].UpdateBuffer(omegaV);

    auto directions0 = this->wavefield.Direction_V()[0];
    float dir0[2] = {
        static_cast<float>(directions0.X()),
        static_cast<float>(directions0.Y())};
    (*vsParams)["dir0"].InitializeBuffer(2);
    (*vsParams)["dir0"].UpdateBuffer(dir0);

    auto directions1 = this->wavefield.Direction_V()[1];
    float dir1[2] = {
        static_cast<float>(directions1.X()),
        static_cast<float>(directions1.Y())};
    (*vsParams)["dir1"].InitializeBuffer(2);
    (*vsParams)["dir1"].UpdateBuffer(dir1);

    auto directions2 = this->wavefield.Direction_V()[2];
    float dir2[2] = {
        static_cast<float>(directions2.X()),
        static_cast<float>(directions2.Y())};
    (*vsParams)["dir2"].InitializeBuffer(2);
    (*vsParams)["dir2"].UpdateBuffer(dir2);

    float steepnessV[3] = {
        static_cast<float>(this->wavefield.Steepness_V()[0]),
        static_cast<float>(this->wavefield.Steepness_V()[1]),
        static_cast<float>(this->wavefield.Steepness_V()[2])};
    (*vsParams)["steepness"].InitializeBuffer(3);
    (*vsParams)["steepness"].UpdateBuffer(steepnessV);

    float tau = this->wavefield.Tau();
    (*vsParams)["tau"] = tau;

    // camera_position_object_space is a constant defined by ogre.
    (*vsParams)["camera_position_object_space"] = 1;

    // set fragment shader params
    auto fsParams = this->material->FragmentShaderParams();

    // HDR effect
    (*fsParams)["hdrMultiplier"] = this->hdrMultiplier;

    // Fresnel power - refraction
    (*fsParams)["fresnelPower"] = this->fresnelPower;

    // water color:
    float shallowColorV[4] = {
        static_cast<float>(this->shallowColor.R()),
        static_cast<float>(this->shallowColor.G()),
        static_cast<float>(this->shallowColor.B()),
        static_cast<float>(this->shallowColor.A())};
    (*fsParams)["shallowColor"].InitializeBuffer(4);
    (*fsParams)["shallowColor"].UpdateBuffer(shallowColorV);

    float deepColorV[4] = {
        static_cast<float>(this->deepColor.R()),
        static_cast<float>(this->deepColor.G()),
        static_cast<float>(this->deepColor.B()),
        static_cast<float>(this->deepColor.A())};
    (*fsParams)["deepColor"].InitializeBuffer(4);
    (*fsParams)["deepColor"].UpdateBuffer(deepColorV);

    // \todo(anyone) find a more generic way of getting path to the textures
    // than using hard coded material paths
    std::string bumpMapPath = common::findFile(
        sim::asFullPath("materials/textures/wave_normals.dds",
        this->modelPath));
    (*fsParams)["bumpMap"].SetTexture(bumpMapPath,
        rendering::ShaderParam::ParamType::PARAM_TEXTURE);

    std::string cubeMapPath = common::findFile(
      sim::asFullPath("materials/textures/skybox_lowres.dds", this->modelPath));
    (*fsParams)["cubeMap"].SetTexture(cubeMapPath,
        rendering::ShaderParam::ParamType::PARAM_TEXTURE_CUBE, 1u);
    this->paramsSet = true;
  }

  // time variables need to be updated every iteration
  {
    float floatValue = (std::chrono::duration_cast<std::chrono::nanoseconds>(
        this->currentSimTime).count()) * 1e-9;
    rendering::ShaderParamsPtr params;
    params = this->material->VertexShaderParams();
    (*params)["t"] = floatValue;
  }
}

//////////////////////////////////////////////////
void WaveVisual::Implementation::OnWavefield(const msgs::Param &_msg)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->wavefield.Load(_msg);
}

GZ_ADD_PLUGIN(maritime::WaveVisual,
              sim::System,
              WaveVisual::ISystemConfigure,
              WaveVisual::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(WaveVisual, "maritime::WaveVisual")
