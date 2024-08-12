/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <string>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>

#include <gz/rendering/Camera.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/RenderPass.hh>
#include <gz/rendering/LensFlarePass.hh>
#include <gz/rendering/RenderPassSystem.hh>

#include "gz/sim/rendering/RenderUtil.hh"
#include "gz/sim/rendering/Events.hh"

#include "gz/sim/components/Camera.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/Util.hh"

#include "LensFlare.hh"

using namespace gz;
using namespace sim;
using namespace systems;

// Private data class
class gz::sim::systems::LensFlarePrivate
{
  /// \brief Callback invoked in the rendering thread after a render update
  public: void OnPostRender();

  /// \brief function used to connect to render event
  public: void ConnectToPostRender();

  /// \brief Connection to the post-render event
  public: common::ConnectionPtr postRenderConn;

  /// \brief Pointer to the event manager
  public: EventManager *eventMgr = nullptr;

  /// \brief camera entity
  public: Entity entity;

  /// \brief Pointer to the rendering scene
  public: rendering::ScenePtr scene;

  /// \brief Pointer to the rendering camera
  public: rendering::CameraPtr camera;

  /// \brief Name of the camera
  public: std::string cameraName;

  /// \brief Name of the Light
  public: std::string lightName;

  /// \brief Scale of the lens flare
  public: double scale = 1.0;

  /// \brief Color of the lens flare
  public: math::Vector3d color = math::Vector3d{1.4, 1.2, 1.0};

  /// \brief Number of occlusion steps to take in each direction.
  public: uint32_t occlusionSteps = 10;
};


//////////////////////////////////////////////////
LensFlare::LensFlare()
  : System(), dataPtr(std::make_unique<LensFlarePrivate>())
{
}


//////////////////////////////////////////////////
void LensFlare::Configure(
  const Entity &_entity, const std::shared_ptr<const sdf::Element> &_sdf,
  EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  this->dataPtr->entity = _entity;
  this->dataPtr->eventMgr = &_eventMgr;

  gzmsg << "Lens flare attached to camera named "
      << this->dataPtr->cameraName << std::endl;

  if (_sdf->HasElement("scale"))
  {
    this->dataPtr->scale = _sdf->Get<double>("scale");
  }

  if (_sdf->HasElement("color"))
  {
    this->dataPtr->color = _sdf->Get<math::Vector3d>("color");
  }

  if (_sdf->HasElement("occulision_steps"))
  {
    this->dataPtr->occlusionSteps = _sdf->Get<uint32_t>("occlusion_steps");
  }

  if (_sdf->HasElement("light_name"))
  {
    this->dataPtr->lightName = _sdf->Get<std::string>("light_name");
  }

  // Get Camera Name
  this->dataPtr->cameraName = scopedName(this->dataPtr->entity,
                      _ecm, "::", false);

  // call function that connects to post render event
  this->dataPtr->postRenderConn =
    this->dataPtr->eventMgr->Connect<events::PostRender>(
    std::bind(&LensFlarePrivate::OnPostRender, this->dataPtr.get()));
}


//////////////////////////////////////////////////
void LensFlarePrivate::ConnectToPostRender()
{
  // Set up the render connection so we can add LensFlare render pass to
  // the camera in the rendering thread callback
  if (!this->postRenderConn)
  {
      this->postRenderConn =
          this->eventMgr->Connect<events::PostRender>(
          std::bind(&LensFlarePrivate::OnPostRender, this));
  }
}


//////////////////////////////////////////////////
void LensFlarePrivate::OnPostRender()
{
  // get rendering scene
  if (!this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
  }

  // return if scene not ready or no sensors are available
  if (!this->scene->IsInitialized() ||
    this->scene->SensorCount() == 0)
  {
    gzmsg << "Scene is not initialized or sensor count was found 0"
        << std::endl;
  }

  // get camera
  if (!this->camera)
  {
    auto sensor = this->scene->SensorByName(this->cameraName);
    if (!sensor)
    {
      gzerr << "Unable to find sensor: " << this->cameraName
          << std::endl;
      return;
    }

    this->camera = std::dynamic_pointer_cast<rendering::Camera>(sensor);
    if (!this->camera)
    {
      gzerr << "Sensor: " << this->cameraName << " is not a Camera sensor"
          << std::endl;
      return;
    }

    return;
  }

  // get light
  rendering::LightPtr light;
  if (!this->lightName.empty())
  {
    light = this->scene->LightByName(this->lightName);
  }
  else if (this->scene->LightCount() > 0)
  {
    light = this->scene->LightByIndex(0);
  }
  // Light not found. Keep trying in case it's not available in the world yet.
  if (!light)
    return;

  rendering::RenderEngine *engine = this->camera->Scene()->Engine();
  rendering::RenderPassSystemPtr rpSystem = engine->RenderPassSystem();

  if (rpSystem)
  {
    rendering::RenderPassPtr flarePass =
      rpSystem->Create<rendering::LensFlarePass>();
    if (!flarePass)
    {
      gzwarn << "Lens Flare is not supported by the "
          << engine->Name() << std::endl;
      return;
    }

    auto lensFlarePass =
      std::dynamic_pointer_cast<rendering::LensFlarePass>(flarePass);
    lensFlarePass->Init(this->scene);
    lensFlarePass->SetEnabled(true);
    lensFlarePass->SetLight(light);
    lensFlarePass->SetScale(this->scale);
    lensFlarePass->SetColor(this->color);
    lensFlarePass->SetOcclusionSteps(this->occlusionSteps);

    this->camera->AddRenderPass(lensFlarePass);
    gzmsg << "LensFlare Render pass added to the camera" << std::endl;
  }

  // disconnect from render event after adding lens flare pass to prevent
  // unecessary callbacks;
  this->postRenderConn.reset();
  this->scene.reset();
  this->camera.reset();
}

GZ_ADD_PLUGIN(LensFlare,
        System,
        LensFlare::ISystemConfigure)

// Add plugin alias so that we can refer to the plugin without the version
// namespace
GZ_ADD_PLUGIN_ALIAS(LensFlare, "gz::sim::systems::LensFlare")
