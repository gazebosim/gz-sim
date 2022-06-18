/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include "RenderingServerPlugin.hh"

//! [includeRenderingEvents]
#include <gz/sim/rendering/Events.hh>
//! [includeRenderingEvents]
#include <gz/math/Rand.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

using namespace std::literals::chrono_literals;

//////////////////////////////////////////////////
void RenderingServerPlugin::Configure(
    const gz::sim::Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &_eventMgr)
{
//! [connectToServerEvent]
  this->connection = _eventMgr.Connect<gz::sim::events::PreRender>(
    std::bind(&RenderingServerPlugin::PerformRenderingOperations, this));
//! [connectToServerEvent]
}

//////////////////////////////////////////////////
//! [performRenderingOperations]
void RenderingServerPlugin::PerformRenderingOperations()
{
  if (nullptr == this->scene)
  {
    this->FindScene();
  }

  if (nullptr == this->scene)
    return;

  if (this->simTime - this->lastUpdate < 2s)
    return;

  this->scene->SetAmbientLight({
      static_cast<float>(gz::math::Rand::DblUniform(0.0, 1.0)),
      static_cast<float>(gz::math::Rand::DblUniform(0.0, 1.0)),
      static_cast<float>(gz::math::Rand::DblUniform(0.0, 1.0)),
      1.0});

  this->lastUpdate = this->simTime;
}
//! [performRenderingOperations]

/////////////////////////////////////////////////
//! [findScene]
void RenderingServerPlugin::FindScene()
{
  auto loadedEngNames = gz::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    gzdbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    gzdbg << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = gz::rendering::engine(engineName);
  if (!engine)
  {
    gzerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    gzdbg << "No scene has been created yet" << std::endl;
    return;
  }

  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    gzerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (engine->SceneCount() > 1)
  {
    gzdbg << "More than one scene is available. "
      << "Using scene [" << scene->Name() << "]" << std::endl;
  }

  if (!scenePtr->IsInitialized() || nullptr == scenePtr->RootVisual())
  {
    return;
  }

  this->scene = scenePtr;
}
//! [findScene]

//////////////////////////////////////////////////
void RenderingServerPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  this->simTime = _info.simTime;
}

GZ_ADD_PLUGIN(
  RenderingServerPlugin,
  gz::sim::System,
  RenderingServerPlugin::ISystemConfigure,
  RenderingServerPlugin::ISystemPreUpdate
)
