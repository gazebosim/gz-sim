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
#include <ignition/gazebo/rendering/Events.hh>
//! [includeRenderingEvents]
#include <ignition/math/Rand.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/rendering/RenderEngine.hh>
#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

using namespace std::literals::chrono_literals;

//////////////////////////////////////////////////
void RenderingServerPlugin::Configure(
    const ignition::gazebo::Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    ignition::gazebo::EntityComponentManager &/*_ecm*/,
    ignition::gazebo::EventManager &_eventMgr)
{
//! [connectToServerEvent]
  this->connection = _eventMgr.Connect<ignition::gazebo::events::PreRender>(
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
      static_cast<float>(ignition::math::Rand::DblUniform(0.0, 1.0)),
      static_cast<float>(ignition::math::Rand::DblUniform(0.0, 1.0)),
      static_cast<float>(ignition::math::Rand::DblUniform(0.0, 1.0)),
      1.0});

  this->lastUpdate = this->simTime;
}
//! [performRenderingOperations]

/////////////////////////////////////////////////
//! [findScene]
void RenderingServerPlugin::FindScene()
{
  auto loadedEngNames = ignition::rendering::loadedEngines();
  if (loadedEngNames.empty())
  {
    igndbg << "No rendering engine is loaded yet" << std::endl;
    return;
  }

  // assume there is only one engine loaded
  auto engineName = loadedEngNames[0];
  if (loadedEngNames.size() > 1)
  {
    igndbg << "More than one engine is available. "
      << "Using engine [" << engineName << "]" << std::endl;
  }
  auto engine = ignition::rendering::engine(engineName);
  if (!engine)
  {
    ignerr << "Internal error: failed to load engine [" << engineName
      << "]. Grid plugin won't work." << std::endl;
    return;
  }

  if (engine->SceneCount() == 0)
  {
    igndbg << "No scene has been created yet" << std::endl;
    return;
  }

  // Get first scene
  auto scenePtr = engine->SceneByIndex(0);
  if (nullptr == scenePtr)
  {
    ignerr << "Internal error: scene is null." << std::endl;
    return;
  }

  if (engine->SceneCount() > 1)
  {
    igndbg << "More than one scene is available. "
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
void RenderingServerPlugin::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  this->simTime = _info.simTime;
}

IGNITION_ADD_PLUGIN(
  RenderingServerPlugin,
  ignition::gazebo::System,
  RenderingServerPlugin::ISystemConfigure,
  RenderingServerPlugin::ISystemPreUpdate
)
