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

#include "GzSceneManager.hh"

#include <ignition/common/Profiler.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/rendering/RenderingIface.hh>
#include <ignition/rendering/Scene.hh>

#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/rendering/RenderUtil.hh"

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  /// \brief Private data class for GzSceneManager
  class GzSceneManagerPrivate
  {
    /// \brief Update the 3D scene based on the latest state of the ECM.
    public: void OnRender();

    //// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene;

    /// \brief Name of the world
    public: std::string worldName;

    /// \brief Rendering utility
    public: RenderUtil renderUtil;
  };
}
}
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
GzSceneManager::GzSceneManager()
  : GuiSystem(), dataPtr(std::make_unique<GzSceneManagerPrivate>)
{
}

/////////////////////////////////////////////////
GzSceneManager::~GzSceneManager() = default;

/////////////////////////////////////////////////
void GzSceneManager::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Scene Manager";

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void GzSceneManager::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("GzSceneManager::Update");
  if (this->dataPtr->worldName.empty())
  {
    // TODO(anyone) Only one scene is supported for now
    Entity worldEntity;
    _ecm.Each<components::World, components::Name>(
        [&](const Entity &_entity,
          const components::World * /* _world */ ,
          const components::Name *_name)->bool
        {
          this->dataPtr->worldName = _name->Data();
          worldEntity = _entity;
          return false;
        });
  }

  this->dataPtr->renderUtil.UpdateECM(_info, _ecm);
  this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);
}

/////////////////////////////////////////////////
bool GzSceneManager::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void GzSceneManagerPrivate::OnRender()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::someInitializedScene();
    if (nullptr == this->scene)
      return;

    this->renderUtil.SetScene(this->scene);
  }

  this->renderUtil.Update();
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::GzSceneManager,
                    ignition::gui::Plugin)
