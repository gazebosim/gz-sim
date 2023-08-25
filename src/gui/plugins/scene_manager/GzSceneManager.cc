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

#include <map>
#include <set>
#include <string>

#include <QQmlProperty>

#include "../../GuiRunner.hh"
#include "GzSceneManager.hh"

#include <sdf/Element.hh>

#include <gz/common/Profiler.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/SystemPluginInfo.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/gui/GuiEvents.hh"
#include "gz/sim/rendering/RenderUtil.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
  /// \brief Private data class for GzSceneManager
  class GzSceneManagerPrivate
  {
    /// \brief Update the 3D scene based on the latest state of the ECM.
    public: void OnRender();

    //// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene;

    /// \brief Rendering utility
    public: RenderUtil renderUtil;

    /// \brief True if render engine plugins paths are initialized
    public: bool renderEnginePluginPathsInit{false};

    /// \brief List of new entities from a gui event
    public: std::set<Entity> newEntities;

    /// \brief List of removed entities from a gui event
    public: std::set<Entity> removedEntities;

    /// \brief Mutex to protect gui event and system upate call race conditions
    /// for newEntities and removedEntities
    public: std::mutex newRemovedEntityMutex;

    /// \brief Indicates whether initial visual plugins have been loaded or not.
    public: bool initializedVisualPlugins = false;

    /// \brief Whether the plugin was correctly initialized
    public: bool initialized{false};
  };
}
}
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
GzSceneManager::GzSceneManager()
  : GuiSystem(), dataPtr(std::make_unique<GzSceneManagerPrivate>())
{
}

/////////////////////////////////////////////////
GzSceneManager::~GzSceneManager() = default;

/////////////////////////////////////////////////
void GzSceneManager::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Scene Manager";

  static bool done{false};
  if (done)
  {
    std::string msg{"Only one GzSceneManager is supported at a time."};
    gzerr << msg << std::endl;
    QQmlProperty::write(this->PluginItem(), "message",
        QString::fromStdString(msg));
    return;
  }
  done = true;

  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);

  this->dataPtr->initialized = true;
}

//////////////////////////////////////////////////
void GzSceneManager::Update(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  if (!this->dataPtr->initialized)
    return;

  GZ_PROFILE("GzSceneManager::Update");

  if (!this->dataPtr->renderEnginePluginPathsInit)
  {
    this->dataPtr->renderUtil.InitRenderEnginePluginPaths();
    this->dataPtr->renderEnginePluginPathsInit = true;
  }

  this->dataPtr->renderUtil.UpdateECM(_info, _ecm);

  std::lock_guard<std::mutex> lock(this->dataPtr->newRemovedEntityMutex);
  {
    this->dataPtr->renderUtil.CreateVisualsForEntities(_ecm,
        this->dataPtr->newEntities);
    this->dataPtr->newEntities.clear();
  }

  this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);

  // load visual plugin on gui side
  std::map<Entity, sdf::Plugins> plugins;
  if (!this->dataPtr->initializedVisualPlugins)
  {
    _ecm.Each<components::Visual, components::SystemPluginInfo>(
        [&](const Entity &_entity,
            const components::Visual *,
            const components::SystemPluginInfo *_plugins)->bool
    {
      sdf::Plugins convertedPlugins = convert<sdf::Plugins>(_plugins->Data());
      plugins[_entity].insert(plugins[_entity].end(),
          convertedPlugins.begin(), convertedPlugins.end());
      return true;
    });
    this->dataPtr->initializedVisualPlugins = true;
  }
  else
  {
    _ecm.EachNew<components::Visual, components::SystemPluginInfo>(
        [&](const Entity &_entity,
            const components::Visual *,
            const components::SystemPluginInfo *_plugins)->bool
    {
      sdf::Plugins convertedPlugins = convert<sdf::Plugins>(_plugins->Data());
      plugins[_entity].insert(plugins[_entity].end(),
          convertedPlugins.begin(), convertedPlugins.end());
      return true;
    });
  }
  for (const auto &it : plugins)
  {
    // Send the new VisualPlugins event
    gz::sim::gui::events::VisualPlugins visualPluginsEvent(
        it.first, it.second);
    gz::gui::App()->sendEvent(
        gz::gui::App()->findChild<gz::gui::MainWindow *>(),
        &visualPluginsEvent);
  }

  // Emit entities created / removed event for gui::Plugins which don't have
  // direct access to the ECM.
  std::set<Entity> created;
  _ecm.EachNew<components::Name>(
      [&](const Entity &_entity, const components::Name *)->bool
      {
        created.insert(_entity);
        return true;
      });
  std::set<Entity> removed;
  _ecm.EachRemoved<components::Name>(
      [&](const Entity &_entity, const components::Name *)->bool
      {
        removed.insert(_entity);
        return true;
      });

  gz::sim::gui::events::NewRemovedEntities removedEvent(
      created, removed);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &removedEvent);
}

/////////////////////////////////////////////////
bool GzSceneManager::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
  }
  else if (_event->type() ==
           gz::sim::gui::events::GuiNewRemovedEntities::kType)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->newRemovedEntityMutex);
    auto addedRemovedEvent =
        reinterpret_cast<gui::events::GuiNewRemovedEntities *>(_event);
    if (addedRemovedEvent)
    {
      for (auto entity : addedRemovedEvent->NewEntities())
        this->dataPtr->newEntities.insert(entity);

      for (auto entity : addedRemovedEvent->RemovedEntities())
        this->dataPtr->removedEntities.insert(entity);
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void GzSceneManagerPrivate::OnRender()
{
  if (nullptr == this->scene)
  {
    this->scene = rendering::sceneFromFirstRenderEngine();
    if (nullptr == this->scene)
      return;

    this->renderUtil.SetScene(this->scene);

    auto runners = gz::gui::App()->findChildren<GuiRunner *>();
    if (runners.empty() || runners[0] == nullptr)
    {
      gzerr << "Internal error: no GuiRunner found." << std::endl;
    }
    else
    {
      this->renderUtil.SetEventManager(&runners[0]->GuiEventManager());
    }
  }

  this->renderUtil.Update();
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::GzSceneManager,
                    gz::gui::Plugin)
