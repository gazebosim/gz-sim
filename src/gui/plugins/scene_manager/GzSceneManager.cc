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

#include "ignition/gazebo/Events.hh"
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

    /// \brief This method is used to connect with the event
    /// events::EnableSensors. It will set if the simulation is running any
    /// sensors
    /// \param[in] _enable True if the sensors thread is enabled, false
    /// otherwise
    public: void EnableSensors(bool _enable);

    //// \brief Pointer to the rendering scene
    public: rendering::ScenePtr scene;

    /// \brief Event Manager
    public: EventManager *eventManager{nullptr};

    /// \brief Is the simulation running the GUI and server in the same process
    public: bool sameProcess{false};

    /// \brief is the sensors system plugin running ?
    public: bool enableSensors{false};

    /// \brief did the first render event occur?
    public: bool emitFirstRender{false};

    /// \brief Track connection to "EnableSensors" Event
    public: ignition::common::ConnectionPtr enableSensorsConn;

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
  : GuiSystem(), dataPtr(std::make_unique<GzSceneManagerPrivate>())
{
}

/////////////////////////////////////////////////
GzSceneManager::~GzSceneManager()
{
  this->dataPtr->eventManager = nullptr;
  this->dataPtr->enableSensorsConn = nullptr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
void GzSceneManager::Configure(EventManager &_eventMgr, bool _sameProcess)
{
  if (this->dataPtr->eventManager)
    return;

  this->dataPtr->eventManager = &_eventMgr;
  this->dataPtr->sameProcess = _sameProcess;

  this->dataPtr->renderUtil.SetEventManager(_eventMgr);

  if (_sameProcess)
  {
    this->dataPtr->enableSensorsConn =
      _eventMgr.Connect<ignition::gazebo::events::EnableSensors>(
        std::bind(&GzSceneManagerPrivate::EnableSensors, this->dataPtr.get(),
          std::placeholders::_1));
  }
}

/////////////////////////////////////////////////
void GzSceneManager::LoadConfig(const tinyxml2::XMLElement *)
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

  // When we are running with the same process
  //  * has sensors - sensors system calls RenderUtil::Update*
  //  * no sensors  - GzSceneManager calls RenderUtil::Update*
  if (this->dataPtr->emitFirstRender &&
    (!this->dataPtr->sameProcess || !this->dataPtr->enableSensors))
  {
    this->dataPtr->renderUtil.UpdateECM(_info, _ecm);
    this->dataPtr->renderUtil.UpdateFromECM(_info, _ecm);
  }
}

/////////////////////////////////////////////////
bool GzSceneManager::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gui::events::Render::kType)
  {
    this->dataPtr->OnRender();
    if (this->dataPtr->sameProcess)
    {
      this->dataPtr->eventManager->Emit<ignition::gazebo::events::Render>();
    }
    this->dataPtr->emitFirstRender = true;
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void GzSceneManagerPrivate::EnableSensors(bool _enable)
{
  this->enableSensors = _enable;
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
  }

  if (this->emitFirstRender && (!this->sameProcess || !this->enableSensors))
  {
    this->renderUtil.Update();
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::GzSceneManager,
                    ignition::gui::Plugin)
