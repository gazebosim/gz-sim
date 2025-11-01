/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <memory>
#include <utility>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/serialized_map.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/world_control_state.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/fuel_tools/Interface.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/transport/Node.hh>

// Include all components so they have first-class support
#include "gz/sim/components/components.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/EntityComponentManager.hh"
#include <gz/sim/gui/GuiEvents.hh>
#include "gz/sim/gui/GuiSystem.hh"
#include "gz/sim/SystemLoader.hh"

#include "GuiRunner.hh"

using namespace gz;
using namespace sim;

// Register SerializedStepMap to the Qt meta type system so we can pass objects
// of this type in QMetaObject::invokeMethod
Q_DECLARE_METATYPE(msgs::SerializedStepMap)

/////////////////////////////////////////////////
class gz::sim::GuiRunner::Implementation
{
  /// \brief Entity-component manager.
  public: sim::EntityComponentManager ecm;

  /// \brief Transport node.
  public: transport::Node node{};

  /// \brief Topic to request state
  public: std::string stateTopic;

  /// \brief Latest update info
  public: UpdateInfo updateInfo;

  /// \brief Flag used to end the updateThread.
  public: bool running{false};

  /// \brief Mutex to protect the plugin update.
  public: std::mutex updateMutex;

  /// \brief The plugin update thread..
  public: std::thread updateThread;

  /// \brief True if the initial state has been received and processed.
  public: std::atomic<bool> receivedInitialState{false};

  /// \brief Name of WorldControl service
  public: std::string controlService;

  /// \brief System loader for loading gz-sim systems
  public: std::unique_ptr<SystemLoader> systemLoader;

  /// \brief Mutex to protect systemLoader
  public: std::mutex systemLoadMutex;

  /// \brief Events containing visual plugins to load
  public: std::vector<std::pair<gz::sim::Entity, sdf::Plugin>>
      visualPlugins;

  /// \brief Systems implementing PreUpdate
  public: std::vector<SystemPluginPtr> systems;

  /// \brief Systems implementing PreUpdate
  public: std::vector<ISystemPreUpdate *> systemsPreupdate;

  /// \brief Systems implementing Update
  public: std::vector<ISystemUpdate *> systemsUpdate;

  /// \brief Systems implementing PostUpdate
  public: std::vector<ISystemPostUpdate *> systemsPostupdate;

  /// \brief Manager of all events.
  public: EventManager eventMgr;
};

/////////////////////////////////////////////////
GuiRunner::GuiRunner(const std::string &_worldName)
  : dataPtr(utils::MakeUniqueImpl<Implementation>())
{
  qRegisterMetaType<msgs::SerializedStepMap>();

  this->setProperty("worldName", QString::fromStdString(_worldName));

  // Allow for creation of entities on GUI side.
  // Note we have to start the entity id at an offset so it does not conflict
  // with the ones on the server. The log playback starts at max 64bit int / 2
  // On the gui side, we will start entity id at an offset of max 32bit int / 4
  // because currently many plugins cast entity ids to a 32 bit signed/unsigned
  // int.
  // todo(anyone) fix all gui plugins to use 64bit unsigned int for Entity ids
  // and add support for accepting uint64_t data in gz-rendering Node's
  // UserData object.
  // todo(anyone) address
  // https://github.com/gazebosim/gz-sim/issues/1134
  // so that an offset is not required
  this->dataPtr->ecm.SetEntityCreateOffset(math::MAX_I32 / 2);

  auto win = gz::gui::App()->findChild<gz::gui::MainWindow *>();
  auto winWorldNames = win->property("worldNames").toStringList();
  winWorldNames.append(QString::fromStdString(_worldName));
  win->setProperty("worldNames", winWorldNames);

  this->dataPtr->stateTopic = transport::TopicUtils::AsValidTopic("/world/" +
      _worldName + "/state");
  if (this->dataPtr->stateTopic.empty())
  {
    gzerr << "Failed to generate valid topic for world [" << _worldName << "]"
           << std::endl;
    return;
  }

  common::addFindFileURICallback([] (common::URI _uri)
  {
    return fuel_tools::fetchResource(_uri.Str());
  });

  gzdbg << "Requesting initial state from [" << this->dataPtr->stateTopic
         << "]..." << std::endl;

  this->RequestState();

  // Periodically update the plugins
  QPointer<QTimer> timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &GuiRunner::UpdatePlugins);
  timer->start(33);

  this->dataPtr->controlService = "/world/" + _worldName + "/control/state";

  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
GuiRunner::~GuiRunner() = default;

/////////////////////////////////////////////////
bool GuiRunner::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gz::gui::events::WorldControl::kType)
  {
    auto worldControlEvent =
      reinterpret_cast<gz::gui::events::WorldControl *>(_event);
    if (worldControlEvent)
    {
      msgs::WorldControlState req;
      req.mutable_world_control()->CopyFrom(
          worldControlEvent->WorldControlInfo());

      // share the GUI's ECM with the server if:
      //  1. Play was pressed
      //  2. Step was pressed while paused
      const auto &info = worldControlEvent->WorldControlInfo();
      const bool pressedStep = info.multi_step() > 0u;
      const bool pressedPlay = !info.pause() && !pressedStep;
      const bool pressedStepWhilePaused = info.pause() && pressedStep;
      if (pressedPlay || pressedStepWhilePaused)
        req.mutable_state()->CopyFrom(this->dataPtr->ecm.State());

      std::function<void(const gz::msgs::Boolean &, const bool)> cb =
          [](const gz::msgs::Boolean &/*_rep*/, const bool _result)
          {
            if (!_result)
              gzerr << "Error sharing WorldControl info with the server.\n";
          };
      this->dataPtr->node.Request(this->dataPtr->controlService, req, cb);
    }
  }
  else if (_event->type() ==
      gz::sim::gui::events::VisualPlugins::kType)
  {
    auto visualPluginEvent =
      reinterpret_cast<gui::events::VisualPlugins *>(_event);
    if (visualPluginEvent)
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->systemLoadMutex);

      Entity entity = visualPluginEvent->Entity();
      for (const sdf::Plugin &plugin : visualPluginEvent->Plugins())
      {
        this->dataPtr->visualPlugins.push_back(std::make_pair(entity, plugin));
      }
    }
  }
  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void GuiRunner::RequestState()
{
  // set up service for async state response callback
  std::string id = std::to_string(gz::gui::App()->applicationPid());
  std::string reqSrv =
      this->dataPtr->node.Options().NameSpace() + "/" + id + "/state_async";
  auto reqSrvValid = transport::TopicUtils::AsValidTopic(reqSrv);
  if (reqSrvValid.empty())
  {
    gzerr << "Failed to generate valid service [" << reqSrv << "]"
           << std::endl;
    return;
  }
  reqSrv = reqSrvValid;

  auto advertised = this->dataPtr->node.AdvertisedServices();
  if (std::find(advertised.begin(), advertised.end(), reqSrv) ==
      advertised.end())
  {
    if (!this->dataPtr->node.Advertise(reqSrv, &GuiRunner::OnStateAsyncService,
        this))
    {
      gzerr << "Failed to advertise [" << reqSrv << "]" << std::endl;
    }
  }

  msgs::StringMsg req;
  req.set_data(reqSrv);

  // Subscribe to periodic updates.
  this->dataPtr->node.Subscribe(this->dataPtr->stateTopic,
      &GuiRunner::OnState, this);

  // send async state request
  this->dataPtr->node.Request(this->dataPtr->stateTopic + "_async", req);
}

/////////////////////////////////////////////////
void GuiRunner::OnStateAsyncService(const msgs::SerializedStepMap &_res)
{
  // Since this function may be called from a transport thread, we push the
  // OnStateQt function to the queue so that its called from the Qt thread. This
  // ensures that only one thread has access to the ecm and updateInfo
  // variables.
  QMetaObject::invokeMethod(this, "OnStateQt", Qt::QueuedConnection,
                            Q_ARG(msgs::SerializedStepMap, _res));

  // todo(anyone) store reqSrv string in a member variable and use it here
  // and in RequestState()
  std::string id = std::to_string(gz::gui::App()->applicationPid());
  std::string reqSrv =
      this->dataPtr->node.Options().NameSpace() + "/" + id + "/state_async";
  this->dataPtr->node.UnadvertiseSrv(reqSrv);
}

/////////////////////////////////////////////////
void GuiRunner::OnState(const msgs::SerializedStepMap &_msg)
{
  GZ_PROFILE_THREAD_NAME("GuiRunner::OnState");
  GZ_PROFILE("GuiRunner::Update");

  // Only process state updates after initial state has been received.
  if (!this->dataPtr->receivedInitialState)
    return;

  // Since this function may be called from a transport thread, we push the
  // OnStateQt function to the queue so that its called from the Qt thread. This
  // ensures that only one thread has access to the ecm and updateInfo
  // variables.
  QMetaObject::invokeMethod(this, "OnStateQt", Qt::QueuedConnection,
                            Q_ARG(msgs::SerializedStepMap, _msg));
}

/////////////////////////////////////////////////
void GuiRunner::OnStateQt(const msgs::SerializedStepMap &_msg)
{
  GZ_PROFILE_THREAD_NAME("Qt thread");
  GZ_PROFILE("GuiRunner::Update");
  this->dataPtr->ecm.SetState(_msg.state());

  // Update all plugins
  this->dataPtr->updateInfo = convert<UpdateInfo>(_msg.stats());
  this->UpdatePlugins();

  this->dataPtr->receivedInitialState = true;
}

/////////////////////////////////////////////////
void GuiRunner::UpdatePlugins()
{
  // gui plugins
  auto plugins = gz::gui::App()->findChildren<GuiSystem *>();
  for (auto plugin : plugins)
  {
    plugin->Update(this->dataPtr->updateInfo, this->dataPtr->ecm);
  }
  this->dataPtr->ecm.ClearRemovedComponents();
  this->dataPtr->ecm.ClearNewlyCreatedEntities();
  this->dataPtr->ecm.ProcessRemoveEntityRequests();

  // gz-sim systems
  this->LoadSystems();
  this->UpdateSystems();
}

/////////////////////////////////////////////////
void GuiRunner::LoadSystems()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->systemLoadMutex);
  // currently only support systems that are visual plugins
  for (auto &visualPlugin : this->dataPtr->visualPlugins)
  {
    Entity entity = visualPlugin.first;
    sdf::Plugin plugin = visualPlugin.second;
    if (plugin.Filename() != "__default__" && plugin.Name() != "__default__")
    {
      std::optional<SystemPluginPtr> system;
      if (!this->dataPtr->systemLoader)
        this->dataPtr->systemLoader = std::make_unique<SystemLoader>();
      system = this->dataPtr->systemLoader->LoadPlugin(plugin);
      if (system)
      {
        SystemPluginPtr sys = system.value();
        this->dataPtr->systems.push_back(sys);
        this->dataPtr->systemsPreupdate.push_back(
            sys->QueryInterface<ISystemPreUpdate>());
        this->dataPtr->systemsUpdate.push_back(
            sys->QueryInterface<ISystemUpdate>());
        this->dataPtr->systemsPostupdate.push_back(
            sys->QueryInterface<ISystemPostUpdate>());

        auto sysConfigure = sys->QueryInterface<ISystemConfigure>();
        if (sysConfigure)
        {
          sysConfigure->Configure(entity, plugin.ToElement(),
              this->dataPtr->ecm, this->dataPtr->eventMgr);
        }
        gzdbg << "Loaded system [" << plugin.Name()
               << "] for entity [" << entity << "] in GUI"
               << std::endl;
      }
    }
  }
  this->dataPtr->visualPlugins.clear();
}

/////////////////////////////////////////////////
void GuiRunner::UpdateSystems()
{
  GZ_PROFILE("GuiRunner::UpdateSystems");

  {
    GZ_PROFILE("PreUpdate");
    for (auto& system : this->dataPtr->systemsPreupdate)
    {
      if (system)
        system->PreUpdate(this->dataPtr->updateInfo, this->dataPtr->ecm);
    }
  }

  {
    GZ_PROFILE("Update");
    for (auto& system : this->dataPtr->systemsUpdate)
    {
      if (system)
        system->Update(this->dataPtr->updateInfo, this->dataPtr->ecm);
    }
  }

  {
    GZ_PROFILE("PostUpdate");
    // \todo(anyone) Do PostUpdates in parallel
    for (auto& system : this->dataPtr->systemsPostupdate)
    {
      if (system)
        system->PostUpdate(this->dataPtr->updateInfo, this->dataPtr->ecm);
    }
  }
}

/////////////////////////////////////////////////
EventManager &GuiRunner::GuiEventManager() const
{
  return this->dataPtr->eventMgr;
}
