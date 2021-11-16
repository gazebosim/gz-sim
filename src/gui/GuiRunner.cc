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

#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/fuel_tools/Interface.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/msgs.hh>
#include <ignition/transport/Node.hh>

// Include all components so they have first-class support
#include "ignition/gazebo/components/components.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiSystem.hh"

#include "GuiRunner.hh"

using namespace ignition;
using namespace gazebo;

// Register SerializedStepMap to the Qt meta type system so we can pass objects
// of this type in QMetaObject::invokeMethod
Q_DECLARE_METATYPE(msgs::SerializedStepMap)

/////////////////////////////////////////////////
class ignition::gazebo::GuiRunner::Implementation
{
  /// \brief Entity-component manager.
  public: gazebo::EntityComponentManager ecm;

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
  public: bool receivedInitialState{false};

  /// \brief Name of WorldControl service
  public: std::string controlService;
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
  // and add support for accepting uint64_t data in ign-rendering Node's
  // UserData object.
  // todo(anyone) address
  // https://github.com/ignitionrobotics/ign-gazebo/issues/1134
  // so that an offset is not required
  this->dataPtr->ecm.SetEntityCreateOffset(math::MAX_I32 / 2);

  auto win = gui::App()->findChild<ignition::gui::MainWindow *>();
  auto winWorldNames = win->property("worldNames").toStringList();
  winWorldNames.append(QString::fromStdString(_worldName));
  win->setProperty("worldNames", winWorldNames);

  this->dataPtr->stateTopic = transport::TopicUtils::AsValidTopic("/world/" +
      _worldName + "/state");
  if (this->dataPtr->stateTopic.empty())
  {
    ignerr << "Failed to generate valid topic for world [" << _worldName << "]"
           << std::endl;
    return;
  }

  common::addFindFileURICallback([] (common::URI _uri)
  {
    return fuel_tools::fetchResource(_uri.Str());
  });

  igndbg << "Requesting initial state from [" << this->dataPtr->stateTopic
         << "]..." << std::endl;

  this->RequestState();

  // Periodically update the plugins
  QPointer<QTimer> timer = new QTimer(this);
  connect(timer, &QTimer::timeout, this, &GuiRunner::UpdatePlugins);
  timer->start(33);

  this->dataPtr->controlService = "/world/" + _worldName + "/control/state";

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
GuiRunner::~GuiRunner() = default;

/////////////////////////////////////////////////
bool GuiRunner::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gui::events::WorldControl::kType)
  {
    auto worldControlEvent =
      reinterpret_cast<gui::events::WorldControl *>(_event);
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

      std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
          [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
          {
            if (!_result)
              ignerr << "Error sharing WorldControl info with the server.\n";
          };
      this->dataPtr->node.Request(this->dataPtr->controlService, req, cb);
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
void GuiRunner::RequestState()
{
  // set up service for async state response callback
  std::string id = std::to_string(gui::App()->applicationPid());
  std::string reqSrv =
      this->dataPtr->node.Options().NameSpace() + "/" + id + "/state_async";
  auto reqSrvValid = transport::TopicUtils::AsValidTopic(reqSrv);
  if (reqSrvValid.empty())
  {
    ignerr << "Failed to generate valid service [" << reqSrv << "]"
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
      ignerr << "Failed to advertise [" << reqSrv << "]" << std::endl;
    }
  }

  ignition::msgs::StringMsg req;
  req.set_data(reqSrv);

  // Subscribe to periodic updates.
  this->dataPtr->node.Subscribe(this->dataPtr->stateTopic,
      &GuiRunner::OnState, this);

  // send async state request
  this->dataPtr->node.Request(this->dataPtr->stateTopic + "_async", req);
}

/////////////////////////////////////////////////
void GuiRunner::OnPluginAdded(const QString &)
{
  // This function used to call Update on the plugin, but that's no longer
  // necessary. The function is left here for ABI compatibility.
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
  this->dataPtr->receivedInitialState = true;

  // todo(anyone) store reqSrv string in a member variable and use it here
  // and in RequestState()
  std::string id = std::to_string(gui::App()->applicationPid());
  std::string reqSrv =
      this->dataPtr->node.Options().NameSpace() + "/" + id + "/state_async";
  this->dataPtr->node.UnadvertiseSrv(reqSrv);
}

/////////////////////////////////////////////////
void GuiRunner::OnState(const msgs::SerializedStepMap &_msg)
{
  IGN_PROFILE_THREAD_NAME("GuiRunner::OnState");
  IGN_PROFILE("GuiRunner::Update");

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
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("GuiRunner::Update");
  this->dataPtr->ecm.SetState(_msg.state());

  // Update all plugins
  this->dataPtr->updateInfo = convert<UpdateInfo>(_msg.stats());
  this->UpdatePlugins();
}

/////////////////////////////////////////////////
void GuiRunner::UpdatePlugins()
{
  auto plugins = gui::App()->findChildren<GuiSystem *>();
  for (auto plugin : plugins)
  {
    plugin->Update(this->dataPtr->updateInfo, this->dataPtr->ecm);
  }
  this->dataPtr->ecm.ClearRemovedComponents();
  this->dataPtr->ecm.ClearNewlyCreatedEntities();
  this->dataPtr->ecm.ProcessRemoveEntityRequests();
}
