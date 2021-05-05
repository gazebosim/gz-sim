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
#include <ignition/gui/MainWindow.hh>
#include <ignition/transport/Node.hh>

// Include all components so they have first-class support
#include "ignition/gazebo/components/components.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiRunner.hh"
#include "ignition/gazebo/gui/GuiSystem.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
class ignition::gazebo::GuiRunner::Implementation
{

  public: explicit Implementation(gazebo::EntityComponentManager &_ecm)
  : ecm(_ecm){}

  /// \brief Update the plugins.
  public: void UpdatePlugins();

  /// \brief Entity-component manager.
  public: gazebo::EntityComponentManager &ecm;

  public: bool sameProcess;

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
};

/////////////////////////////////////////////////
GuiRunner::GuiRunner(const std::string &_worldName,
  EntityComponentManager &_ecm, bool _sameProcess)
  : dataPtr(utils::MakeUniqueImpl<Implementation>(_ecm))
{
  this->dataPtr->sameProcess = _sameProcess;

  this->setProperty("worldName", QString::fromStdString(_worldName));
  this->setProperty("sameProcess",
    QString::fromStdString(std::to_string(_sameProcess)));

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
  // \todo(anyone) Move the global variables to GuiRunner::Implementation on v5
  this->dataPtr->running = true;
  this->dataPtr->updateThread = std::thread([&]()
  {
    while (this->dataPtr->running)
    {
      {
        std::lock_guard<std::mutex> lock(this->dataPtr->updateMutex);
        this->dataPtr->UpdatePlugins();
      }
      // This is roughly a 30Hz update rate.
      std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
  });
}

/////////////////////////////////////////////////
GuiRunner::~GuiRunner()
{
  this->dataPtr->running = false;
  if (this->dataPtr->updateThread.joinable())
    this->dataPtr->updateThread.join();
}

/////////////////////////////////////////////////
void GuiRunner::RequestState()
{
  if (this->dataPtr->sameProcess)
    return;

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

  // send async state request
  this->dataPtr->node.Request(this->dataPtr->stateTopic + "_async", req);
}

/////////////////////////////////////////////////
void GuiRunner::OnPluginAdded(const QString &_objectName)
{
  auto plugin = gui::App()->findChild<GuiSystem *>(_objectName);
  if (!plugin)
  {
    ignerr << "Failed to get plugin [" << _objectName.toStdString()
           << "]" << std::endl;
    return;
  }

  this->RequestState();
}

/////////////////////////////////////////////////
void GuiRunner::OnStateAsyncService(const msgs::SerializedStepMap &_res)
{
  this->OnState(_res);

  // todo(anyone) store reqSrv string in a member variable and use it here
  // and in RequestState()
  std::string id = std::to_string(gui::App()->applicationPid());
  std::string reqSrv =
      this->dataPtr->node.Options().NameSpace() + "/" + id + "/state_async";
  this->dataPtr->node.UnadvertiseSrv(reqSrv);

  // Only subscribe to periodic updates after receiving initial state
  if (this->dataPtr->node.SubscribedTopics().empty())
  {
    this->dataPtr->node.Subscribe(this->dataPtr->stateTopic,
        &GuiRunner::OnState, this);
  }
}

/////////////////////////////////////////////////
void GuiRunner::OnState(const msgs::SerializedStepMap &_msg)
{
  IGN_PROFILE_THREAD_NAME("GuiRunner::OnState");
  IGN_PROFILE("GuiRunner::Update");

  std::lock_guard<std::mutex> lock(this->dataPtr->updateMutex);
  this->dataPtr->ecm.SetState(_msg.state());

  // Update all plugins
  this->dataPtr->updateInfo = convert<UpdateInfo>(_msg.stats());
  this->dataPtr->UpdatePlugins();
  this->dataPtr->ecm.ClearNewlyCreatedEntities();
  this->dataPtr->ecm.ProcessRemoveEntityRequests();
}

/////////////////////////////////////////////////
void GuiRunner::Implementation::UpdatePlugins()
{
  auto plugins = gui::App()->findChildren<GuiSystem *>();
  for (auto plugin : plugins)
  {
    plugin->Update(this->updateInfo, this->ecm);
  }
  this->ecm.ClearRemovedComponents();
}
