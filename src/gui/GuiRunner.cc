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
class ignition::gazebo::GuiRunnerPrivate
{
  /// \brief Entity-component manager.
  public: gazebo::EntityComponentManager ecm;

  /// \brief Transport node.
  public: transport::Node node;

  /// \brief Topic to request state
  public: std::string stateTopic;

  /// \brief Latest update info
  public: UpdateInfo updateInfo;
};

/////////////////////////////////////////////////
GuiRunner::GuiRunner(const std::string &_worldName)
  : dataPtr(std::make_unique<GuiRunnerPrivate>())
{
  this->setProperty("worldName", QString::fromStdString(_worldName));

  auto win = gui::App()->findChild<ignition::gui::MainWindow *>();
  auto winWorldNames = win->property("worldNames").toStringList();
  winWorldNames.append(QString::fromStdString(_worldName));
  win->setProperty("worldNames", winWorldNames);

  this->dataPtr->stateTopic = "/world/" + _worldName + "/state";

  common::addFindFileURICallback([] (common::URI _uri)
  {
    return fuel_tools::fetchResource(_uri.Str());
  });

  igndbg << "Requesting initial state from [" << this->dataPtr->stateTopic << "]..."
         << std::endl;

  this->RequestState();
}

/////////////////////////////////////////////////
GuiRunner::~GuiRunner() = default;

/////////////////////////////////////////////////
void GuiRunner::RequestState()
{
  // set up service for async state response callback
  std::string id = std::to_string(gui::App()->applicationPid());
  std::string reqSrv =
      this->dataPtr->node.Options().NameSpace() + "/" + id + "/state_async";
  this->dataPtr->node.Advertise(reqSrv, &GuiRunner::OnStateAsyncService, this);
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

  plugin->Update(this->dataPtr->updateInfo, this->dataPtr->ecm);
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

  this->dataPtr->ecm.SetState(_msg.state());

  // Update all plugins
  this->dataPtr->updateInfo = convert<UpdateInfo>(_msg.stats());
  auto plugins = gui::App()->findChildren<GuiSystem *>();
  for (auto plugin : plugins)
  {
    plugin->Update(this->dataPtr->updateInfo, this->dataPtr->ecm);
  }
  this->dataPtr->ecm.ClearNewlyCreatedEntities();
  this->dataPtr->ecm.ProcessRemoveEntityRequests();
  this->dataPtr->ecm.ClearRemovedComponents();
}

