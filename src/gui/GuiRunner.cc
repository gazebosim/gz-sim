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

// Include all components so they have first-class support
#include "ignition/gazebo/components/components.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/gui/GuiRunner.hh"
#include "ignition/gazebo/gui/GuiSystem.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
GuiRunner::GuiRunner(const std::string &_worldName)
{
  this->setProperty("worldName", QString::fromStdString(_worldName));
  this->stateTopic = "/world/" + _worldName + "/state";

  common::addFindFileURICallback([] (common::URI _uri)
  {
    return fuel_tools::fetchResource(_uri.Str());
  });

  igndbg << "Requesting initial state from [" << this->stateTopic << "]..."
         << std::endl;

  this->RequestState();

  // Periodic change updates
  this->node.Subscribe(stateTopic, &GuiRunner::OnState, this);
}

/////////////////////////////////////////////////
GuiRunner::~GuiRunner() = default;

/////////////////////////////////////////////////
void GuiRunner::RequestState()
{
  this->node.Request(this->stateTopic, &GuiRunner::OnStateService, this);
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

  plugin->Update(this->updateInfo, this->ecm);
}

/////////////////////////////////////////////////
void GuiRunner::OnStateService(const msgs::SerializedStepMap &_res,
    const bool _result)
{
  if (!_result)
  {
    ignerr << "Service call failed for [" << this->stateTopic << "]"
           << std::endl;
    return;
  }
  this->OnState(_res);
}

/////////////////////////////////////////////////
void GuiRunner::OnState(const msgs::SerializedStepMap &_msg)
{
  IGN_PROFILE_THREAD_NAME("GuiRunner::OnState");
  IGN_PROFILE("GuiRunner::Update");

  this->ecm.SetState(_msg.state());

  // Update all plugins
  this->updateInfo = convert<UpdateInfo>(_msg.stats());
  auto plugins = gui::App()->findChildren<GuiSystem *>();
  for (auto plugin : plugins)
  {
    plugin->Update(this->updateInfo, this->ecm);
  }
  this->ecm.ClearNewlyCreatedEntities();
  this->ecm.ProcessRemoveEntityRequests();
}

