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
#include <ignition/gui/Application.hh>

#include "ignition/gazebo/gui/GuiRunner.hh"
#include "ignition/gazebo/gui/GuiSystem.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
GuiRunner::GuiRunner(const std::string &_worldName)
{
  this->stateTopic = "/world/" + _worldName + "/state";

  igndbg << "Requesting initial state from [" << this->stateTopic << "]..."
         << std::endl;

  // Blocking request at startup
  bool result{false};
  unsigned int timeout{5000};

  ignition::msgs::SerializedState stateMsg;
  bool executed = this->node.Request(stateTopic, timeout, stateMsg, result);

  if (!executed)
  {
    ignerr << "Service call timed out for [" << stateTopic << "]"
           << std::endl;
  }
  else if (!result)
  {
    ignerr << "Service call failed for [" << stateTopic << "]" << std::endl;
  }
  else
  {
    this->OnState(stateMsg);

    // Periodic change updates
    this->node.Subscribe(stateTopic, &GuiRunner::OnState, this);
  }
}

/////////////////////////////////////////////////
GuiRunner::~GuiRunner()
{
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

  // TODO(louise) get update info
  UpdateInfo updateInfo;
  plugin->Update(updateInfo, this->ecm);
}

/////////////////////////////////////////////////
void GuiRunner::OnState(const msgs::SerializedState &_msg)
{
  this->ecm.SetState(_msg);

  // TODO(louise) get update info from message header
  UpdateInfo updateInfo;

  // Update all plugins
  auto plugins = gui::App()->findChildren<GuiSystem *>();
  for (auto plugin : plugins)
  {
    plugin->Update(updateInfo, this->ecm);
  }
  this->ecm.ClearNewlyCreatedEntities();
  this->ecm.ProcessRemoveEntityRequests();
}

