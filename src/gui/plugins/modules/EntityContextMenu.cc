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

#include "EntityContextMenu.hh"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/entity.pb.h>

#include <iostream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/gazebo/gui/GuiRunner.hh>
#include <ignition/gazebo/Conversions.hh>
#include <ignition/gui/Application.hh>
#include <ignition/transport/Node.hh>

namespace ignition::gazebo
{
  /// \brief Private data class for EntityContextMenu
  class EntityContextMenuPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Move to service name
    public: std::string moveToService;

    /// \brief Follow service name
    public: std::string followService;

    /// \brief Remove service name
    public: std::string removeService;

    /// \brief View collisions service name
    public: std::string viewCollisionsService;

    /// \brief Name of world.
    public: std::string worldName;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
void IgnGazeboPlugin::registerTypes(const char *_uri)
{
  // Register our 'EntityContextMenuItem' in qml engine
  qmlRegisterType<ignition::gazebo::EntityContextMenu>(_uri, 1, 0,
      "EntityContextMenuItem");
}

/////////////////////////////////////////////////
EntityContextMenu::EntityContextMenu()
  : dataPtr(std::make_unique<EntityContextMenuPrivate>())
{
  // For move to service requests
  this->dataPtr->moveToService = "/gui/move_to";

  // For follow service requests
  this->dataPtr->followService = "/gui/follow";

  // For remove service requests
  this->dataPtr->removeService = "/world/default/remove";

  // For view collisions service requests
  this->dataPtr->viewCollisionsService = "/gui/view/collisions";
}

/////////////////////////////////////////////////
EntityContextMenu::~EntityContextMenu() = default;

/////////////////////////////////////////////////
void EntityContextMenu::OnRemove(
  const QString &_data, const QString &_type)
{
  if (this->dataPtr->worldName.empty())
  {
    auto runners = gui::App()->findChildren<GuiRunner *>();
    if (runners.empty() || runners[0] == nullptr)
    {
      ignerr << "Internal error: no GuiRunner found." << std::endl;
      return;
    }

    this->dataPtr->worldName = "default";
    auto worldNameVariant = runners[0]->property("worldName");
    if (!worldNameVariant.isValid())
    {
      ignwarn << "GuiRunner's worldName not set, using["
              << this->dataPtr->worldName << "]" << std::endl;
    }
    else
    {
      this->dataPtr->worldName = worldNameVariant.toString().toStdString();
    }

    this->dataPtr->removeService =
      "/world/" + this->dataPtr->worldName + "/remove";
  }

  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &_rep, const bool _result)
  {
    if (!_result || !_rep.data())
      ignerr << "Error sending remove request" << std::endl;
  };

  ignition::msgs::Entity req;
  req.set_name(_data.toStdString());
  req.set_type(convert<msgs::Entity_Type>(_type.toStdString()));

  this->dataPtr->node.Request(this->dataPtr->removeService, req, cb);
}

/////////////////////////////////////////////////
void EntityContextMenu::OnRequest(const QString &_request, const QString &_data)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending move to request" << std::endl;
  };

  std::string request = _request.toStdString();
  if (request == "move_to")
  {
    ignition::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->moveToService, req, cb);
  }
  else if (request == "follow")
  {
    ignition::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->followService, req, cb);
  }
  else if (request == "view_collisions")
  {
    ignition::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewCollisionsService, req, cb);
  }
  else
  {
    ignwarn << "Unknown request [" << request << "]" << std::endl;
  }
}
