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
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "EntityContextMenu.hh"

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
}

/////////////////////////////////////////////////
EntityContextMenu::~EntityContextMenu() = default;

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

  else
  {
    ignwarn << "Unknown request [" << request << "]" << std::endl;
  }
}
