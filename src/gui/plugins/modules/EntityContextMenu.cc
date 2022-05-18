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

#include "../../GuiRunner.hh"
#include "EntityContextMenu.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/entity.pb.h>

#include <iostream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/sim/Conversions.hh>
#include <gz/gui/Application.hh>
#include <gz/transport/Node.hh>

namespace gz::sim
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

    /// \brief View as transparent service name
    public: std::string viewTransparentService;

    /// \brief View center of mass service name
    public: std::string viewCOMService;

    /// \brief View inertia service name
    public: std::string viewInertiaService;

    /// \brief View joints service name
    public: std::string viewJointsService;

    /// \brief View wireframes service name
    public: std::string viewWireframesService;

    /// \brief View collisions service name
    public: std::string viewCollisionsService;

    /// \brief View frames service name
    public: std::string viewFramesService;

    /// \brief Copy service name
    public: std::string copyService;

    /// \brief Paste service name
    public: std::string pasteService;

    /// \brief Name of world.
    public: std::string worldName;
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
void GzSimPlugin::registerTypes(const char *_uri)
{
  // Register our 'EntityContextMenuItem' in qml engine
  qmlRegisterType<gz::sim::EntityContextMenu>(_uri, 1, 0,
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

  // For view transparent service requests
  this->dataPtr->viewTransparentService = "/gui/view/transparent";

  // For view center of mass service requests
  this->dataPtr->viewCOMService = "/gui/view/com";

  // For view inertia service requests
  this->dataPtr->viewInertiaService = "/gui/view/inertia";

  // For view joints service requests
  this->dataPtr->viewJointsService = "/gui/view/joints";

  // For view wireframes service requests
  this->dataPtr->viewWireframesService = "/gui/view/wireframes";

  // For view collisions service requests
  this->dataPtr->viewCollisionsService = "/gui/view/collisions";

  // For view frames service requests
  this->dataPtr->viewFramesService = "/gui/view/frames";

  // For copy service requests
  this->dataPtr->copyService = "/gui/copy";

  // For paste service requests
  this->dataPtr->pasteService = "/gui/paste";
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

  std::function<void(const gz::msgs::Boolean &, const bool)> cb =
      [](const gz::msgs::Boolean &_rep, const bool _result)
  {
    if (!_result || !_rep.data())
      ignerr << "Error sending remove request" << std::endl;
  };

  gz::msgs::Entity req;
  req.set_name(_data.toStdString());
  req.set_type(convert<msgs::Entity_Type>(_type.toStdString()));

  this->dataPtr->node.Request(this->dataPtr->removeService, req, cb);
}

/////////////////////////////////////////////////
void EntityContextMenu::OnRequest(const QString &_request, const QString &_data)
{
  std::function<void(const gz::msgs::Boolean &, const bool)> cb =
      [](const gz::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error sending move to request" << std::endl;
  };

  std::string request = _request.toStdString();
  if (request == "move_to")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->moveToService, req, cb);
  }
  else if (request == "follow")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->followService, req, cb);
  }
  else if (request == "view_transparent")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewTransparentService, req, cb);
  }
  else if (request == "view_com")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewCOMService, req, cb);
  }
  else if (request == "view_inertia")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewInertiaService, req, cb);
  }
  else if (request == "view_joints")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewJointsService, req, cb);
  }
  else if (request == "view_wireframes")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewWireframesService, req, cb);
  }
  else if (request == "view_collisions")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewCollisionsService, req, cb);
  }
  else if (request == "view_frames")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->viewFramesService, req, cb);
  }
  else if (request == "copy")
  {
    gz::msgs::StringMsg req;
    req.set_data(_data.toStdString());
    this->dataPtr->node.Request(this->dataPtr->copyService, req, cb);
  }
  else if (request == "paste")
  {
    gz::msgs::Empty req;
    this->dataPtr->node.Request(this->dataPtr->pasteService, req, cb);
  }
  else
  {
    ignwarn << "Unknown request [" << request << "]" << std::endl;
  }
}
