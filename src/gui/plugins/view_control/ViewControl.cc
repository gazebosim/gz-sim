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

#include "ViewControl.hh"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <iostream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>


namespace ignition::gazebo
{
  class ViewControlPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
    public: std::mutex mutex;

    /// \brief Transform control service name
    public: std::string service;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
ViewControl::ViewControl()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<ViewControlPrivate>())
{
}

/////////////////////////////////////////////////
ViewControl::~ViewControl() = default;

/////////////////////////////////////////////////
void ViewControl::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "View control";

  // view control requests
  this->dataPtr->service = "/gui/camera/view_control";
}

/////////////////////////////////////////////////
void ViewControl::OnViewControl(const QString &_controller)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting view controller" << std::endl;
  };

  ignition::msgs::StringMsg req;
  std::string str = _controller.toStdString();
  if (str.find("Orbit") != std::string::npos)
    req.set_data("orbit");
  else if (str.find("Ortho") != std::string::npos)
    req.set_data("ortho");
  else
  {
    ignerr << "Unknown view controller selected: " << str << std::endl;
    return;
  }

  this->dataPtr->node.Request(this->dataPtr->service, req, cb);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ViewControl,
                    ignition::gui::Plugin)
