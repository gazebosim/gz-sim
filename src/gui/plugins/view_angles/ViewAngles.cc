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
#include <ignition/gui/Application.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ViewAngles.hh"

namespace ignition::gazebo
{
  class ViewAnglesPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
    public: std::mutex mutex;

    /// \brief View Angles service name
    public: std::string service;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
ViewAngles::ViewAngles()
  : gui::Plugin(), dataPtr(std::make_unique<ViewAnglesPrivate>())
{
}

/////////////////////////////////////////////////
ViewAngles::~ViewAngles() = default;
/////////////////////////////////////////////////
void ViewAngles::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "View Angles";

  // For view angle requests
  this->dataPtr->service = "/gui/view_angles";
}

/////////////////////////////////////////////////
void ViewAngles::OnMode(const QString &_mode)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting view angle mode" << std::endl;
  };

  ignition::msgs::StringMsg req;
  req.set_data(_mode.toStdString());
  this->dataPtr->node.Request(this->dataPtr->service, req, cb);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ViewAngles,
                    ignition::gui::Plugin)
