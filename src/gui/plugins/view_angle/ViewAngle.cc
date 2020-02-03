/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <ignition/msgs/vector3d.pb.h>

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include "ViewAngle.hh"

namespace ignition::gazebo
{
  class ViewAnglePrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect angle mode
    public: std::mutex mutex;

    /// \brief View Angle service name
    public: std::string service;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
ViewAngle::ViewAngle()
  : gui::Plugin(), dataPtr(std::make_unique<ViewAnglePrivate>())
{
}

/////////////////////////////////////////////////
ViewAngle::~ViewAngle() = default;

/////////////////////////////////////////////////
void ViewAngle::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "View Angle";

  // For view angle requests
  this->dataPtr->service = "/gui/view_angle";
}

/////////////////////////////////////////////////
void ViewAngle::OnAngleMode(int _x, int _y, int _z)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting view angle mode" << std::endl;
  };

  ignition::msgs::Vector3d req;
  req.set_x(_x);
  req.set_y(_y);
  req.set_z(_z);

  this->dataPtr->node.Request(this->dataPtr->service, req, cb);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ViewAngle,
                    ignition::gui::Plugin)
