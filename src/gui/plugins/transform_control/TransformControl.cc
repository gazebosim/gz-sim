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
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "TransformControl.hh"

namespace ignition::gazebo
{
  class TransformControlPrivate
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
TransformControl::TransformControl()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<TransformControlPrivate>())
{
}

/////////////////////////////////////////////////
TransformControl::~TransformControl() = default;

/////////////////////////////////////////////////
void TransformControl::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Transform control";

  // For transform requests
  this->dataPtr->service = "/gui/transform_mode";
}

/////////////////////////////////////////////////
void TransformControl::OnSnapUpdate(
    double _x, double _y, double _z,
    double _roll, double _pitch, double _yaw,
    double _scaleX, double _scaleY, double _scaleZ)
{
  auto event = new gui::events::SnapIntervals(
      math::Vector3d(_x, _y, _z), math::Vector3d(_roll, _pitch, _yaw),
      math::Vector3d(_scaleX, _scaleY, _scaleZ));
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(), event);
}

/////////////////////////////////////////////////
void TransformControl::OnMode(const QString &_mode)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting transform mode" << std::endl;
  };

  ignition::msgs::StringMsg req;
  req.set_data(_mode.toStdString());
  this->dataPtr->node.Request(this->dataPtr->service, req, cb);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::TransformControl,
                    ignition::gui::Plugin)
