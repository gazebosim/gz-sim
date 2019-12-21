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

#include "TransformControl.hh"

namespace ignition::gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
  class TransformControlPrivate
  {
    /// \brief Ignition communication node.
    public: transport::Node node;

    /// \brief Mutex to protect mode
    public: std::mutex mutex;

    /// \brief Transform control service name
    public: std::string service;

    public: EventManager* eventMgr;
  };
}
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
TransformControl::TransformControl()
  : GuiSystem(), dataPtr(new TransformControlPrivate)
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
void TransformControl::Configure(EntityComponentManager &_ecm, EventManager &_eventMgr)
{
  this->dataPtr->eventMgr = &_eventMgr;
}

void TransformControl::OnNewSnapTranslation(double x)
{
  /*
  math::Vector3d &_snap;
  this->dataPtr->eventMgr->Emit<>();
  */
}

void TransformControl::OnNewSnapRotation(double x, double y, double z)
{
  math::Vector3d snap(x, y, z);
  this->dataPtr->eventMgr->Emit<events::SnapRotateConfig>(snap);
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
