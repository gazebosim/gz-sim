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

#include "Shapes.hh"

#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/stringmsg.pb.h>

#include <algorithm>
#include <iostream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/GuiEvents.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include <ignition/gazebo/Primitives.hh>

namespace ignition::gazebo
{
  class ShapesPrivate
  {
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
Shapes::Shapes()
  : ignition::gui::Plugin(),
  dataPtr(std::make_unique<ShapesPrivate>())
{
}

/////////////////////////////////////////////////
Shapes::~Shapes() = default;

/////////////////////////////////////////////////
void Shapes::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Shapes";

  // For shapes requests
  ignition::gui::App()->findChild
    <ignition::gui::MainWindow *>()->installEventFilter(this);
}

/////////////////////////////////////////////////
void Shapes::OnMode(const QString &_mode)
{
  std::string modelSdfString = _mode.toStdString();
  modelSdfString = getPrimitive(modelSdfString);

  if (!modelSdfString.empty())
  {
    ignition::gui::events::SpawnFromDescription event(modelSdfString);
    ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &event);
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Shapes,
                    ignition::gui::Plugin)
