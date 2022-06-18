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

#include "Lights.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <algorithm>
#include <iostream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/Publisher.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Primitives.hh"

namespace gz::sim
{
  class LightsPrivate
  {
  };
}

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
Lights::Lights()
  : gz::gui::Plugin(),
  dataPtr(std::make_unique<LightsPrivate>())
{
}

/////////////////////////////////////////////////
Lights::~Lights() = default;

/////////////////////////////////////////////////
void Lights::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Lights";
}

/////////////////////////////////////////////////
void Lights::OnNewLightClicked(const QString &_sdfString)
{
  std::string modelSdfString = _sdfString.toStdString();
  modelSdfString = getPrimitive(modelSdfString);

  if (!modelSdfString.empty())
  {
    gz::gui::events::SpawnFromDescription event(modelSdfString);
    gz::gui::App()->sendEvent(
          gz::gui::App()->findChild<gz::gui::MainWindow *>(),
          &event);
  }
}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::Lights,
                    gz::gui::Plugin)
