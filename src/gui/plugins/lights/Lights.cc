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

#include <ignition/gazebo/gui/GuiUtils.hh>
#include "ignition/gazebo/EntityComponentManager.hh"

namespace ignition::gazebo
{
  class LightsPrivate
  {
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
Lights::Lights()
  : ignition::gui::Plugin(),
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
  using PrimitiveLight = ignition::gazebo::gui::PrimitiveLight;

  std::string modelSdfString = _sdfString.toStdString();
  std::transform(modelSdfString.begin(), modelSdfString.end(),
                 modelSdfString.begin(), ::tolower);

  if (modelSdfString == "point")
  {
    modelSdfString = gui::getPrimitiveLight(PrimitiveLight::kPoint);
  }
  else if (modelSdfString == "directional")
  {
    modelSdfString = gui::getPrimitiveLight(PrimitiveLight::kDirectional);
  }
  else if (modelSdfString == "spot")
  {
    modelSdfString = gui::getPrimitiveLight(PrimitiveLight::kSpot);
  }
  else
  {
    ignwarn << "Invalid model string " << modelSdfString << "\n";
    ignwarn << "The valid options are:\n";
    ignwarn << " - point\n";
    ignwarn << " - directional\n";
    ignwarn << " - spot\n";
    return;
  }

  ignition::gui::events::SpawnFromDescription event(modelSdfString);
  ignition::gui::App()->sendEvent(
        ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
        &event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Lights,
                    ignition::gui::Plugin)
