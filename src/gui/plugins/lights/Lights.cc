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

#include <iostream>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/Publisher.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

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
  std::string modelSdfString = _sdfString.toStdString();
  std::transform(modelSdfString.begin(), modelSdfString.end(),
                 modelSdfString.begin(), ::tolower);

  if (modelSdfString == "point")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.6\">"
                                 "<light type='point' name='pointlight'>"
                                   "<pose>0 0 2 0 0 0</pose>"
                                   "<cast_shadows>false</cast_shadows>"
                                   "<diffuse>0.5 0.5 0.5 1</diffuse>"
                                   "<specular>0.5 0.5 0.5 1</specular>"
                                   "<attenuation>"
                                     "<range>4</range>"
                                     "<constant>0.2</constant>"
                                     "<linear>0.5</linear>"
                                     "<quadratic>0.01</quadratic>"
                                   "</attenuation>"
                                 "</light>"
                                 "</sdf>");
  }
  else if (modelSdfString == "directional")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.6\">"
                                 "<light type='directional'"
                                  "name='directionallight'>"
                                   "<pose>0 0 5 0 0 0</pose>"
                                   "<cast_shadows>true</cast_shadows>"
                                   "<diffuse>0.8 0.8 0.8 1</diffuse>"
                                   "<specular>0.2 0.2 0.2 1</specular>"
                                   "<attenuation>"
                                     "<range>1000</range>"
                                     "<constant>0.9</constant>"
                                     "<linear>0.01</linear>"
                                     "<quadratic>0.001</quadratic>"
                                   "</attenuation>"
                                   "<direction>0 0 -1</direction>"
                                 "</light>"
                                 "</sdf>");
  }
  else if (modelSdfString == "spot")
  {
    modelSdfString = std::string("<?xml version=\"1.0\"?>"
                                 "<sdf version=\"1.6\">"
                                 "<light type='spot' name='spotlight'>"
                                   "<pose>0 0 2 0 0 0</pose>"
                                   "<cast_shadows>true</cast_shadows>"
                                   "<diffuse>0.5 0.5 0.5 1</diffuse>"
                                   "<specular>0.5 0.5 0.5 1</specular>"
                                   "<attenuation>"
                                     "<range>4</range>"
                                     "<constant>0.2</constant>"
                                     "<linear>0.5</linear>"
                                     "<quadratic>0.01</quadratic>"
                                   "</attenuation>"
                                   "<direction>0 0 -1</direction>"
                                   "<spot>"
                                     "<inner_angle>0.1</inner_angle>"
                                     "<outer_angle>0.5</outer_angle>"
                                     "<falloff>0.8</falloff>"
                                   "</spot>"
                                 "</light>"
                                 "</sdf>");
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

  gui::events::SpawnPreviewModel event(modelSdfString);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      &event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Lights,
                    ignition::gui::Plugin)
