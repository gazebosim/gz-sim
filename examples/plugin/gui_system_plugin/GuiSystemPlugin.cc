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

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/gui/GuiEvents.hh>

#include "GuiSystemPlugin.hh"

/////////////////////////////////////////////////
GuiSystemPlugin::GuiSystemPlugin() = default;

/////////////////////////////////////////////////
GuiSystemPlugin::~GuiSystemPlugin() = default;

/////////////////////////////////////////////////
void GuiSystemPlugin::LoadConfig(const tinyxml2::XMLElement * /*_pluginElem*/)
{
  if (this->title.empty())
    this->title = "GUI system plugin";

  // Here you can read configuration from _pluginElem, if it's not null.
}

//////////////////////////////////////////////////
void GuiSystemPlugin::Update(const gz::sim::UpdateInfo & /*_info*/,
    gz::sim::EntityComponentManager &_ecm)
{
  // In the update loop, you can for example get the name of the world and set
  // it as a property that can be read from the QML.
  _ecm.Each<gz::sim::components::Name,
            gz::sim::components::World>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Name *_name,
        const gz::sim::components::World *)->bool
  {
    this->SetCustomProperty(QString::fromStdString(_name->Data()));
    return true;
  });
}

/////////////////////////////////////////////////
QString GuiSystemPlugin::CustomProperty() const
{
  return this->customProperty;
}

/////////////////////////////////////////////////
void GuiSystemPlugin::SetCustomProperty(const QString &_customProperty)
{
  this->customProperty = _customProperty;
  this->CustomPropertyChanged();
}

// Register this plugin
GZ_ADD_PLUGIN(GuiSystemPlugin,
                    gz::gui::Plugin)
