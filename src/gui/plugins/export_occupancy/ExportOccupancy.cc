/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "ExportOccupancy.hh"
#include <gz/plugin/Register.hh>

using namespace gz;
using namespace sim;

class gz::sim::ExportOccupancyUiPrivate
{
};

ExportOccupancyUi::ExportOccupancyUi() : dataPtr(std::make_unique<ExportOccupancyUiPrivate>())
{

}

ExportOccupancyUi::~ExportOccupancyUi()
{

}

void ExportOccupancyUi::LoadConfig(
    const tinyxml2::XMLElement *_pluginElem)
{

}

void ExportOccupancyUi::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{

}

// Register this plugin
GZ_ADD_PLUGIN(gz::sim::ExportOccupancyUi, gz::gui::Plugin)
