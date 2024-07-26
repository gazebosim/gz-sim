/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

// We'll use a string and the gzmsg command below for a brief example.
// Remove these includes if your plugin doesn't need them.
#include <memory>
#include <string>
#include <gz/common/Console.hh>

// This header is required to register plugins. It's good practice to place it
// in the cc file, like it's done here.
#include <gz/plugin/Register.hh>

// Don't forget to include the plugin's header.
#include "PriorityPrinter.hh"

// This is required to register the plugin. Make sure the interfaces match
// what's in the header.
GZ_ADD_PLUGIN(
    priority_printer::PriorityPrinter,
    gz::sim::System,
    priority_printer::PriorityPrinter::ISystemConfigure,
    priority_printer::PriorityPrinter::ISystemPreUpdate,
    priority_printer::PriorityPrinter::ISystemUpdate)

using namespace priority_printer;

void PriorityPrinter::Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr)
{
  // Parse priority value as a string for printing
  const std::string priorityElementName {gz::sim::System::kPriorityElementName};
  if (_sdf && _sdf->HasElement(priorityElementName))
  {
    this->systemPriority = _sdf->Get<std::string>(priorityElementName);
  }

  const std::string labelElementName {"label"};
  if (_sdf && _sdf->HasElement(labelElementName))
  {
    this->systemLabel = _sdf->Get<std::string>(labelElementName);
  }
}

void PriorityPrinter::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
  gzmsg << "PreUpdate: "
        << "Iteration " << _info.iterations
        << ", system priority " << this->systemPriority
        << ", system label " << this->systemLabel
        << '\n';
}

void PriorityPrinter::Update(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &/*_ecm*/)
{
  gzmsg << "Update: "
        << "Iteration " << _info.iterations
        << ", system priority " << this->systemPriority
        << ", system label " << this->systemLabel
        << '\n';
}
