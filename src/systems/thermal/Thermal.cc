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

#include "Thermal.hh"

#include <string>

#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/SourceFilePath.hh"
#include "ignition/gazebo/components/Temperature.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private Thermal data class.
class ignition::gazebo::systems::ThermalPrivate
{
};

//////////////////////////////////////////////////
Thermal::Thermal() : System(), dataPtr(std::make_unique<ThermalPrivate>())
{
}

//////////////////////////////////////////////////
Thermal::~Thermal() = default;

//////////////////////////////////////////////////
void Thermal::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gazebo::EntityComponentManager &_ecm,
    gazebo::EventManager & /*_eventMgr*/)
{
  const std::string temperatureTag = "temperature";
  const std::string heatSignatureTag = "heat_signature";

  if (_sdf->HasElement(temperatureTag) && _sdf->HasElement(heatSignatureTag))
  {
    ignerr << "Failed to load thermal system. "
           << "Both <" << temperatureTag << "> and <" << heatSignatureTag
           << "> were specified, but the thermal system only uses one.\n";
  }
  else if (_sdf->HasElement(temperatureTag))
  {
    double temperature = _sdf->Get<double>(temperatureTag);
    _ecm.CreateComponent(_entity, components::Temperature(temperature));
  }
  else if (_sdf->HasElement(heatSignatureTag))
  {
    std::string heatSignature = _sdf->Get<std::string>(heatSignatureTag);
    _ecm.CreateComponent(_entity, components::SourceFilePath(heatSignature));
  }
  else
  {
    ignerr << "Failed to load thermal system. "
           << "Neither <" << temperatureTag << "> or <" << heatSignatureTag
           << "> were specified.\n";
  }
}


IGNITION_ADD_PLUGIN(Thermal, System,
  Thermal::ISystemConfigure
)

IGNITION_ADD_PLUGIN_ALIAS(Thermal, "ignition::gazebo::systems::Thermal")
