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

#include <limits>
#include <string>

#include <ignition/common/URI.hh>
#include <ignition/common/Util.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/Atmosphere.hh"
#include "ignition/gazebo/components/SourceFilePath.hh"
#include "ignition/gazebo/components/Temperature.hh"
#include "ignition/gazebo/components/TemperatureRange.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

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
  const std::string minTempTag = "min_temp";
  const std::string maxTempTag = "max_temp";

  if (_sdf->HasElement(temperatureTag) && _sdf->HasElement(heatSignatureTag))
  {
    ignwarn << "Both <" << temperatureTag << "> and <" << heatSignatureTag
           << "> were specified, but the thermal system only uses one. "
           << "<" << heatSignatureTag << "> will be used.\n";
  }

  if (_sdf->HasElement(heatSignatureTag))
  {
    std::string heatSignature = _sdf->Get<std::string>(heatSignatureTag);
    auto modelEntity = topLevelModel(_entity, _ecm);
    auto modelPath =
      _ecm.ComponentData<components::SourceFilePath>(modelEntity);
    auto path = common::findFile(asFullPath(heatSignature, modelPath.value()));

    // make sure the specified heat signature can be found
    if (path.empty())
    {
      ignerr << "Failed to load thermal system. Heat signature ["
        << heatSignature << "] could not be found\n";
      return;
    }
    _ecm.CreateComponent(_entity, components::SourceFilePath(path));

    // see if the user defined a custom temperature range
    // (default to ambient temperature if possible)
    double min = 250.0;
    double max = 300.0;
    if (auto worldEntity = _ecm.EntityByComponents(components::World()))
    {
      if (auto atmosphere =
          _ecm.Component<components::Atmosphere>(worldEntity))
      {
        auto atmosphericTemp = atmosphere->Data().Temperature().Kelvin();
        min = atmosphericTemp;
        max = atmosphericTemp;
      }
    }
    if (_sdf->HasElement(minTempTag))
      min = _sdf->Get<double>(minTempTag);
    if (_sdf->HasElement(maxTempTag))
      max = _sdf->Get<double>(maxTempTag);
    // make sure that min is actually less than max
    if (min > max)
    {
      auto temporary = max;
      max = min;
      min = temporary;
    }
    igndbg << "Thermal plugin, heat signature: using a minimum temperature of "
      << min << " kelvin, and a max temperature of " << max << " kelvin.\n";

    components::TemperatureRangeInfo rangeInfo;
    rangeInfo.min = min;
    rangeInfo.max = max;
    _ecm.CreateComponent(_entity, components::TemperatureRange(rangeInfo));
  }
  else if (_sdf->HasElement(temperatureTag))
  {
    double temperature = _sdf->Get<double>(temperatureTag);
    _ecm.CreateComponent(_entity, components::Temperature(temperature));
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
