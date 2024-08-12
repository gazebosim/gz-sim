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

#include "ThermalSensor.hh"

#include <limits>
#include <string>

#include <gz/common/Util.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/components/ThermalCamera.hh"
#include "gz/sim/components/Temperature.hh"
#include "gz/sim/components/TemperatureRange.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private Thermal sensor data class.
class gz::sim::systems::ThermalSensorPrivate
{
};

//////////////////////////////////////////////////
ThermalSensor::ThermalSensor() : System(),
    dataPtr(std::make_unique<ThermalSensorPrivate>())
{
}

//////////////////////////////////////////////////
ThermalSensor::~ThermalSensor() = default;

//////////////////////////////////////////////////
void ThermalSensor::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    sim::EntityComponentManager &_ecm,
    sim::EventManager & /*_eventMgr*/)
{
  const std::string resolutionTag = "resolution";
  const std::string minTempTag = "min_temp";
  const std::string maxTempTag = "max_temp";

  auto sensorComp = _ecm.Component<components::ThermalCamera>(_entity);
  if (sensorComp == nullptr)
  {
    gzerr << "The thermal sensor system can only be used to configure "
           << "parameters of thermal camera sensor " << std::endl;
    return;
  }

  if (_sdf->HasElement(resolutionTag))
  {
     double resolution = _sdf->Get<double>(resolutionTag);
    _ecm.CreateComponent(_entity,
        components::TemperatureLinearResolution(resolution));
  }

  if (_sdf->HasElement(minTempTag) || _sdf->HasElement(maxTempTag))
  {
    double min = 0.0;
    double max = std::numeric_limits<double>::max();
    min = _sdf->Get<double>(minTempTag, min).first;
    max = _sdf->Get<double>(maxTempTag, max).first;
    components::TemperatureRangeInfo rangeInfo;
    rangeInfo.min = min;
    rangeInfo.max = max;
    _ecm.CreateComponent(_entity, components::TemperatureRange(rangeInfo));
  }
}

GZ_ADD_PLUGIN(ThermalSensor, System,
  ThermalSensor::ISystemConfigure
)

GZ_ADD_PLUGIN_ALIAS(ThermalSensor,
    "gz::sim::systems::ThermalSensor")
