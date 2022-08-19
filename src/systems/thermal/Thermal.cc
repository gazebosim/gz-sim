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

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Temperature.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"

#include "Thermal.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private Thermal data class.
class gz::sim::systems::ThermalPrivate
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
    EntityComponentManager &_ecm,
    EventManager & /*_eventMgr*/)
{
  if (!_sdf->HasElement("temperature"))
  {
    ignerr << "Fail to load thermal system: <temperature> is not specified"
           << std::endl;
    return;
  }
  double temperature = _sdf->Get<double>("temperature");
  _ecm.CreateComponent(_entity, components::Temperature(temperature));
}


IGNITION_ADD_PLUGIN(Thermal, System,
  Thermal::ISystemConfigure
)

IGNITION_ADD_PLUGIN_ALIAS(Thermal, "gz::sim::systems::Thermal")
