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

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Temperature.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Util.hh"

#include "Thermal.hh"

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

IGNITION_ADD_PLUGIN_ALIAS(Thermal, "ignition::gazebo::systems::Thermal")
