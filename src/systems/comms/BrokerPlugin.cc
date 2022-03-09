/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <map>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <sdf/sdf.hh>

#include "ignition/gazebo/Util.hh"

#include "Broker.hh"
#include "BrokerPlugin.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::BrokerPluginPrivate
{
  /// \brief Broker instance.
  public: Broker broker;
};

//////////////////////////////////////////////////
BrokerPlugin::BrokerPlugin()
  : dataPtr(std::make_unique<BrokerPluginPrivate>())
{
}

//////////////////////////////////////////////////
void BrokerPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
}

//////////////////////////////////////////////////
void BrokerPlugin::PreUpdate(
    const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  IGN_PROFILE("BrokerPlugin::PreUpdate");
}

IGNITION_ADD_PLUGIN(BrokerPlugin,
                    ignition::gazebo::System,
                    BrokerPlugin::ISystemConfigure,
                    BrokerPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(BrokerPlugin,
                          "ignition::gazebo::systems::BrokerPlugin")
