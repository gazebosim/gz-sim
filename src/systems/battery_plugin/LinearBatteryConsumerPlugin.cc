/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "LinearBatteryConsumerPlugin.hh"

#include <ignition/plugin/Register.hh>

#include <ignition/common/Util.hh>
#include <ignition/common/Battery.hh>

#include <sdf/Element.hh>

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/components/Battery.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LinearBatteryConsumerPluginPrivate
{
  /// \brief Pointer to battery.
  public: common::BatteryPtr battery;

  /// \brief Battery consumer identifier.
  public: int32_t consumerId;
};

/////////////////////////////////////////////////
LinearBatteryConsumerPlugin::LinearBatteryConsumerPlugin()
    : System(), dataPtr(std::make_unique<LinearBatteryConsumerPluginPrivate>())
{
  this->dataPtr->consumerId = -1;
}

/////////////////////////////////////////////////
LinearBatteryConsumerPlugin::~LinearBatteryConsumerPlugin()
{
  if (this->dataPtr->battery && this->dataPtr->consumerId != -1)
    this->dataPtr->battery->RemoveConsumer(this->dataPtr->consumerId);
}

/////////////////////////////////////////////////
void LinearBatteryConsumerPlugin::Configure(const Entity &_entity,
               const std::shared_ptr<const sdf::Element> &_sdf,
               EntityComponentManager &_ecm,
               EventManager &/*_eventMgr*/)
{
  // Store the pointer to the model
  Model model = Model(_entity);
  if (!model.Valid(_ecm))
  {
    ignerr << "Linear battery plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Pointer to link that this plugin targets
  Entity linkEntity = kNullEntity;
  if (_sdf->HasElement("link_name"))
  {
    auto linkName = _sdf->Get<std::string>("link_name");

    linkEntity = model.LinkByName(_ecm, linkName);
    IGN_ASSERT(linkEntity != kNullEntity, "Link was NULL");
  }
  else
  {
    ignerr << "link_name not supplied, ignoring LinearBatteryConsumerPlugin.\n";
    return;
  }

  if (_sdf->HasElement("battery_name"))
  {
    auto batteryName = _sdf->Get<std::string>("battery_name");

    _ecm.Each<components::Battery, components::Name>(
        [&](const Entity &_batEntity, const components::Battery *_batComp,
            const components::Name *_nameComp) -> bool
        {
          // If parent link matches the target link, and battery name matches
          //   the target battery
          if ((_ecm.ParentEntity(_batEntity) == linkEntity) &&
            (_nameComp->Data() == batteryName))
          {
            this->dataPtr->battery = _batComp->Data();
            return true;
          }
          return true;
        });

    IGN_ASSERT(this->dataPtr->battery, "Battery was NULL");

    if (!this->dataPtr->battery)
    {
      ignerr << "Battery with name[" << batteryName << "] not found. "
            << "The LinearBatteryPlugin will not update its voltage\n";
    }
  }
  else
  {
    ignerr << "No <battery_name> specified.\n";
  }

  if (_sdf->HasElement("power_load"))
  {
    auto powerLoad = _sdf->Get<double>("power_load");
    this->dataPtr->consumerId = this->dataPtr->battery->AddConsumer();
    bool success = this->dataPtr->battery->SetPowerLoad(
      this->dataPtr->consumerId, powerLoad);
    if (!success)
      ignerr << "Failed to set consumer power load." << std::endl;
  }
  else
  {
    ignwarn << "Required attribute power_load missing "
            << "in LinearBatteryConsumerPlugin SDF" << std::endl;
  }

  ignmsg << "LinearBatteryConsumerPlugin configured\n";
}

IGNITION_ADD_PLUGIN(LinearBatteryConsumerPlugin,
                    ignition::gazebo::System,
                    LinearBatteryConsumerPlugin::ISystemConfigure)

IGNITION_ADD_PLUGIN_ALIAS(LinearBatteryConsumerPlugin,
  "ignition::gazebo::systems::LinearBatteryConsumerPlugin")
