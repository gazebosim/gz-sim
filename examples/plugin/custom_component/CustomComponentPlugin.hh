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
#ifndef EXAMPLES_PLUGINS_CUSTOMCOMPONENTPLUGIN_HH_
#define EXAMPLES_PLUGINS_CUSTOMCOMPONENTPLUGIN_HH_

// This header provides components::Component
#include <ignition/gazebo/components/Component.hh>

// This header provides the registration macro
#include <ignition/gazebo/components/Factory.hh>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/config.hh>

namespace examples
{
// The most convenient way to create a component is to create a template alias
// of `components::Component` where the first argument is the type being wrapped
// (i.e. int) and the second is a unique name tag.
using CustomComponent =
    ignition::gazebo::components::Component<int, class CustomComponentTag>;

// Use this macro to register a component. Give it a unique name across the
// entire simulation.
IGN_GAZEBO_REGISTER_COMPONENT("examples::CustomComponent", CustomComponent)

class CustomComponentPlugin :
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure
{
  public: CustomComponentPlugin() = default;

  public: void Configure(const ignition::gazebo::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &,
                         ignition::gazebo::EntityComponentManager &_ecm,
                         ignition::gazebo::EventManager &) override
  {
    // You can create the custom component as you would create any other
    igndbg << "Creating component" << std::endl;
    _ecm.CreateComponent(_entity, CustomComponent(123));

    // You can use the ECM's API, such as Each, to query for the
    // component:
    _ecm.Each<CustomComponent>(
        [&](const ignition::gazebo::Entity &_entityEach,
            const CustomComponent *_comp) -> bool
    {
      igndbg << "Entity: " << _entityEach << std::endl;
      igndbg << "Component's data: " << _comp->Data() << std::endl;
      return true;
    });
  }
};
}

#endif
