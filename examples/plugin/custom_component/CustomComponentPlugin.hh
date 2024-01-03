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
#include <gz/sim/components/Component.hh>

// This header provides the registration macro
#include <gz/sim/components/Factory.hh>

#include <gz/sim/System.hh>
#include <gz/sim/config.hh>

namespace examples
{
// The most convenient way to create a component is to create a template alias
// of `components::Component` where the first argument is the type being wrapped
// (i.e. int) and the second is a unique name tag.
using CustomComponent =
    gz::sim::components::Component<int, class CustomComponentTag>;

// Use this macro to register a component. Give it a unique name across the
// entire simulation.
GZ_SIM_REGISTER_COMPONENT("examples::CustomComponent", CustomComponent)

class CustomComponentPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure
{
  public: CustomComponentPlugin() = default;

  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &) override
  {
    // You can create the custom component as you would create any other
    gzdbg << "Creating component" << std::endl;
    _ecm.CreateComponent(_entity, CustomComponent(123));

    // You can use the ECM's API, such as Each, to query for the
    // component:
    _ecm.Each<CustomComponent>(
        [&](const gz::sim::Entity &_entityEach,
            const CustomComponent *_comp) -> bool
    {
      gzdbg << "Entity: " << _entityEach << std::endl;
      gzdbg << "Component's data: " << _comp->Data() << std::endl;
      return true;
    });
  }
};
}

#endif
