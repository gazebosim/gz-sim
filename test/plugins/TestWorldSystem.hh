/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_SIM_TEST_TESTWORLDSYSTEM_HH_
#define GZ_SIM_TEST_TESTWORLDSYSTEM_HH_

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/System.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
using WorldPluginComponent =
    components::Component<double, class WorldPluginComponentTag>;
GZ_SIM_REGISTER_COMPONENT("WorldPluginComponent",
    WorldPluginComponent)
}
}

class TestWorldSystem :
  public System,
  public ISystemConfigure,
  public ISystemUpdate
{
  public: TestWorldSystem()
        {
          gzdbg << "Constructing TestWorldSystem" << std::endl;
        }

  public: ~TestWorldSystem()
        {
          gzdbg << "Destroying TestWorldSystem" << std::endl;
        }

  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventManager*/) override
        {
          gzdbg << "Configuring TestWorldSystem" << std::endl;
          auto value = _sdf->Get<double>("world_key");
          _ecm.CreateComponent(_entity,
              components::WorldPluginComponent(value));
        }

  public: void Update(const sim::UpdateInfo &_info,
                      EntityComponentManager &) override
          {
            std::cout << "iteration " << _info.iterations << std::endl;
          }
};
}
}

#endif
