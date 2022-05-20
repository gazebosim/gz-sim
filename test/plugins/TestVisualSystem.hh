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
#ifndef GZ_GAZEBO_TEST_TESTVISUALSYSTEM_HH_
#define GZ_GAZEBO_TEST_TESTVISUALSYSTEM_HH_

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/config.hh>
#include <ignition/transport/Node.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_GAZEBO_VERSION_NAMESPACE {
namespace components
{
using VisualPluginComponent = components::Component<int,
    class VisualPluginComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("VisualPluginComponent",
    VisualPluginComponent)
}
}

class TestVisualSystem :
  public System,
  public ISystemConfigure
{
  public: TestVisualSystem()
        {
          igndbg << "Constructing TestVisualSystem" << std::endl;
        }

  public: ~TestVisualSystem()
        {
          igndbg << "Destroying TestVisualSystem" << std::endl;
        }

  private: bool Service(msgs::StringMsg &_msg)
        {
          igndbg << "TestVisualSystem service called" << std::endl;
          _msg.set_data("TestVisualSystem");
          return true;
        }

  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventManager*/) override
        {
          igndbg << "Configuring TestVisualSystem" << std::endl;
          auto value = _sdf->Get<int>("visual_key");
          _ecm.CreateComponent(_entity,
              components::VisualPluginComponent(value));

          // Create a test service
          this->node.Advertise("/test/service/visual",
              &TestVisualSystem::Service, this);
        }

  private: transport::Node node;
};
}
}

#endif
