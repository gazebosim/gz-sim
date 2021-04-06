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
#ifndef IGNITION_GAZEBO_TEST_TESTMODELSYSTEM_HH_
#define IGNITION_GAZEBO_TEST_TESTMODELSYSTEM_HH_

#include <ignition/gazebo/components/Component.hh>
#include <ignition/gazebo/components/Factory.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/gazebo/config.hh>

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
using ModelPluginComponent = components::Component<int,
    class ModelPluginComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ModelPluginComponent",
    ModelPluginComponent)
}
}

class TestModelSystem :
  public System,
  public ISystemConfigure
{
  public: TestModelSystem()
        {
          igndbg << "Constructing TestModelSystem" << std::endl;
        }

  public: ~TestModelSystem()
        {
          igndbg << "Destroying TestModelSystem" << std::endl;
        }

  private: bool Service(msgs::StringMsg &_msg)
           {
             igndbg << "TestModelSystem service called" << std::endl;
             _msg.set_data("TestModelSystem");
             return true;
           }

  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventManager*/) override
        {
          igndbg << "Configuring TestModelSystem" << std::endl;
          this->model = Model(_entity);

          auto link = this->model.LinkByName(_ecm, "link_1");
          // This plugin might have been attached to the models in
          // test/world/shapes.world.
          if (link == kNullEntity)
            link = this->model.LinkByName(_ecm, "box_link");
          if (link == kNullEntity)
            link = this->model.LinkByName(_ecm, "sphere_link");
          if (link == kNullEntity)
            link = this->model.LinkByName(_ecm, "cylinder_link");

          // Fail to create component if link is not found
          if (link == kNullEntity)
          {
            ignerr << "Failed to find link" << std::endl;
            return;
          }

          // Create a test service
          this->node.Advertise("/test/service",
              &TestModelSystem::Service, this);

          auto value = _sdf->Get<int>("model_key");
          _ecm.CreateComponent(_entity,
              components::ModelPluginComponent(value));
        }

  private: Model model;
  private: transport::Node node;
};
}
}

#endif
