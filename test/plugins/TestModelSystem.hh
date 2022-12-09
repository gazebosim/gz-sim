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
#ifndef GZ_SIM_TEST_TESTMODELSYSTEM_HH_
#define GZ_SIM_TEST_TESTMODELSYSTEM_HH_

#include <gz/msgs/stringmsg.pb.h>

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/config.hh>


namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
using ModelPluginComponent = components::Component<int,
    class ModelPluginComponentTag>;
GZ_SIM_REGISTER_COMPONENT("ModelPluginComponent",
    ModelPluginComponent)
}
}

class TestModelSystem :
  public System,
  public ISystemConfigure
{
  public: TestModelSystem()
        {
          gzdbg << "Constructing TestModelSystem" << std::endl;
        }

  public: ~TestModelSystem()
        {
          gzdbg << "Destroying TestModelSystem" << std::endl;
        }

  private: bool Service(msgs::StringMsg &_msg)
           {
             gzdbg << "TestModelSystem service called" << std::endl;
             _msg.set_data("TestModelSystem");
             return true;
           }

  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventManager*/) override
        {
          gzdbg << "Configuring TestModelSystem" << std::endl;
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
            gzerr << "Failed to find link" << std::endl;
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
