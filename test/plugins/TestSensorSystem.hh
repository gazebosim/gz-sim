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
#ifndef GZ_SIM_TEST_TESTSENSORSYSTEM_HH_
#define GZ_SIM_TEST_TESTSENSORSYSTEM_HH_

#include <gz/msgs/stringmsg.pb.h>

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/System.hh>
#include <gz/sim/config.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
using SensorPluginComponent = components::Component<int,
    class SensorPluginComponentTag>;
GZ_SIM_REGISTER_COMPONENT("SensorPluginComponent",
    SensorPluginComponent)
}
}

class TestSensorSystem :
  public System,
  public ISystemConfigure
{
  public: TestSensorSystem()
        {
          gzdbg << "Constructing TestSensorSystem" << std::endl;
        }

  public: ~TestSensorSystem()
        {
          gzdbg << "Destroying TestSensorSystem" << std::endl;
        }

  private: bool Service(msgs::StringMsg &_msg)
           {
             gzdbg << "TestSensorSystem service called" << std::endl;
             _msg.set_data("TestSensorSystem");
             return true;
           }

  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &/*_eventManager*/) override
        {
          gzdbg << "Configuring TestSensorSystem" << std::endl;
          auto value = _sdf->Get<int>("sensor_key");
          _ecm.CreateComponent(_entity,
              components::SensorPluginComponent(value));

          // Create a test service
          this->node.Advertise("/test/service/sensor",
              &TestSensorSystem::Service, this);
        }

  private: transport::Node node;
};
}
}

#endif
