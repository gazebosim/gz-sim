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

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/ForceTorque.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;

class ForceTorqueTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    ignition::common::setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str());
  }
};

/////////////////////////////////////////////////
// The test checks the world pose and sensor readings of a falling imu
TEST_F(ForceTorqueTest, Foo)
{
  double stepSize = 0.001;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/force_torque.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "force_torque_sensor";

  auto topic =
      "world/imu_sensor/model/imu_model/link/link/sensor/imu_sensor/imu";

  // Create a system that records imu data
  test::Relay testSystem;

  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        std::cerr << "OnPostUpdate" << std::endl;
        _ecm.Each<components::ForceTorque,
                  components::Name>(
                  
            [&](const ignition::gazebo::Entity &_entity,
                const components::ForceTorque *,
                const components::Name *_name) -> bool
            {
              std::cerr << _name->Data() << std::endl;
              EXPECT_EQ(_name->Data(), sensorName);

              auto sensorComp = _ecm.Component<components::Sensor>(_entity);
              EXPECT_NE(nullptr, sensorComp);

              auto topicComp = _ecm.Component<components::SensorTopic>(_entity);
              EXPECT_NE(nullptr, topicComp);
              if (topicComp)
              {
                EXPECT_EQ(topic, topicComp->Data());
              }

              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // subscribe to imu topic
  transport::Node node;

  // Run server
  size_t iters200 = 200u;
  server.Run(true, iters200, false);
}
