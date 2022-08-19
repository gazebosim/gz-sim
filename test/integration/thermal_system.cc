/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Temperature.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

#define tol 10e-4

using namespace gz;
using namespace sim;

/// \brief Test Thermal system
class ThermalTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(ThermalTest, TemperatureComponent)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/thermal.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that checks for thermal component
  test::Relay testSystem;

  std::map<std::string, math::Temperature> entityTemp;
  testSystem.OnPostUpdate([&](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Temperature, components::Name>(
          [&](const Entity &_id,
              const components::Temperature *_temp,
              const components::Name *_name) -> bool
          {
            // store temperature data
            entityTemp[_name->Data()] = _temp->Data();

            // verify temperature data belongs to a visual
            EXPECT_NE(nullptr, _ecm.Component<components::Visual>(_id));

            return true;
          });
    });
  server.AddSystem(testSystem.systemPtr);

  // verify nothing in map at beginning
  EXPECT_TRUE(entityTemp.empty());

  // Run server
  server.Run(true, 1, false);

  // verify temperature components are created and the values are correct
  EXPECT_EQ(3u, entityTemp.size());
  EXPECT_DOUBLE_EQ(200.0, entityTemp["box_visual"].Kelvin());
  EXPECT_DOUBLE_EQ(600.0, entityTemp["sphere_visual"].Kelvin());
  EXPECT_DOUBLE_EQ(400.0, entityTemp["cylinder_visual"].Kelvin());
}
