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

#include <gtest/gtest.h>

#include <gz/common/Filesystem.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Environment.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/TestFixture.hh"
#include "gz/sim/Util.hh"

#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

using namespace gz;
using namespace sim;

/// \brief Test EnvironmentPreload system
class EnvironmentPreloadTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(EnvironmentPreloadTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(CanPreload))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "test",
      "worlds", "environmental_data.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  bool dataLoaded{false};

  // Create a system that looks for environmental data components
  test::Relay testSystem;
  testSystem.OnPostUpdate(
      [&](const sim::UpdateInfo &,
          const sim::EntityComponentManager &_ecm)
      {
        _ecm.EachNew<components::Environment>(
            [&](const gz::sim::Entity &,
                const components::Environment *_component) -> bool
            {
              auto data = _component->Data();
              EXPECT_TRUE(data->frame.Has("humidity"));
              const auto &humidityData = data->frame["humidity"];
              auto humiditySession = humidityData.StepTo(
                  humidityData.CreateSession(), 1658923062.5);
              EXPECT_TRUE(humiditySession.has_value());
              if (humiditySession.has_value())
              {
                const math::Vector3d position{36.80079505, -121.789472517, 0.8};
                auto humidity =
                    humidityData.LookUp(humiditySession.value(),
                      position);
                EXPECT_NEAR(89.5, humidity.value_or(0.), 1e-6);
                dataLoaded = true;
              }
              EXPECT_EQ(data->reference, math::SphericalCoordinates::SPHERICAL);
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server
  server.RunOnce();

  EXPECT_TRUE(dataLoaded);
}


/////////////////////////////////////////////////
TEST_F(EnvironmentPreloadTest,
  GZ_UTILS_TEST_DISABLED_ON_WIN32(CorrectSphericalTransform))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "test",
      "worlds", "environmental_data.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  bool dataLoaded{false};

  // Create a system that looks for environmental data components
  test::Relay testSystem;
  testSystem.OnPostUpdate(
      [&](const sim::UpdateInfo &,
          const sim::EntityComponentManager &_ecm)
      {
        _ecm.EachNew<components::Environment>(
            [&](const gz::sim::Entity &,
                const components::Environment *_component) -> bool
            {
              auto data = _component->Data();
              EXPECT_TRUE(data->frame.Has("humidity"));
              const auto &humidityData = data->frame["humidity"];
              auto humiditySession = humidityData.StepTo(
                  humidityData.CreateSession(), 1658923062.5);
              EXPECT_TRUE(humiditySession.has_value());
              if (humiditySession.has_value())
              {
                const math::Vector3d position{0, 0, 0.8};
                auto transformedCoordinates =
                  getGridFieldCoordinates(_ecm, position, data);
                auto humidity =
                    humidityData.LookUp(humiditySession.value(),
                      transformedCoordinates.value());
                EXPECT_NEAR(89.5, humidity.value_or(0.), 1e-6);
                dataLoaded = true;
              }
              EXPECT_EQ(data->reference, math::SphericalCoordinates::SPHERICAL);
              return true;
            });
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server
  server.RunOnce();

  EXPECT_TRUE(dataLoaded);
}
