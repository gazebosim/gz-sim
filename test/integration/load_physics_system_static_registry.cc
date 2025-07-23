/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include <gz/common/Util.hh>

#include "../helpers/EnvTestFixture.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/PhysicsEnginePlugin.hh"
#include "plugins/MockSystem.hh"

using namespace gz;
using namespace sim;

/// \brief Test loading physics system and physics plugin from static
/// plugin registry
class LoadPhysicsSystemStaticRegistryTest
    : public InternalFixture<::testing::Test> {};

TEST_F(LoadPhysicsSystemStaticRegistryTest, LoadDartsim)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfString(R"(
    <?xml version="1.0"?>
    <sdf version="1.12">
      <world name="default">
        <plugin
          filename="static://gz::sim::systems::Physics"
          name="gz::sim::systems::Physics">
          <engine>
            <filename>static://gz::physics::dartsim::Plugin</filename>
          </engine>
        </plugin>
      </world>
    </sdf>)");

  Server server(serverConfig);

  // Verify that server was initialized correctly
  auto iterationCount = server.IterationCount();
  ASSERT_NE(iterationCount, std::nullopt);
  ASSERT_EQ(*iterationCount, 0);

  // Verify that the physics system is loading the static dartsim plugin.
  auto mockSystem = std::make_shared<MockSystem>();
  mockSystem->postUpdateCallback =
      [](const sim::UpdateInfo &,
         const sim::EntityComponentManager &_ecm)
      {
        auto plugin = _ecm.ComponentData<components::PhysicsEnginePlugin>(
            worldEntity(_ecm));
        ASSERT_TRUE(plugin.has_value());
        EXPECT_EQ("static://gz::physics::dartsim::Plugin", plugin.value());
      };
  ASSERT_TRUE(server.AddSystem(mockSystem));
  server.RunOnce();
  EXPECT_EQ(1, mockSystem->postUpdateCallCount);
}
