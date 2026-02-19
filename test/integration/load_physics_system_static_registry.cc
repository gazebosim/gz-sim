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

#include <optional>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>

#include "../helpers/EnvTestFixture.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
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
        <model name="test_model">
          <link name="link_1">
            <collision name="collision">
              <geometry>
                <box>
                  <size>1 1 1</size>
                </box>
              </geometry>
            </collision>
          </link>
        </model>
      </world>
    </sdf>)");

  Server server(serverConfig);

  // Verify that server was initialized correctly
  auto iterationCount = server.IterationCount();
  ASSERT_NE(iterationCount, std::nullopt);
  ASSERT_EQ(*iterationCount, 0);

  std::optional<Entity> modelId = server.EntityByName("test_model");
  ASSERT_NE(modelId, std::nullopt);

  EntityComponentManager *ecm{nullptr};
  auto mockSystem = std::make_shared<MockSystem>();
  mockSystem->preUpdateCallback =
    [&ecm](const UpdateInfo &, EntityComponentManager &_ecm)
    {
      ecm = &_ecm;
    };
  ASSERT_TRUE(server.AddSystem(mockSystem));

  server.RunOnce();
  EXPECT_EQ(1, mockSystem->preUpdateCallCount);

  // Verify that the physics system loaded the static dartsim plugin.
  auto plugin = ecm->ComponentData<components::PhysicsEnginePlugin>(
      worldEntity(*ecm));
  ASSERT_TRUE(plugin.has_value());
  EXPECT_EQ("static://gz::physics::dartsim::Plugin", plugin.value());

  // Verify that physics engine is running by checking that
  // the link is falling and has negative world linear velocity.
  const std::string linkName{"link_1"};
  auto linkEntity = ecm->EntityByComponents(
      components::Link(), components::Name(linkName));
  sim::Link link(linkEntity);
  link.EnableVelocityChecks(*ecm, true);
  server.RunOnce(false);
  std::optional<math::Pose3d> linkPose = link.WorldPose(*ecm);
  ASSERT_TRUE(linkPose.has_value());
  EXPECT_GT(0, linkPose->Pos().Z());
  std::optional<math::Vector3d> linkLinearVel = link.WorldLinearVelocity(*ecm);
  ASSERT_TRUE(linkLinearVel.has_value());
  EXPECT_GT(0, linkLinearVel->Z());
}
