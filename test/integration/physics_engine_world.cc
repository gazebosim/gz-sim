/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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

#include <string>

#include <gz/common/Util.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "../helpers/EnvTestFixture.hh"
#include "../plugins/MockPhysicsEngine.hh"

using namespace gz;
using namespace sim;

/// \brief Test what the Physics system hands over to a physics engine plugin.
class PhysicsEngineWorldTest : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    // Let the Physics system find the MockPhysicsEngine plugin.
    common::setenv("GZ_SIM_PHYSICS_ENGINE_PATH",
        common::joinPaths(std::string(PROJECT_BINARY_PATH), "lib").c_str());
  }
};

/////////////////////////////////////////////////
// The sdf::World that the Physics system constructs in the engine has the
// <physics> element of the world, so engine plugins can read their own settings
// from it.
TEST_F(PhysicsEngineWorldTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(WorldPhysicsReachesEngine))
{
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "physics_engine_world.sdf"));

  Server server(serverConfig);
  server.Run(true, 1, false);

  const auto &worlds = test::MockPhysicsEngineWorlds();
  ASSERT_EQ(1u, worlds.size());
  const auto &world = worlds[0];

  // These were already passed before the physics element was.
  EXPECT_EQ("physics_engine_world", world.name);
  EXPECT_EQ(math::Vector3d(0, 0, -3.5), world.gravity);

  ASSERT_EQ(1u, world.physicsCount);
  EXPECT_EQ("mock_physics", world.physicsName);
  EXPECT_DOUBLE_EQ(0.002, world.maxStepSize);
  EXPECT_DOUBLE_EQ(0.0, world.realTimeFactor);
  EXPECT_TRUE(world.hasCustomElement);
  EXPECT_EQ("engine_setting", world.customElementValue);
}
