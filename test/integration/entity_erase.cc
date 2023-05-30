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

#include <gtest/gtest.h>

#include <optional>

#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"  // NOLINT(build/include)
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

class PhysicsSystemFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(CreatePhysicsWorld))
{
  ServerConfig serverConfig;

  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/shapes.sdf");

  Server server(serverConfig);
  EXPECT_TRUE(server.HasEntity("box"));
  EXPECT_TRUE(server.HasEntity("capsule"));
  EXPECT_TRUE(server.HasEntity("cylinder"));
  EXPECT_TRUE(server.HasEntity("ellipsoid"));
  EXPECT_TRUE(server.HasEntity("sphere"));
  server.SetUpdatePeriod(1ns);

  // Remove the entity.
  EXPECT_TRUE(server.RequestRemoveEntity("box"));
  // Nothing changes because the server has not been stepped.
  EXPECT_TRUE(server.HasEntity("box"));
  EXPECT_TRUE(server.HasEntity("capsule"));
  EXPECT_TRUE(server.HasEntity("cylinder"));
  EXPECT_TRUE(server.HasEntity("ellipsoid"));
  EXPECT_TRUE(server.HasEntity("sphere"));
  // Take one step and the entity should be removed.
  server.Run(true, 1, false);
  EXPECT_FALSE(server.HasEntity("box"));
  EXPECT_TRUE(server.HasEntity("capsule"));
  EXPECT_TRUE(server.HasEntity("cylinder"));
  EXPECT_TRUE(server.HasEntity("ellipsoid"));
  EXPECT_TRUE(server.HasEntity("sphere"));

  EXPECT_TRUE(server.RequestRemoveEntity("cylinder"));
  server.Run(true, 1, false);
  EXPECT_FALSE(server.HasEntity("box"));
  EXPECT_FALSE(server.HasEntity("cylinder"));
  EXPECT_TRUE(server.HasEntity("capsule"));
  EXPECT_TRUE(server.HasEntity("ellipsoid"));
  std::optional<Entity> entity = server.EntityByName("sphere");
  EXPECT_NE(std::nullopt, entity);
  EXPECT_TRUE(server.RequestRemoveEntity(*entity));

  server.Run(true, 1, false);
  EXPECT_FALSE(server.HasEntity("box"));
  EXPECT_FALSE(server.HasEntity("cylinder"));
  EXPECT_TRUE(server.HasEntity("capsule"));
  EXPECT_TRUE(server.HasEntity("ellipsoid"));
  EXPECT_FALSE(server.HasEntity("sphere"));
}
