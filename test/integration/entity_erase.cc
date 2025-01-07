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

#include "gz/sim/components/DetachableJoint.hh"
#include "gz/sim/Server.hh"
#include "test_config.hh"  // NOLINT(build/include)
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

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

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(PhysicsSystemFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(RemoveModelWithDetachableJoints))
{
  ServerConfig serverConfig;

  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "examples", "worlds", "detachable_joint.sdf"));

  Server server(serverConfig);

  unsigned int detachableJointCount = 0u;
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const UpdateInfo &,
    const EntityComponentManager &_ecm)
    {
      _ecm.Each<components::DetachableJoint>(
              [&](const Entity &,
                  const components::DetachableJoint *) -> bool
              {
                detachableJointCount++;
                return true;
              });
    });
  server.AddSystem(testSystem.systemPtr);

  server.Run(true, 1, false);
  EXPECT_TRUE(server.HasEntity("vehicle_blue"));
  EXPECT_TRUE(server.HasEntity("chassis"));
  EXPECT_TRUE(server.HasEntity("front_left_wheel"));
  EXPECT_TRUE(server.HasEntity("front_right_wheel"));
  EXPECT_TRUE(server.HasEntity("rear_left_wheel"));
  EXPECT_TRUE(server.HasEntity("rear_right_wheel"));
  EXPECT_TRUE(server.HasEntity("front_left_wheel_joint"));
  EXPECT_TRUE(server.HasEntity("front_right_wheel_joint"));
  EXPECT_TRUE(server.HasEntity("rear_left_wheel_joint"));
  EXPECT_TRUE(server.HasEntity("rear_right_wheel_joint"));
  EXPECT_TRUE(server.HasEntity("B1"));
  EXPECT_TRUE(server.HasEntity("B2"));
  EXPECT_TRUE(server.HasEntity("B3"));
  EXPECT_EQ(3u, detachableJointCount);

  EXPECT_TRUE(server.RequestRemoveEntity("vehicle_blue"));
  server.Run(true, 1, false);

  detachableJointCount = 0u;
  server.Run(true, 1, false);
  EXPECT_FALSE(server.HasEntity("vehicle_blue"));
  EXPECT_FALSE(server.HasEntity("chassis"));
  EXPECT_FALSE(server.HasEntity("front_left_wheel"));
  EXPECT_FALSE(server.HasEntity("front_right_wheel"));
  EXPECT_FALSE(server.HasEntity("rear_left_wheel"));
  EXPECT_FALSE(server.HasEntity("rear_right_wheel"));
  EXPECT_FALSE(server.HasEntity("front_left_wheel_joint"));
  EXPECT_FALSE(server.HasEntity("front_right_wheel_joint"));
  EXPECT_FALSE(server.HasEntity("rear_left_wheel_joint"));
  EXPECT_FALSE(server.HasEntity("rear_right_wheel_joint"));
  EXPECT_TRUE(server.HasEntity("B1"));
  EXPECT_TRUE(server.HasEntity("B2"));
  EXPECT_TRUE(server.HasEntity("B3"));
  EXPECT_EQ(0u, detachableJointCount);
}
