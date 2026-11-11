/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <cstddef>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/math/Pose3.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Model.hh"

#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class NestedModelPhysicsTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
/// Test that a tower of 3 boxes built with an <include> and further nesting
/// moves appropriately with joints in dartsim
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(NestedModelPhysicsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Movement))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/include_connected_nested_models.sdf";
  std::string path = std::string(PROJECT_SOURCE_PATH) + "/test/worlds/models";
  gz::common::setenv("GZ_SIM_RESOURCE_PATH", path.c_str());
  serverConfig.SetResourceCache(path);
  serverConfig.SetPhysicsEngine("gz-physics-dartsim-plugin");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ms);

  constexpr std::size_t kIterations = 1000;
  constexpr double kEpsilon = 1e-2;
  // base_link does not move in this world
  const math::Pose3d expectedBaseLinkPose(0, 0, 0.5, 0, 0, 0);
  // link_01 starts 2m from link_00, which sits 3m above its model origin
  const math::Pose3d expectedLink01StartPose(0, 2, 3, 0, 0, 0);

  auto expectBaseLinkStatic = [&](const EntityComponentManager &_ecm,
                                  Entity _baseLink)
  {
    auto baseLinkPose = _ecm.Component<components::Pose>(_baseLink);
    ASSERT_NE(nullptr, baseLinkPose);
    EXPECT_NEAR(expectedBaseLinkPose.X(), baseLinkPose->Data().Pos().X(),
        kEpsilon);
    EXPECT_NEAR(expectedBaseLinkPose.Y(), baseLinkPose->Data().Pos().Y(),
        kEpsilon);
    EXPECT_NEAR(expectedBaseLinkPose.Z(), baseLinkPose->Data().Pos().Z(),
        kEpsilon);
  };

  // Run one iteration so that the physics entities are created
  server.Run(true, 1, false);

  Entity baseLink{kNullEntity};
  Entity link01{kNullEntity};
  {
    auto ecm = server.EcmScope();
    ASSERT_TRUE(ecm.Valid());

    // Get the top level model
    EXPECT_NE(kNullEntity, ecm->EntityByComponents(
        components::Name("include_connected_nested_new_name"),
        components::Model()));

    baseLink = ecm->EntityByComponents(
        components::Link(), components::Name("base_link"));
    ASSERT_NE(kNullEntity, baseLink);
    link01 = ecm->EntityByComponents(
        components::Link(), components::Name("link_01"));
    ASSERT_NE(kNullEntity, link01);

    expectBaseLinkStatic(*ecm, baseLink);

    // link_01 is still at its initial pose
    auto link01Pose = ecm->Component<components::Pose>(link01);
    ASSERT_NE(nullptr, link01Pose);
    EXPECT_NEAR(expectedLink01StartPose.X(), link01Pose->Data().Pos().X(),
        kEpsilon);
    EXPECT_NEAR(expectedLink01StartPose.Y(), link01Pose->Data().Pos().Y(),
        kEpsilon);
    EXPECT_NEAR(expectedLink01StartPose.Z(), link01Pose->Data().Pos().Z(),
        kEpsilon);
  }

  server.Run(true, kIterations - 1, false);
  {
    auto ecm = server.EcmScope();
    ASSERT_TRUE(ecm.Valid());

    expectBaseLinkStatic(*ecm, baseLink);

    auto link01Pose = ecm->Component<components::Pose>(link01);
    ASSERT_NE(nullptr, link01Pose);
    EXPECT_NEAR(expectedLink01StartPose.X(), link01Pose->Data().Pos().X(),
        kEpsilon);
    // Rough approximation on its whereabouts after 1 second
    EXPECT_NEAR(-0.36, link01Pose->Data().Pos().Y(), 0.1);
    EXPECT_NEAR(1.03, link01Pose->Data().Pos().Z(), 0.1);
  }
}
