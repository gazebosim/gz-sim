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

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>

#include "ignition/math/Pose3.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Model.hh"

#include "ignition/gazebo/test_config.hh"
#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;

class NestedModelPhysicsTest : public ::testing::Test
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
/// Test that a tower of 3 boxes built with an <include> and further nesting
/// moves appropriately with joints in dartsim
TEST_F(NestedModelPhysicsTest, Movement)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/include_connected_nested_models.sdf";
  std::string path = std::string(PROJECT_SOURCE_PATH) + "/test/worlds/models";
  ignition::common::setenv("IGN_GAZEBO_RESOURCE_PATH", path.c_str());
  serverConfig.SetResourceCache(path);
  serverConfig.SetPhysicsEngine("libignition-physics-dartsim-plugin.so");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ms);

  std::size_t iterations = 1000;

  bool finished = false;
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Check pose
    Entity baseLink = _ecm.EntityByComponents(
        components::Link(),
        components::Name("base_link"));
    ASSERT_NE(baseLink, kNullEntity);

    Entity link01 = _ecm.EntityByComponents(
        components::Link(), components::Name("link_01"));
    ASSERT_NE(link01, kNullEntity);

    // Get the top level model
    auto topModel = _ecm.EntityByComponents(
      components::Name(
        "include_connected_nested_new_name"),
      components::Model());
    ASSERT_NE(topModel, kNullEntity);

    auto baseLinkPose = _ecm.Component<components::Pose>(baseLink);
    ASSERT_NE(baseLinkPose , nullptr);

    auto link01Pose = _ecm.Component<components::Pose>(link01);
    ASSERT_NE(link01Pose , nullptr);

    constexpr double epsilon = 1e-2;
    // base_link does not move in this world
    const ignition::math::Pose3d expectedBazeLinkPose(0, 0, 0.5, 0, 0, 0);
    const ignition::math::Pose3d expectedLink01StartPose(0, 2, 0, 0, 0, 0);

    EXPECT_NEAR(
      expectedBazeLinkPose.X(), baseLinkPose->Data().Pos().X(), epsilon);
    EXPECT_NEAR(
      expectedBazeLinkPose.Y(), baseLinkPose->Data().Pos().Y(), epsilon);
    EXPECT_NEAR(
      expectedBazeLinkPose.Z(), baseLinkPose->Data().Pos().Z(), epsilon);

    if (_info.iterations == 0)
    {
      EXPECT_NEAR(
        expectedLink01StartPose.X(), link01Pose->Data().Pos().X(), epsilon);
      EXPECT_NEAR(
        expectedLink01StartPose.Y(), link01Pose->Data().Pos().Y(), epsilon);
      EXPECT_NEAR(
        expectedLink01StartPose.Z(), link01Pose->Data().Pos().Z(), epsilon);
    }
    else if (_info.iterations == iterations)
    {
      EXPECT_NEAR(
        expectedLink01StartPose.X(), link01Pose->Data().Pos().X(), epsilon);

      // Rough approximation on its whereabouts after 1 second
      EXPECT_NEAR(-0.36, link01Pose->Data().Pos().Y(), 0.1);
      EXPECT_NEAR(1.03, link01Pose->Data().Pos().Z(), 0.1);
      finished = true;
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, iterations, false);
  EXPECT_TRUE(finished);
}
