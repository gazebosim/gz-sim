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

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/components/CenterOfVolume.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Volume.hh"

#include "ignition/gazebo/test_config.hh"
#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

class BuoyancyTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(BuoyancyTest, Movement)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_BINARY_PATH) +
    "/test/worlds/buoyancy.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::size_t iterations = 1000;

  bool finished = false;
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Check pose
    Entity submarine = _ecm.EntityByComponents(
        components::Model(), components::Name("submarine"));

    Entity submarineSinking = _ecm.EntityByComponents(
        components::Model(), components::Name("submarine_sinking"));

    Entity submarineBuoyant = _ecm.EntityByComponents(
        components::Model(), components::Name("submarine_buoyant"));

    Entity duck = _ecm.EntityByComponents(
        components::Model(), components::Name("duck"));

    ASSERT_NE(submarine, kNullEntity);
    ASSERT_NE(submarineSinking, kNullEntity);
    ASSERT_NE(submarineBuoyant, kNullEntity);
    ASSERT_NE(duck, kNullEntity);

    // Get the submarine link
    auto submarineLink = _ecm.EntityByComponents(
      components::ParentEntity(submarine),
      components::Name("body"),
      components::Link());

    // Check the submarine buoyant volume and center of volume
    auto submarineVolume = _ecm.Component<components::Volume>(
        submarineLink);
    ASSERT_NE(submarineVolume , nullptr);
    EXPECT_NEAR(0.25132741228718347, submarineVolume->Data(), 1e-3);

    auto submarineCenterOfVolume =
      _ecm.Component<components::CenterOfVolume>(submarineLink);
    ASSERT_NE(submarineCenterOfVolume, nullptr);
    EXPECT_EQ(ignition::math::Vector3d(0, 0, 0),
        submarineCenterOfVolume->Data());

    // Get the submarine buoyant link
    auto submarineBuoyantLink = _ecm.EntityByComponents(
      components::ParentEntity(submarineBuoyant),
      components::Name("body"),
      components::Link());

    // Check the submarine buoyant volume and center of volume
    auto submarineBuoyantVolume = _ecm.Component<components::Volume>(
        submarineBuoyantLink);
    ASSERT_NE(submarineBuoyantVolume , nullptr);
    EXPECT_NEAR(0.735133, submarineBuoyantVolume->Data(), 1e-3);

    auto submarineBuoyantCenterOfVolume =
      _ecm.Component<components::CenterOfVolume>(submarineBuoyantLink);
    ASSERT_NE(submarineBuoyantCenterOfVolume, nullptr);
    EXPECT_EQ(ignition::math::Vector3d(0, 0, 0),
        submarineBuoyantCenterOfVolume->Data());

    // Get the submarine sinking link
    auto submarineSinkingLink = _ecm.EntityByComponents(
      components::ParentEntity(submarineSinking),
      components::Name("body"),
      components::Link());

    // Check the submarine sinking volume and center of volume
    auto submarineSinkingVolume = _ecm.Component<components::Volume>(
        submarineSinkingLink);
    ASSERT_NE(submarineSinkingVolume , nullptr);
    EXPECT_NEAR(0.735133, submarineSinkingVolume->Data(), 1e-3);

    auto submarineSinkingCenterOfVolume =
      _ecm.Component<components::CenterOfVolume>(submarineSinkingLink);
    ASSERT_NE(submarineSinkingCenterOfVolume, nullptr);
    EXPECT_EQ(ignition::math::Vector3d(0, 0, 0),
        submarineSinkingCenterOfVolume->Data());

    // Get the duck link
    auto duckLink = _ecm.EntityByComponents(
      components::ParentEntity(duck),
      components::Name("link"),
      components::Link());

    // Check the duck volume and center of volume
    auto duckVolume = _ecm.Component<components::Volume>(duckLink);
    ASSERT_NE(duckVolume, nullptr);
    EXPECT_NEAR(1.40186, duckVolume->Data(), 1e-3);
    auto duckCenterOfVolume =
      _ecm.Component<components::CenterOfVolume>(duckLink);
    ASSERT_NE(duckCenterOfVolume, nullptr);
    EXPECT_EQ(ignition::math::Vector3d(0, 0, -0.4),
        duckCenterOfVolume->Data());

    auto submarinePose = _ecm.Component<components::Pose>(submarine);
    ASSERT_NE(submarinePose , nullptr);

    auto submarineSinkingPose = _ecm.Component<components::Pose>(
        submarineSinking);
    ASSERT_NE(submarineSinkingPose , nullptr);

    auto submarineBuoyantPose = _ecm.Component<components::Pose>(
        submarineBuoyant);
    ASSERT_NE(submarineSinkingPose , nullptr);

    auto duckPose = _ecm.Component<components::Pose>(duck);
    ASSERT_NE(duckPose , nullptr);

    // The "submarine" should stay in its starting location of 0, 0, 1.5 meters.
    EXPECT_NEAR(0, submarinePose->Data().Pos().X(), 1e-2);
    EXPECT_NEAR(0, submarinePose->Data().Pos().Y(), 1e-2);
    EXPECT_NEAR(0, submarinePose->Data().Pos().Z(), 1e-2);

    if (_info.iterations > 10)
    {
      EXPECT_LT(submarineSinkingPose->Data().Pos().Z(),
                submarinePose->Data().Pos().Z());
      EXPECT_GT(submarineBuoyantPose->Data().Pos().Z(),
                submarinePose->Data().Pos().Z());
      EXPECT_GT(duckPose->Data().Pos().Z(),
                submarinePose->Data().Pos().Z());
    }

    if (_info.iterations == iterations)
    {
      EXPECT_NEAR(-1.63, submarineSinkingPose->Data().Pos().Z(), 1e-2);
      EXPECT_NEAR(4.90, submarineBuoyantPose->Data().Pos().Z(), 1e-2);
      EXPECT_NEAR(171.4, duckPose->Data().Pos().Z(), 1e-2);
      finished = true;
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, iterations, false);
  EXPECT_TRUE(finished);
}
