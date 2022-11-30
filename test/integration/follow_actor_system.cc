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
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

#define tol 10e-4

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test FollowActor system
class FollowActorTest : public InternalFixture<::testing::TestWithParam<int>>
{
};

class Relay
{
  public: Relay()
  {
    sdf::Plugin sdfPlugin;
    sdfPlugin.SetFilename("libMockSystem.so");
    sdfPlugin.SetName("gz::sim::MockSystem");
    auto plugin = loader.LoadPlugin(sdfPlugin);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem = static_cast<MockSystem *>(
        systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: Relay &OnPreUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnPostUpdate(MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: SystemPluginPtr systemPtr;

  private: SystemLoader loader;
  private: MockSystem *mockSystem;
};


/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_P(FollowActorTest,
       GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(PublishCmd))
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/follow_actor.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  unsigned int preUpdateCount{0};
  unsigned int postUpdateCount{0};
  unsigned int boxMoveCount{0};

  // Create a system that records the actor poses
  Relay testSystem;
  testSystem.OnPreUpdate(
    [&](const sim::UpdateInfo &_info,
        sim::EntityComponentManager &_ecm)
    {
      auto entity = _ecm.EntityByComponents(
        components::Name("box"));
      EXPECT_NE(kNullEntity, entity);

      auto poseComp = _ecm.Component<components::Pose>(entity);
      ASSERT_NE(nullptr, poseComp);

      // Move box every 500 ms
      if (_info.iterations % 500 == 0)
      {
        auto pose = poseComp->Data();
        pose.Pos().X() += 0.5;
        poseComp->SetData(pose,
            [](const math::Pose3d &, const math::Pose3d &){return true;});
        boxMoveCount++;
      }

      preUpdateCount++;
      EXPECT_EQ(_info.iterations, preUpdateCount);
    });

  testSystem.OnPostUpdate(
    [&](const sim::UpdateInfo &_info,
        const sim::EntityComponentManager &_ecm)
    {
      auto actorEntity = _ecm.EntityByComponents(
        components::Name("walker"));
      EXPECT_NE(kNullEntity, actorEntity);

      auto targetEntity = _ecm.EntityByComponents(
        components::Name("box"));
      EXPECT_NE(kNullEntity, targetEntity);

      auto animNameComp =
          _ecm.Component<components::AnimationName>(actorEntity);
      ASSERT_NE(nullptr, animNameComp);
      EXPECT_EQ("walking", animNameComp->Data());

      // Animation time is always increasing
      auto animTimeComp =
          _ecm.Component<components::AnimationTime>(actorEntity);
      ASSERT_NE(nullptr, animTimeComp);

      // Actor pose is fixed to zero X/Y and initial Z
      auto actorPoseComp = _ecm.Component<components::Pose>(actorEntity);
      ASSERT_NE(nullptr, actorPoseComp);
      EXPECT_EQ(math::Pose3d(0, 0, 1, 0, 0, 0), actorPoseComp->Data());

      // Target is moving
      auto targetPoseComp = _ecm.Component<components::Pose>(targetEntity);
      ASSERT_NE(nullptr, targetPoseComp);
      EXPECT_DOUBLE_EQ(1.0 + static_cast<int>(_info.iterations / 500) * 0.5,
          targetPoseComp->Data().Pos().X());

      // Actor trajectory pose X increases to follow target
      auto actorTrajPoseComp = _ecm.Component<components::TrajectoryPose>(
          actorEntity);
      ASSERT_NE(nullptr, actorTrajPoseComp);

      // Actor is behind box, respecting min_distance and never staying too far
      auto diff = targetPoseComp->Data().Pos() -
                  actorTrajPoseComp->Data().Pos();
      EXPECT_GE(diff.X(), 1.0) << _info.iterations;
      EXPECT_LE(diff.X(), 1.8) << _info.iterations;

      postUpdateCount++;
      EXPECT_EQ(_info.iterations, postUpdateCount);
    });
  server.AddSystem(testSystem.systemPtr);

  unsigned int iterations{400};
  server.Run(true /* blocking */, iterations, false /* paused */);
  EXPECT_EQ(iterations, preUpdateCount);
  EXPECT_EQ(iterations, postUpdateCount);
  EXPECT_EQ(iterations / 500, boxMoveCount);
}

// Run multiple times
INSTANTIATE_TEST_SUITE_P(ServerRepeat, FollowActorTest,
    ::testing::Range(1, 2));
