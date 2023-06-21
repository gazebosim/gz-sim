/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Actor.hh>
#include <gz/sim/components/Actor.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class ActorIntegrationTest : public InternalFixture<::testing::Test>
{
};

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, Valid)
{
  EntityComponentManager ecm;

  // No ID
  {
    Actor actor;
    EXPECT_FALSE(actor.Valid(ecm));
  }

  // Missing actor component
  {
    auto id = ecm.CreateEntity();
    Actor actor(id);
    EXPECT_FALSE(actor.Valid(ecm));
  }

  // Valid
  {
    auto id = ecm.CreateEntity();
    ecm.CreateComponent<components::Actor>(id, components::Actor());

    Actor actor(id);
    EXPECT_TRUE(actor.Valid(ecm));
  }
}

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, Name)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Actor>(id, components::Actor());

  Actor actor(id);

  // No name
  EXPECT_EQ(std::nullopt, actor.Name(ecm));

  // Add name
  ecm.CreateComponent<components::Name>(id, components::Name("actor_name"));
  EXPECT_EQ("actor_name", actor.Name(ecm));
}

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, Pose)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Actor>(id, components::Actor());

  Actor actor(id);

  // No pose
  EXPECT_EQ(std::nullopt, actor.Pose(ecm));

  // Add pose
  math::Pose3d pose(1, 2, 3, 0.1, 0.2, 0.3);
  ecm.CreateComponent<components::Pose>(id,
      components::Pose(pose));
  EXPECT_EQ(pose, actor.Pose(ecm));
}

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, WorldPose)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Actor>(id, components::Actor());

  Actor actor(id);

  // No pose
  EXPECT_EQ(std::nullopt, actor.WorldPose(ecm));

  // Add world pose
  math::Pose3d pose(0, 3, 9, 0.0, 1.2, -3.0);
  ecm.CreateComponent<components::WorldPose>(id,
      components::WorldPose(pose));
  EXPECT_EQ(pose, actor.WorldPose(ecm));
}

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, TrajectoryPose)
{
  EntityComponentManager ecm;

  auto id = ecm.CreateEntity();
  ecm.CreateComponent<components::Actor>(id, components::Actor());

  Actor actor(id);

  // No pose
  EXPECT_EQ(std::nullopt, actor.TrajectoryPose(ecm));

  // Add pose
  math::Pose3d pose(3, 42, 35, 2.1, 3.2, 1.3);
  ecm.CreateComponent<components::TrajectoryPose>(id,
      components::TrajectoryPose(pose));
  EXPECT_EQ(pose, actor.TrajectoryPose(ecm));
}

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, SetTrajectoryPose)
{
  EntityComponentManager ecm;

  auto eActor = ecm.CreateEntity();
  ecm.CreateComponent(eActor, components::Actor());

  Actor actor(eActor);
  EXPECT_EQ(eActor, actor.Entity());

  ASSERT_TRUE(actor.Valid(ecm));

  // No TrajectoryPose should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::TrajectoryPose>(eActor));

  math::Pose3d pose(0.1, 2.3, 4.7, 0, 0, 1.0);
  actor.SetTrajectoryPose(ecm, pose);

  // trajectory pose should exist
  EXPECT_NE(nullptr, ecm.Component<components::TrajectoryPose>(eActor));
  EXPECT_EQ(pose,
    ecm.Component<components::TrajectoryPose>(eActor)->Data());

  // Make sure the trajectory pose is updated
  math::Pose3d pose2(1.0, 3.2, 7.4, 1.0, 0, 0.0);
  actor.SetTrajectoryPose(ecm, pose2);
  EXPECT_EQ(pose2,
    ecm.Component<components::TrajectoryPose>(eActor)->Data());
}

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, SetAnimationName)
{
  EntityComponentManager ecm;

  auto eActor = ecm.CreateEntity();
  ecm.CreateComponent(eActor, components::Actor());

  Actor actor(eActor);
  EXPECT_EQ(eActor, actor.Entity());

  ASSERT_TRUE(actor.Valid(ecm));

  // No AnimationName should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::AnimationName>(eActor));

  std::string animName = "animation_name";
  actor.SetAnimationName(ecm, animName);

  // animation name should exist
  EXPECT_NE(nullptr, ecm.Component<components::AnimationName>(eActor));
  EXPECT_EQ(animName,
    ecm.Component<components::AnimationName>(eActor)->Data());

  // Make sure the animation name is updated
  std::string animName2 = "animation_name2";
  actor.SetAnimationName(ecm, animName2);
  EXPECT_EQ(animName2,
    ecm.Component<components::AnimationName>(eActor)->Data());
}

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, SetAnimationTime)
{
  EntityComponentManager ecm;

  auto eActor = ecm.CreateEntity();
  ecm.CreateComponent(eActor, components::Actor());

  Actor actor(eActor);
  EXPECT_EQ(eActor, actor.Entity());

  ASSERT_TRUE(actor.Valid(ecm));

  // No AnimationTime should exist by default
  EXPECT_EQ(nullptr, ecm.Component<components::AnimationTime>(eActor));

  std::chrono::steady_clock::duration animTime = std::chrono::milliseconds(30);
  actor.SetAnimationTime(ecm, animTime);

  // animation time should exist
  EXPECT_NE(nullptr, ecm.Component<components::AnimationTime>(eActor));
  EXPECT_EQ(animTime,
    ecm.Component<components::AnimationTime>(eActor)->Data());

  // Make sure the animation time is updated
  std::chrono::steady_clock::duration animTime2 = std::chrono::milliseconds(70);
  actor.SetAnimationTime(ecm, animTime2);
  EXPECT_EQ(animTime2,
    ecm.Component<components::AnimationTime>(eActor)->Data());
}

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, AnimationName)
{
  EntityComponentManager ecm;

  auto eActor = ecm.CreateEntity();
  ecm.CreateComponent(eActor, components::Actor());

  Actor actor(eActor);
  EXPECT_EQ(eActor, actor.Entity());

  ASSERT_TRUE(actor.Valid(ecm));

  // animation name should return nullopt by default
  EXPECT_EQ(std::nullopt, actor.AnimationName(ecm));

  // get animation name
  std::string animName = "animation_name";
  ecm.SetComponentData<components::AnimationName>(eActor, animName);
  EXPECT_EQ(animName, actor.AnimationName(ecm));
}

//////////////////////////////////////////////////
TEST_F(ActorIntegrationTest, AnimationTime)
{
  EntityComponentManager ecm;

  auto eActor = ecm.CreateEntity();
  ecm.CreateComponent(eActor, components::Actor());

  Actor actor(eActor);
  EXPECT_EQ(eActor, actor.Entity());

  ASSERT_TRUE(actor.Valid(ecm));

  // animation time should return nullopt by default
  EXPECT_EQ(std::nullopt, actor.AnimationTime(ecm));

  // get animation time
  std::chrono::steady_clock::duration animTime = std::chrono::milliseconds(60);
  ecm.CreateComponent<components::AnimationTime>(eActor,
      components::AnimationTime(animTime));
  EXPECT_EQ(animTime, actor.AnimationTime(ecm));
}
