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
#include <ignition/common/Console.hh>
#include <sdf/Root.hh>

#include "ignition/gazebo/test_config.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;

class SimulationRunnerTest : public ::testing::TestWithParam<int>
{
};

/////////////////////////////////////////////////
TEST_P(SimulationRunnerTest, CreateEntities)
{
  // Load SDF file
  sdf::Root root;
  root.Load(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  ASSERT_EQ(1u, root.WorldCount());

  // Create simulation runner
  SimulationRunner runner(root.WorldByIndex(0));

  // Check component types
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::World>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Model>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Link>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Collision>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Visual>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::Name>()));
  EXPECT_TRUE(runner.entityCompMgr.HasComponentType(
      EntityComponentManager::ComponentType<components::ParentEntity>()));

  // Check entities
  EXPECT_EQ(13u, runner.entityCompMgr.EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  EntityId worldEntity = kNullEntity;
  runner.entityCompMgr.Each<components::World,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::World *_world,
        const components::Name *_name)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_world)
        return;
      ASSERT_NE(nullptr, _world);
      ASSERT_NE(nullptr, _name);

      EXPECT_EQ("default", _name->Data());

      worldCount++;

      worldEntity = _entity;
    });

  EXPECT_EQ(1u, worldCount);
  EXPECT_NE(kNullEntity, worldEntity);

  // Check models
  unsigned int modelCount{0};
  EntityId boxModelEntity = kNullEntity;
  EntityId cylModelEntity = kNullEntity;
  EntityId sphModelEntity = kNullEntity;
  runner.entityCompMgr.Each<components::Model,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::Model *_model,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_model)
        return;

      ASSERT_NE(nullptr, _model);
      ASSERT_NE(nullptr, _pose);
      ASSERT_NE(nullptr, _parent);
      ASSERT_NE(nullptr, _name);

      modelCount++;

      EXPECT_EQ(worldEntity, _parent->Id());
      if (modelCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("box", _name->Data());
        boxModelEntity = _entity;
      }
      else if (modelCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(-1, -2, -3, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("cylinder", _name->Data());
        cylModelEntity = _entity;
      }
      else if (modelCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0, 0, 0, 0, 0, 1),
            _pose->Data());
        EXPECT_EQ("sphere", _name->Data());
        sphModelEntity = _entity;
      }
    });

  EXPECT_EQ(3u, modelCount);
  EXPECT_NE(kNullEntity, boxModelEntity);
  EXPECT_NE(kNullEntity, cylModelEntity);
  EXPECT_NE(kNullEntity, sphModelEntity);

  // Check links
  unsigned int linkCount{0};
  EntityId boxLinkEntity = kNullEntity;
  EntityId cylLinkEntity = kNullEntity;
  EntityId sphLinkEntity = kNullEntity;
  runner.entityCompMgr.Each<components::Link,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &_entity,
        const components::Link *_link,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_link)
        return;

      ASSERT_NE(nullptr, _link);
      ASSERT_NE(nullptr, _pose);
      ASSERT_NE(nullptr, _parent);
      ASSERT_NE(nullptr, _name);

      linkCount++;

      if (linkCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.1, 0.1, 0.1, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("box_link", _name->Data());
        EXPECT_EQ(boxModelEntity, _parent->Id());
        boxLinkEntity = _entity;
      }
      else if (linkCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.2, 0.2, 0.2, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("cylinder_link", _name->Data());
        EXPECT_EQ(cylModelEntity, _parent->Id());
        cylLinkEntity = _entity;
      }
      else if (linkCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.3, 0.3, 0.3, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("sphere_link", _name->Data());
        EXPECT_EQ(sphModelEntity, _parent->Id());
        sphLinkEntity = _entity;
      }
    });

  EXPECT_EQ(3u, linkCount);
  EXPECT_NE(kNullEntity, boxLinkEntity);
  EXPECT_NE(kNullEntity, cylLinkEntity);
  EXPECT_NE(kNullEntity, sphLinkEntity);

  // Check collisions
  unsigned int collisionCount{0};
  runner.entityCompMgr.Each<components::Collision,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &/*_entity*/,
        const components::Collision *_collision,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_collision)
        return;

      ASSERT_NE(nullptr, _collision);
      ASSERT_NE(nullptr, _pose);
      ASSERT_NE(nullptr, _parent);
      ASSERT_NE(nullptr, _name);

      collisionCount++;

      if (collisionCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.11, 0.11, 0.11, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("box_collision", _name->Data());
        EXPECT_EQ(boxLinkEntity, _parent->Id());
      }
      else if (collisionCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.21, 0.21, 0.21, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("cylinder_collision", _name->Data());
        EXPECT_EQ(cylLinkEntity, _parent->Id());
      }
      else if (collisionCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.31, 0.31, 0.31, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("sphere_collision", _name->Data());
        EXPECT_EQ(sphLinkEntity, _parent->Id());
      }
    });

  EXPECT_EQ(3u, collisionCount);

  // Check visuals
  unsigned int visualCount{0};
  runner.entityCompMgr.Each<components::Visual,
                            components::Pose,
                            components::ParentEntity,
                            components::Name>(
    [&](const EntityId &/*_entity*/,
        const components::Visual *_visual,
        const components::Pose *_pose,
        const components::ParentEntity *_parent,
        const components::Name *_name)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_visual)
        return;

      ASSERT_NE(nullptr, _visual);
      ASSERT_NE(nullptr, _pose);
      ASSERT_NE(nullptr, _parent);
      ASSERT_NE(nullptr, _name);

      visualCount++;

      if (visualCount == 1)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.12, 0.12, 0.12, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("box_visual", _name->Data());
        EXPECT_EQ(boxLinkEntity, _parent->Id());
      }
      else if (visualCount == 2)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.22, 0.22, 0.22, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("cylinder_visual", _name->Data());
        EXPECT_EQ(cylLinkEntity, _parent->Id());
      }
      else if (visualCount == 3)
      {
        EXPECT_EQ(ignition::math::Pose3d(0.32, 0.32, 0.32, 0, 0, 0),
            _pose->Data());
        EXPECT_EQ("sphere_visual", _name->Data());
        EXPECT_EQ(sphLinkEntity, _parent->Id());
      }
    });

  EXPECT_EQ(3u, visualCount);
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(ServerRepeat, SimulationRunnerTest, ::testing::Range(1, 2));
