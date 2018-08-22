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

  // Check entities
  EXPECT_EQ(13u, runner.entityCompMgr.EntityCount());

  // Check worlds
  unsigned int worldCount{0};
  runner.entityCompMgr.Each<components::World>(
    [&](const EntityId &/*_entity*/, const components::World *_world)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_world)
        return;
      ASSERT_NE(nullptr, _world);

      worldCount++;
    });

  EXPECT_EQ(1u, worldCount);

  // Check models
  unsigned int modelCount{0};
  runner.entityCompMgr.Each<components::Model>(
    [&](const EntityId &/*_entity*/, const components::Model *_model)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_model)
        return;
      ASSERT_NE(nullptr, _model);

      modelCount++;
    });

  EXPECT_EQ(3u, modelCount);

  // Check links
  unsigned int linkCount{0};
  runner.entityCompMgr.Each<components::Link>(
    [&](const EntityId &/*_entity*/, const components::Link *_link)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_link)
        return;
      ASSERT_NE(nullptr, _link);

      linkCount++;
    });

  EXPECT_EQ(3u, modelCount);

  // Check collisions
  unsigned int collisionCount{0};
  runner.entityCompMgr.Each<components::Collision>(
    [&](const EntityId &/*_entity*/, const components::Collision *_collision)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_collision)
        return;
      ASSERT_NE(nullptr, _collision);

      collisionCount++;
    });

  EXPECT_EQ(3u, collisionCount);

  // Check visuals
  unsigned int visualCount{0};
  runner.entityCompMgr.Each<components::Visual>(
    [&](const EntityId &/*_entity*/, const components::Visual *_visual)
    {
      // \todo(louise) Fix Each so it only returns matching entities
      if (!_visual)
        return;
      ASSERT_NE(nullptr, _visual);

      visualCount++;
    });

  EXPECT_EQ(3u, visualCount);
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(ServerRepeat, SimulationRunnerTest, ::testing::Range(1, 2));
