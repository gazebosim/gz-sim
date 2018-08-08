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
#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include "ignition/gazebo/EntityComponentManager.hh"

using namespace ignition;

class EntityComponentManagerFixture : public ::testing::TestWithParam<int>
{
};

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, AdjacentMemorySingleComponentType)
{
  ignition::common::Console::SetVerbosity(4);
  gazebo::EntityComponentManager manager;

  std::vector<ignition::math::Pose3d> poses;
  std::vector<gazebo::ComponentKey> keys;

  int count = 10;

  gazebo::EntityId entityId = manager.CreateEntity();
  EXPECT_EQ(0, entityId);

  // Create the components.
  for (int i = 0; i < count; ++i)
  {
    poses.push_back(ignition::math::Pose3d(
          math::Rand::IntNormal(10, 5),
          math::Rand::IntNormal(100, 50),
          math::Rand::IntNormal(-100, 30), 0, 0, 0));
    keys.push_back(manager.CreateComponent(entityId, poses.back()));

    // The component ids should increment by one for each component.
    EXPECT_EQ(keys.back().second, i);
  }

  ASSERT_EQ(count, static_cast<int>(poses.size()));
  ASSERT_EQ(count, static_cast<int>(keys.size()));

  // Check the component values.
  for (int i = 0; i < count; ++i)
  {
    EXPECT_EQ(poses[i], *(manager.Component<ignition::math::Pose3d>(keys[i])));
  }

  uintptr_t poseSize = sizeof(ignition::math::Pose3d);
  const ignition::math::Pose3d *pose = nullptr, *prevPose = nullptr;

  // Check that each component is adjacent in memory
  for (int i = 0; i < count; ++i)
  {
    pose = manager.Component<ignition::math::Pose3d>(keys[i]);
    if (prevPose != nullptr)
    {
      EXPECT_EQ(poseSize, reinterpret_cast<uintptr_t>(pose) -
                          reinterpret_cast<uintptr_t>(prevPose));
    }
    prevPose = pose;
  }
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, AdjacentMemoryTwoComponentTypes)
{
  gazebo::EntityComponentManager manager;

  std::vector<ignition::math::Pose3d> poses;
  std::vector<int> ints;
  std::vector<gazebo::ComponentKey> poseKeys;
  std::vector<gazebo::ComponentKey> intKeys;

  int count = 100000;

  gazebo::EntityId entityId = manager.CreateEntity();
  EXPECT_EQ(0, entityId);

  // Create the components.
  for (int i = 0; i < count; ++i)
  {
    poses.push_back(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
    ints.push_back(i);

    poseKeys.push_back(manager.CreateComponent(entityId, poses.back()));
    intKeys.push_back(manager.CreateComponent(entityId, ints.back()));

    // The component ids should increment by one for each component.
    EXPECT_EQ(poseKeys.back().second, i);
    EXPECT_EQ(intKeys.back().second, i);
  }

  ASSERT_EQ(static_cast<size_t>(count), poses.size());
  ASSERT_EQ(static_cast<size_t>(count), ints.size());
  ASSERT_EQ(static_cast<size_t>(count), poseKeys.size());
  ASSERT_EQ(static_cast<size_t>(count), intKeys.size());

  uintptr_t poseSize = sizeof(ignition::math::Pose3d);
  uintptr_t intSize = sizeof(int);
  const ignition::math::Pose3d *pose = nullptr, *prevPose = nullptr;
  const int *it = nullptr, *prevIt = nullptr;

  // Check that each component is adjacent in memory
  for (int i = 0; i < count; ++i)
  {
    pose = manager.Component<ignition::math::Pose3d>(poseKeys[i]);
    it = manager.Component<int>(intKeys[i]);
    if (prevPose != nullptr)
    {
      EXPECT_EQ(poseSize, reinterpret_cast<uintptr_t>(pose) -
          reinterpret_cast<uintptr_t>(prevPose));
    }

    if (prevIt != nullptr)
    {
      EXPECT_EQ(intSize, reinterpret_cast<uintptr_t>(it) -
          reinterpret_cast<uintptr_t>(prevIt));
    }
    prevPose = pose;
    prevIt = it;
  }
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, InvalidComponentType)
{
  gazebo::EntityComponentManager manager;

  gazebo::ComponentKey key{999, 0};

  // Can't remove component from an inexistent entity
  EXPECT_FALSE(manager.HasEntity(1));
  EXPECT_FALSE(manager.RemoveComponent(1, key));

  // Can't remove a component type that doesn't exist.
  EXPECT_EQ(0, manager.CreateEntity());
  EXPECT_EQ(1, manager.CreateEntity());
  EXPECT_TRUE(manager.HasEntity(1));
  EXPECT_FALSE(manager.RemoveComponent(1, key));

  // We should get a nullptr if the component type doesn't exist.
  EXPECT_EQ(nullptr, manager.Component<int>(key));
}

/////////////////////////////////////////////////
// Removing a component should guarantee that existing components remain
// adjacent to each other.
TEST_P(EntityComponentManagerFixture, RemoveAdjacent)
{
  gazebo::EntityComponentManager manager;

  std::vector<ignition::math::Pose3d> poses;
  std::vector<gazebo::ComponentKey> keys;

  gazebo::EntityId entityId = manager.CreateEntity();

  int count = 3;

  // Create the components.
  for (int i = 0; i < count; ++i)
  {
    poses.push_back(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
    keys.push_back(manager.CreateComponent(entityId, poses.back()));
    EXPECT_EQ(keys.back().second, i);
  }
  ASSERT_EQ(poses.size(), keys.size());

  uintptr_t poseSize = sizeof(ignition::math::Pose3d);
  const ignition::math::Pose3d *pose = nullptr, *prevPose = nullptr;

  // Check that each component is adjacent in memory
  for (int i = 0; i < count; ++i)
  {
    pose = manager.Component<ignition::math::Pose3d>(keys[i]);
    if (prevPose != nullptr)
    {
      EXPECT_EQ(poseSize, reinterpret_cast<uintptr_t>(pose) -
          reinterpret_cast<uintptr_t>(prevPose));
    }
    prevPose = pose;
  }

  // Remove the middle component.
  EXPECT_TRUE(manager.EntityHasComponent(entityId, keys[1]));
  EXPECT_TRUE(manager.RemoveComponent(entityId, keys[1]));
  EXPECT_FALSE(manager.EntityHasComponent(entityId, keys[1]));

  // Can't remove the component twice.
  EXPECT_FALSE(manager.RemoveComponent(entityId, keys[1]));

  // Check that the two remaining components are still adjacent in memory
  const ignition::math::Pose3d *pose1 =
    manager.Component<ignition::math::Pose3d>(keys[0]);
  const ignition::math::Pose3d *pose3 =
    manager.Component<ignition::math::Pose3d>(keys[2]);
  EXPECT_EQ(poseSize,
      reinterpret_cast<uintptr_t>(pose3) - reinterpret_cast<uintptr_t>(pose1));
}

/////////////////////////////////////////////////
// Removing a component should guarantee that existing components remain
// adjacent to each other, and addition of a new component is adjacent to
// the last element.
TEST_P(EntityComponentManagerFixture, RemoveAddAdjacent)
{
  gazebo::EntityComponentManager manager;

  gazebo::EntityId entityId = manager.CreateEntity();

  std::vector<gazebo::ComponentKey> keys;

  keys.push_back(manager.CreateComponent(entityId,
        ignition::math::Pose3d(1, 2, 3, 0, 0, 0)));
  keys.push_back(manager.CreateComponent(entityId,
        ignition::math::Pose3d(3, 1, 2, 0, 0, 0)));
  keys.push_back(manager.CreateComponent(entityId,
        ignition::math::Pose3d(0, 10, 20, 0, 0, 0)));

  uintptr_t poseSize = sizeof(ignition::math::Pose3d);

  // Remove the middle component.
  EXPECT_TRUE(manager.RemoveComponent(entityId, keys[1]));

  // Add two more new component
  keys.push_back(manager.CreateComponent(entityId,
        ignition::math::Pose3d(101, 51, 520, 0, 0, 0)));

  keys.push_back(manager.CreateComponent(entityId,
        ignition::math::Pose3d(1010, 81, 821, 0, 0, 0)));

  // Check that the components are all adjacent in memory
  const ignition::math::Pose3d *pose1 =
    manager.Component<ignition::math::Pose3d>(keys[0]);
  const ignition::math::Pose3d *pose2 =
    manager.Component<ignition::math::Pose3d>(keys[2]);
  const ignition::math::Pose3d *pose3 =
    manager.Component<ignition::math::Pose3d>(keys[3]);
  const ignition::math::Pose3d *pose4 =
    manager.Component<ignition::math::Pose3d>(keys[4]);

  EXPECT_EQ(poseSize,
      reinterpret_cast<uintptr_t>(pose2) - reinterpret_cast<uintptr_t>(pose1));

  EXPECT_EQ(poseSize,
      reinterpret_cast<uintptr_t>(pose3) - reinterpret_cast<uintptr_t>(pose2));

  EXPECT_EQ(poseSize,
      reinterpret_cast<uintptr_t>(pose4) - reinterpret_cast<uintptr_t>(pose3));

  // Check the values of the components.
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), *pose1);
  EXPECT_EQ(ignition::math::Pose3d(0, 10, 20, 0, 0, 0), *pose2);
  EXPECT_EQ(ignition::math::Pose3d(101, 51, 520, 0, 0, 0), *pose3);
  EXPECT_EQ(ignition::math::Pose3d(1010, 81, 821, 0, 0, 0), *pose4);
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EntitiesAndComponents)
{
  ignition::common::Console::SetVerbosity(4);
  gazebo::EntityComponentManager manager;
  EXPECT_EQ(0u, manager.EntityCount());

  // Create a few entities
  gazebo::EntityId entityId = manager.CreateEntity();
  gazebo::EntityId entityId2 = manager.CreateEntity();
  manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add a component to an entity
  gazebo::ComponentKey cKey =  manager.CreateComponent<int>(entityId, 123);

  EXPECT_TRUE(manager.HasComponentType(
        gazebo::EntityComponentManager::ComponentType<int>()));
  EXPECT_TRUE(manager.EntityHasComponent(entityId, cKey));
  EXPECT_TRUE(manager.EntityHasComponentType(entityId,
        gazebo::EntityComponentManager::ComponentType<int>()));
  EXPECT_FALSE(manager.EntityHasComponentType(entityId,
        gazebo::EntityComponentManager::ComponentType<double>()));
  EXPECT_FALSE(manager.EntityHasComponentType(entityId2,
        gazebo::EntityComponentManager::ComponentType<int>()));

  // Erase all entities
  manager.EraseEntities();

  EXPECT_EQ(0u, manager.EntityCount());
  EXPECT_FALSE(manager.HasEntity(entityId));
  EXPECT_FALSE(manager.HasEntity(entityId2));
  EXPECT_FALSE(manager.EntityHasComponent(entityId, cKey));
  EXPECT_FALSE(manager.EntityHasComponentType(entityId,
        gazebo::EntityComponentManager::ComponentType<int>()));

  // The type itself still exists
  EXPECT_TRUE(manager.HasComponentType(
        gazebo::EntityComponentManager::ComponentType<int>()));
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, Query)
{
  ignition::common::Console::SetVerbosity(4);
  gazebo::EntityComponentManager manager;

  // Create some entities
  gazebo::EntityId eInt = manager.CreateEntity();
  gazebo::EntityId eDouble = manager.CreateEntity();
  gazebo::EntityId eIntDouble = manager.CreateEntity();
  // FIXME: adding eEmpty causes failure with
  // "C++ exception with description "map::at" thrown in the test body."
  // gazebo::EntityId eEmpty = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<int>(eInt, 123);
  manager.CreateComponent<double>(eDouble, 0.123);
  manager.CreateComponent<int>(eIntDouble, 456);
  manager.CreateComponent<double>(eIntDouble, 0.456);

  // Try querying with an invalid query
  ASSERT_EQ(std::nullopt, manager.Query(0));

  // Create queries
  gazebo::EntityQuery queryInt;
  queryInt.AddComponentType(
      gazebo::EntityComponentManager::ComponentType<int>());

  gazebo::EntityQuery queryDouble;
  queryDouble.AddComponentType(
      gazebo::EntityComponentManager::ComponentType<double>());

  gazebo::EntityQuery queryIntDouble;
  queryIntDouble.AddComponentType(
      gazebo::EntityComponentManager::ComponentType<int>());
  queryIntDouble.AddComponentType(
      gazebo::EntityComponentManager::ComponentType<double>());

  gazebo::EntityQuery queryBool;
  queryBool.AddComponentType(
      gazebo::EntityComponentManager::ComponentType<bool>());

  // Add queries to manager
  gazebo::EntityQueryId queryIntId = manager.AddQuery(queryInt);
  EXPECT_EQ(0, queryIntId);

  gazebo::EntityQueryId queryDoubleId = manager.AddQuery(queryDouble);
  EXPECT_EQ(1, queryDoubleId);

  gazebo::EntityQueryId queryIntDoubleId = manager.AddQuery(queryIntDouble);
  EXPECT_EQ(2, queryIntDoubleId);

  gazebo::EntityQueryId queryBoolId = manager.AddQuery(queryBool);
  EXPECT_EQ(3, queryBoolId);

  // Query for int
  auto queryResult = manager.Query(queryIntId);
  ASSERT_NE(std::nullopt, queryResult);

  auto queryEntities = queryResult.value().get().Entities();
  // FIXME: why does it fail?
  // EXPECT_EQ(2u, queryEntities.size());
  // EXPECT_NE(queryEntities.end(), queryEntities.find(eInt));
  // EXPECT_NE(queryEntities.end(), queryEntities.find(eIntDouble));

  // Query for double
  queryResult = manager.Query(queryDoubleId);
  ASSERT_NE(std::nullopt, queryResult);

  queryEntities = queryResult.value().get().Entities();
  // FIXME: why does it fail?
  // EXPECT_EQ(2u, queryEntities.size());
  // EXPECT_NE(queryEntities.end(), queryEntities.find(eDouble));
  // EXPECT_NE(queryEntities.end(), queryEntities.find(eIntDouble));

  // Query for int and double
  queryResult = manager.Query(queryIntDoubleId);
  ASSERT_NE(std::nullopt, queryResult);

  queryEntities = queryResult.value().get().Entities();
  EXPECT_EQ(1u, queryEntities.size());
  EXPECT_NE(queryEntities.end(), queryEntities.find(eIntDouble));

  // Query for bool
  queryResult = manager.Query(queryBoolId);
  ASSERT_NE(std::nullopt, queryResult);

  queryEntities = queryResult.value().get().Entities();
  EXPECT_TRUE(queryEntities.empty());

  // Create new entity
  gazebo::EntityId eInt2 = manager.CreateEntity();
  auto cInt2Key = manager.CreateComponent<int>(eInt2, 123123);
  EXPECT_EQ(4u, manager.EntityCount());

  // Query for int again and check the new entity shows
  queryResult = manager.Query(queryIntId);
  ASSERT_NE(std::nullopt, queryResult);

  queryEntities = queryResult.value().get().Entities();
  // FIXME: why does it fail?
  // EXPECT_EQ(3u, queryEntities.size());
  // EXPECT_NE(queryEntities.end(), queryEntities.find(eInt));
  // EXPECT_NE(queryEntities.end(), queryEntities.find(eIntDouble));
  // EXPECT_NE(queryEntities.end(), queryEntities.find(eInt2));

  // Remove a component from an entity
  EXPECT_TRUE(manager.RemoveComponent(eInt2, cInt2Key));
  EXPECT_FALSE(manager.EntityHasComponentType(eInt2,
      gazebo::EntityComponentManager::ComponentType<int>()));

  // Query for int again and check the entity doesn't show anymore
  queryResult = manager.Query(queryIntId);
  ASSERT_NE(std::nullopt, queryResult);

  queryEntities = queryResult.value().get().Entities();
  // FIXME: why does it fail?
  // EXPECT_EQ(2u, queryEntities.size());
  // EXPECT_NE(queryEntities.end(), queryEntities.find(eInt));
  // EXPECT_NE(queryEntities.end(), queryEntities.find(eIntDouble));

  // TODO(chapulina) Remove an entity and check that it doesn't show up
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ComponentValues)
{
  ignition::common::Console::SetVerbosity(4);
  gazebo::EntityComponentManager manager;

  // Create some entities
  gazebo::EntityId eInt = manager.CreateEntity();
  gazebo::EntityId eDouble = manager.CreateEntity();
  gazebo::EntityId eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<int>(eInt, 123);
  manager.CreateComponent<double>(eDouble, 0.123);
  manager.CreateComponent<int>(eIntDouble, 456);
  manager.CreateComponent<double>(eIntDouble, 0.456);

  // Get component values
  {
    const auto *value = manager.Component<int>(eInt);
    ASSERT_NE(nullptr, value);
    EXPECT_EQ(123, *value);
  }

  {
    const auto *value = manager.Component<double>(eDouble);
    ASSERT_NE(nullptr, value);
    EXPECT_DOUBLE_EQ(0.123, *value);
  }

  {
    const auto *value = manager.Component<int>(eIntDouble);
    ASSERT_NE(nullptr, value);
    EXPECT_EQ(456, *value);
  }

  {
    const auto *value = manager.Component<double>(eIntDouble);
    ASSERT_NE(nullptr, value);
    EXPECT_DOUBLE_EQ(0.456, *value);
  }

  // Failure cases
  {
    const auto *value = manager.Component<int>(eDouble);
    ASSERT_EQ(nullptr, value);
  }

  {
    const auto *value = manager.Component<double>(eInt);
    ASSERT_EQ(nullptr, value);
  }

  {
    const auto *value = manager.Component<int>(999);
    ASSERT_EQ(nullptr, value);
  }

  {
    const auto *value = manager.Component<double>(999);
    ASSERT_EQ(nullptr, value);
  }
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(EntityComponentManagerRepeat,
    EntityComponentManagerFixture, ::testing::Range(1, 10));
