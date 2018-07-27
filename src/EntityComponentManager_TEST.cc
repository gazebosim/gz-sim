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

#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/common/Console.hh>
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

  int count = 2;

  manager.RegisterComponentType<ignition::math::Pose3d>(
      "ignition::math::Pose3d");

  gazebo::EntityId entityId = manager.CreateEntity();

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

  manager.RegisterComponentType<ignition::math::Pose3d>(
      "ignition::math::Pose3d");
  manager.RegisterComponentType<int>("int");
  gazebo::EntityId entityId = manager.CreateEntity();

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

  // Can't remove the component type that doesn't exist.
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
  manager.RegisterComponentType<ignition::math::Pose3d>(
      "ignition::math::Pose3d");

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
  EXPECT_TRUE(manager.RemoveComponent(entityId, keys[1]));

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
  manager.RegisterComponentType<ignition::math::Pose3d>(
      "ignition::math::Pose3d");

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

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(EntityComponentManagerRepeat,
    EntityComponentManagerFixture, ::testing::Range(1, 10));
