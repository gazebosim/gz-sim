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

class EntityCompMgrTest : public gazebo::EntityComponentManager
{
  public: void RunClearNewlyCreatedEntities()
  {
    this->ClearNewlyCreatedEntities();
  }
  public: void ProcessEntityRemovals()
  {
    this->ProcessRemoveEntityRequests();
  }
};

class EntityComponentManagerFixture : public ::testing::TestWithParam<int>
{
  public: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }
  public: EntityCompMgrTest manager;
};

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, AdjacentMemorySingleComponentType)
{
  std::vector<ignition::math::Pose3d> poses;
  std::vector<gazebo::ComponentKey> keys;

  int count = 10;

  gazebo::Entity entity = manager.CreateEntity();
  EXPECT_EQ(1u, entity);

  // Create the components.
  for (int i = 0; i < count; ++i)
  {
    poses.push_back(ignition::math::Pose3d(
          math::Rand::IntNormal(10, 5),
          math::Rand::IntNormal(100, 50),
          math::Rand::IntNormal(-100, 30), 0, 0, 0));
    keys.push_back(manager.CreateComponent(entity, poses.back()));

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
  std::vector<ignition::math::Pose3d> poses;
  std::vector<int> ints;
  std::vector<gazebo::ComponentKey> poseKeys;
  std::vector<gazebo::ComponentKey> intKeys;

  int count = 100000;

  gazebo::Entity entity = manager.CreateEntity();
  EXPECT_EQ(1u, entity);

  // Create the components.
  for (int i = 0; i < count; ++i)
  {
    poses.push_back(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
    ints.push_back(i);

    poseKeys.push_back(manager.CreateComponent(entity, poses.back()));
    intKeys.push_back(manager.CreateComponent(entity, ints.back()));

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
  gazebo::ComponentKey key{999, 0};

  // Can't remove component from an nonexistent entity
  EXPECT_FALSE(manager.HasEntity(2));
  EXPECT_FALSE(manager.RemoveComponent(2, key));

  // Can't remove a component that doesn't exist.
  EXPECT_EQ(1u, manager.CreateEntity());
  EXPECT_EQ(2u, manager.CreateEntity());
  EXPECT_TRUE(manager.HasEntity(2));
  EXPECT_FALSE(manager.RemoveComponent(2, key));

  // We should get a nullptr if the component type doesn't exist.
  EXPECT_EQ(nullptr, manager.Component<int>(key));
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, RemoveComponent)
{
  // Create some entities
  auto eInt = manager.CreateEntity();
  auto eDouble = manager.CreateEntity();
  auto eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components and keep their unique ComponentKeys
  auto cIntEInt = manager.CreateComponent<int>(eInt, 123);
  auto cDoubleEDouble = manager.CreateComponent<double>(eDouble, 0.123);
  auto cIntEIntDouble = manager.CreateComponent<int>(eIntDouble, 456);
  auto cDoubleEIntDouble = manager.CreateComponent<double>(eIntDouble, 0.456);

  // Check entities have the components
  EXPECT_TRUE(manager.EntityHasComponent(eInt, cIntEInt));
  EXPECT_TRUE(manager.EntityHasComponent(eDouble, cDoubleEDouble));
  EXPECT_TRUE(manager.EntityHasComponent(eIntDouble, cIntEIntDouble));
  EXPECT_TRUE(manager.EntityHasComponent(eIntDouble, cDoubleEIntDouble));

  // Remove component by key
  EXPECT_TRUE(manager.RemoveComponent(eInt, cIntEInt));
  EXPECT_FALSE(manager.EntityHasComponent(eInt, cIntEInt));

  // Remove component by type id
  auto typeDouble = manager.ComponentType<double>();

  EXPECT_TRUE(manager.RemoveComponent(eDouble, typeDouble));
  EXPECT_FALSE(manager.EntityHasComponent(eDouble, cDoubleEDouble));

  // Remove component by type
  EXPECT_TRUE(manager.RemoveComponent<int>(eIntDouble));
  EXPECT_FALSE(manager.EntityHasComponent(eIntDouble, cIntEIntDouble));
  EXPECT_TRUE(manager.EntityHasComponent(eIntDouble, cDoubleEIntDouble));

  EXPECT_TRUE(manager.RemoveComponent<double>(eIntDouble));
  EXPECT_FALSE(manager.EntityHasComponent(eIntDouble, cIntEIntDouble));
  EXPECT_FALSE(manager.EntityHasComponent(eIntDouble, cDoubleEIntDouble));
}

/////////////////////////////////////////////////
// Removing a component should guarantee that existing components remain
// adjacent to each other.
TEST_P(EntityComponentManagerFixture, RemoveAdjacent)
{
  std::vector<ignition::math::Pose3d> poses;
  std::vector<gazebo::ComponentKey> keys;

  gazebo::Entity entity = manager.CreateEntity();

  int count = 3;

  // Create the components.
  for (int i = 0; i < count; ++i)
  {
    poses.push_back(ignition::math::Pose3d(1, 2, 3, 0, 0, 0));
    keys.push_back(manager.CreateComponent(entity, poses.back()));
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
  EXPECT_TRUE(manager.EntityHasComponent(entity, keys[1]));
  EXPECT_TRUE(manager.RemoveComponent(entity, keys[1]));
  EXPECT_FALSE(manager.EntityHasComponent(entity, keys[1]));

  // Can't remove the component twice.
  EXPECT_FALSE(manager.RemoveComponent(entity, keys[1]));

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
  gazebo::Entity entity = manager.CreateEntity();

  std::vector<gazebo::ComponentKey> keys;

  keys.push_back(manager.CreateComponent(entity,
        ignition::math::Pose3d(1, 2, 3, 0, 0, 0)));
  keys.push_back(manager.CreateComponent(entity,
        ignition::math::Pose3d(3, 1, 2, 0, 0, 0)));
  keys.push_back(manager.CreateComponent(entity,
        ignition::math::Pose3d(0, 10, 20, 0, 0, 0)));

  uintptr_t poseSize = sizeof(ignition::math::Pose3d);

  // Remove the middle component.
  EXPECT_TRUE(manager.RemoveComponent(entity, keys[1]));

  // Add two more new component
  keys.push_back(manager.CreateComponent(entity,
        ignition::math::Pose3d(101, 51, 520, 0, 0, 0)));

  keys.push_back(manager.CreateComponent(entity,
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
  EXPECT_EQ(0u, manager.EntityCount());

  // Create a few entities
  gazebo::Entity entity = manager.CreateEntity();
  gazebo::Entity entity2 = manager.CreateEntity();
  manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add a component to an entity
  gazebo::ComponentKey cKey =  manager.CreateComponent<int>(entity, 123);

  EXPECT_TRUE(manager.HasComponentType(
        gazebo::EntityComponentManager::ComponentType<int>()));
  EXPECT_TRUE(manager.EntityHasComponent(entity, cKey));
  EXPECT_TRUE(manager.EntityHasComponentType(entity,
        gazebo::EntityComponentManager::ComponentType<int>()));
  EXPECT_FALSE(manager.EntityHasComponentType(entity,
        gazebo::EntityComponentManager::ComponentType<double>()));
  EXPECT_FALSE(manager.EntityHasComponentType(entity2,
        gazebo::EntityComponentManager::ComponentType<int>()));

  // Remove all entities
  manager.RequestRemoveEntities();
  EXPECT_EQ(3u, manager.EntityCount());
  manager.ProcessEntityRemovals();

  EXPECT_EQ(0u, manager.EntityCount());
  EXPECT_FALSE(manager.HasEntity(entity));
  EXPECT_FALSE(manager.HasEntity(entity2));
  EXPECT_FALSE(manager.EntityHasComponent(entity, cKey));
  EXPECT_FALSE(manager.EntityHasComponentType(entity,
        gazebo::EntityComponentManager::ComponentType<int>()));

  // The type itself still exists
  EXPECT_TRUE(manager.HasComponentType(
        gazebo::EntityComponentManager::ComponentType<int>()));
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ComponentValues)
{
  // Create some entities
  gazebo::Entity eInt = manager.CreateEntity();
  gazebo::Entity eDouble = manager.CreateEntity();
  gazebo::Entity eIntDouble = manager.CreateEntity();
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

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, RebuildViews)
{
  // Create some entities
  gazebo::Entity eInt = manager.CreateEntity();
  gazebo::Entity eDouble = manager.CreateEntity();
  gazebo::Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<int>(eInt, 123);
  manager.CreateComponent<double>(eDouble, 0.123);
  manager.CreateComponent<int>(eIntDouble, 456);
  manager.CreateComponent<double>(eIntDouble, 0.456);

  // The first iteration of this loop builds views. At the end, views are
  // rebuilt. The second iteration should return the same values as the
  // first iteration.
  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    // The first call to each will create a view.
    manager.Each<int> ([&](const ignition::gazebo::Entity &_entity,
          const int *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, *_value);
          }
          if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, *_value);
          }
          ++count;
          return true;
        });
    EXPECT_EQ(2, count);

    count = 0;
    manager.Each<double> ([&](const ignition::gazebo::Entity &_entity,
          const double *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, *_value);
          }
          if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, *_value);
          }
          ++count;
          return true;
        });
    EXPECT_EQ(2, count);

    // Rebuild the view.
    manager.RebuildViews();
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ViewsAddComponents)
{
  // Create some entities
  gazebo::Entity eInt = manager.CreateEntity();
  gazebo::Entity eDouble = manager.CreateEntity();
  gazebo::Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<int>(eInt, 123);
  manager.CreateComponent<double>(eDouble, 0.123);
  manager.CreateComponent<int>(eIntDouble, 456);
  manager.CreateComponent<double>(eIntDouble, 0.456);

  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    manager.Each<int> ([&](const ignition::gazebo::Entity &_entity,
          const int *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, *_value);
          }
          if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, *_value);
          }
          ++count;
          return true;
        });
    EXPECT_EQ(2, count);

    count = 0;
    manager.Each<double> ([&](const ignition::gazebo::Entity &_entity,
          const double *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_DOUBLE_EQ(12.123, *_value);
          }
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, *_value);
          }
          if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, *_value);
          }
          ++count;
          return true;
        });
    if (i == 0)
      EXPECT_EQ(2, count);
    else
      EXPECT_EQ(3, count);

    manager.CreateComponent<double>(eInt, 12.123);
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ViewsRemoveComponents)
{
  // Create some entities
  gazebo::Entity eInt = manager.CreateEntity();
  gazebo::Entity eDouble = manager.CreateEntity();
  gazebo::Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<int>(eInt, 123);
  manager.CreateComponent<double>(eDouble, 0.123);
  manager.CreateComponent<int>(eIntDouble, 456);
  auto compToRemove = manager.CreateComponent<double>(eIntDouble, 0.456);

  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    manager.Each<int> ([&](const ignition::gazebo::Entity &_entity,
          const int *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, *_value);
          }
          if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, *_value);
          }
          ++count;
          return true;
        });
    EXPECT_EQ(2, count);

    count = 0;
    manager.Each<double> ([&](const ignition::gazebo::Entity &_entity,
          const double *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_DOUBLE_EQ(12.123, *_value);
          }
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, *_value);
          }
          if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, *_value);
          }
          ++count;
          return true;
        });
    if (i == 0)
      EXPECT_EQ(2, count);
    else
      EXPECT_EQ(1, count);

    if (i == 0)
    {
      EXPECT_TRUE(manager.RemoveComponent(eIntDouble, compToRemove));
    }
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ViewsAddEntity)
{
  // Create some entities
  gazebo::Entity eInt = manager.CreateEntity();
  gazebo::Entity eDouble = manager.CreateEntity();
  gazebo::Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<int>(eInt, 123);
  manager.CreateComponent<double>(eDouble, 0.123);
  manager.CreateComponent<int>(eIntDouble, 456);
  manager.CreateComponent<double>(eIntDouble, 0.456);

  gazebo::Entity newEntity;

  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    manager.Each<int> ([&](const ignition::gazebo::Entity &_entity,
          const int *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, *_value);
          }
          else if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, *_value);
          }
          else if (_entity == newEntity)
          {
            EXPECT_EQ(789, *_value);
          }
          else
          {
            // This used to be a FAIL() call, however we can't use FAIL
            // inside a function that has a return value.
            EXPECT_TRUE(false);
          }
          ++count;
          return true;
        });
    if (i == 0)
      EXPECT_EQ(2, count);
    else
      EXPECT_EQ(3, count);

    count = 0;
    manager.Each<double> ([&](const ignition::gazebo::Entity &_entity,
          const double *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, *_value);
          }
          else if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, *_value);
          }
          else
          {
            // This used to be a FAIL() call, however we can't use FAIL
            // inside a function that has a return value.
            EXPECT_TRUE(false);
          }
          ++count;
          return true;
        });
    EXPECT_EQ(2, count);

    newEntity = manager.CreateEntity();
    manager.CreateComponent<int>(newEntity, 789);
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ViewsRemoveEntities)
{
  // Create some entities
  gazebo::Entity eInt = manager.CreateEntity();
  gazebo::Entity eDouble = manager.CreateEntity();
  gazebo::Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<int>(eInt, 123);
  manager.CreateComponent<double>(eDouble, 0.123);
  manager.CreateComponent<int>(eIntDouble, 456);
  manager.CreateComponent<double>(eIntDouble, 0.456);

  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    manager.Each<int> ([&](const ignition::gazebo::Entity &_entity,
          const int *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, *_value);
          }
          if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, *_value);
          }
          ++count;
          return true;
        });
    if (i == 0)
      EXPECT_EQ(2, count);
    else
      EXPECT_EQ(0, count);

    count = 0;
    manager.Each<double> ([&](const ignition::gazebo::Entity &_entity,
          const double *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_DOUBLE_EQ(12.123, *_value);
          }
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, *_value);
          }
          if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, *_value);
          }
          ++count;
          return true;
        });
    if (i == 0)
      EXPECT_EQ(2, count);
    else
      EXPECT_EQ(0, count);

    manager.RequestRemoveEntities();
    manager.ProcessEntityRemovals();
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, RemoveEntity)
{
  // Create some entities
  auto e1 = manager.CreateEntity();
  EXPECT_EQ(1u, e1);
  EXPECT_TRUE(manager.HasEntity(e1));

  auto e2 = manager.CreateEntity();
  EXPECT_EQ(2u, e2);
  EXPECT_TRUE(manager.HasEntity(e2));

  auto e3 = manager.CreateEntity();
  EXPECT_EQ(3u, e3);
  EXPECT_TRUE(manager.HasEntity(e3));

  EXPECT_EQ(3u, manager.EntityCount());

  // Delete an Entity
  manager.RequestRemoveEntity(e2);
  EXPECT_EQ(3u, manager.EntityCount());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(2u, manager.EntityCount());
  EXPECT_FALSE(manager.HasEntity(e2));

  // Creating an new entity shouldn't reuse the previously deleted entity.
  auto e4 = manager.CreateEntity();
  EXPECT_EQ(4u, e4);
  EXPECT_EQ(3u, manager.EntityCount());

  // Can not delete an invalid entity.
  manager.RequestRemoveEntity(6);
  EXPECT_EQ(3u, manager.EntityCount());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(3u, manager.EntityCount());

  // Delete another
  manager.RequestRemoveEntity(1);
  EXPECT_EQ(3u, manager.EntityCount());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(2u, manager.EntityCount());

  // Delete another
  manager.RequestRemoveEntity(3);
  EXPECT_EQ(2u, manager.EntityCount());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(1u, manager.EntityCount());

  // Delete last
  manager.RequestRemoveEntity(4);
  EXPECT_EQ(1u, manager.EntityCount());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(0u, manager.EntityCount());
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ViewsRemoveEntity)
{
  // Create some entities
  gazebo::Entity eInt = manager.CreateEntity();
  gazebo::Entity eDouble = manager.CreateEntity();
  gazebo::Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<int>(eInt, 123);
  manager.CreateComponent<double>(eDouble, 0.123);
  manager.CreateComponent<int>(eIntDouble, 456);
  manager.CreateComponent<double>(eIntDouble, 0.456);

  int count = 0;
  manager.Each<int> ([&](const ignition::gazebo::Entity &_entity,
        const int *_value)->bool
      {
        EXPECT_NE(nullptr, _value);
        if (_entity == eInt)
        {
          EXPECT_EQ(123, *_value);
        }
        if (_entity == eIntDouble)
        {
          EXPECT_EQ(456, *_value);
        }
        ++count;
        return true;
      });
  EXPECT_EQ(2, count);

  // Remove an entity.
  manager.RequestRemoveEntity(eIntDouble);
  manager.ProcessEntityRemovals();

  count = 0;
  manager.Each<int> ([&](const ignition::gazebo::Entity &_entity,
        const int *_value)->bool
      {
        EXPECT_NE(nullptr, _value);
        EXPECT_NE(eIntDouble, _entity);
        if (_entity == eInt)
        {
          EXPECT_EQ(123, *_value);
        }
        ++count;
        return true;
      });
  EXPECT_EQ(1, count);
}

//////////////////////////////////////////////////
/// \brief Helper function to count the number of "new" entities
template<typename ...Ts>
int newCount(EntityCompMgrTest &_manager)
{
  int count = 0;
  _manager.EachNew<Ts...>(
      [&](const ignition::gazebo::Entity &, Ts *... _values) -> bool
      {
        ++count;
        // can always cast to const void *
        auto valSet = std::set<const void *>{_values...};
        for (auto value : valSet )
          EXPECT_NE(nullptr, value);

        return true;
      });

  // get a const ref to test the const version of EachNew
  const EntityCompMgrTest &managerConst = _manager;

  count = 0;
  managerConst.EachNew<Ts ...>(
      [&](const ignition::gazebo::Entity &, const Ts *... _values) -> bool
      {
        ++count;
        // can always cast to const void *
        auto valSet = std::set<const void *>{_values...};
        for (auto value : valSet )
          EXPECT_NE(nullptr, value);
        return true;
      });
  return count;
}

//////////////////////////////////////////////////
/// \brief Helper function to count the number of "removed" entities
template<typename ...Ts>
int removedCount(EntityCompMgrTest &_manager)
{
  int count = 0;
  _manager.EachRemoved<Ts ...>(
      [&](const ignition::gazebo::Entity &, const Ts *... _values) -> bool
      {
        ++count;
        auto valSet = std::set<const void *>{_values...};
        for (auto value : valSet )
          EXPECT_NE(nullptr, value);
        return true;
      });
  return count;
}

//////////////////////////////////////////////////
/// \brief Helper function to count the number of entities returned by an Each
/// call
template<typename ...Ts>
int eachCount(EntityCompMgrTest &_manager)
{
  int count = 0;
  _manager.Each<Ts ...>(
      [&](const ignition::gazebo::Entity &, const Ts *... _values) -> bool
      {
        ++count;
        auto valSet = std::set<const void *>{_values...};
        for (auto value : valSet )
          EXPECT_NE(nullptr, value);
        return true;
      });
  return count;
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewBasic)
{
  // Create entities
  gazebo::Entity e1 = manager.CreateEntity();
  gazebo::Entity e2 = manager.CreateEntity();
  EXPECT_EQ(2u, manager.EntityCount());

  // Add components to each entity
  manager.CreateComponent<int>(e1, 123);
  manager.CreateComponent<int>(e2, 456);

  EXPECT_EQ(2, newCount<int>(manager));

  // This would normally be done after each simulation step after systems are
  // updated
  manager.RunClearNewlyCreatedEntities();
  EXPECT_EQ(0, newCount<int>(manager));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewAfterRemoveComponent)
{
  // Create entities
  gazebo::Entity e1 = manager.CreateEntity();
  auto comp1 = manager.CreateComponent<int>(e1, 123);
  manager.CreateComponent<double>(e1, 0.0);

  EXPECT_EQ(1, newCount<int>(manager));

  manager.RemoveComponent(e1, comp1);
  EXPECT_EQ(1, newCount<double>(manager));

  manager.RunClearNewlyCreatedEntities();
  EXPECT_EQ(0, newCount<double>(manager));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewRemoveComponentFromRemoveEntity)
{
  // Create entities
  gazebo::Entity e1 = manager.CreateEntity();
  manager.CreateComponent<int>(e1, 123);
  manager.RunClearNewlyCreatedEntities();
  // Nothing new after cleared
  EXPECT_EQ(0, newCount<int>(manager));

  gazebo::Entity e2 = manager.CreateEntity();
  manager.CreateComponent<int>(e2, 456);
  EXPECT_EQ(1, newCount<int>(manager));
  // Check if this true after RebuildViews
  manager.RebuildViews();
  EXPECT_EQ(1, newCount<int>(manager));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewAddComponentToExistingEntity)
{
  // Create entities
  gazebo::Entity e1 = manager.CreateEntity();
  gazebo::Entity e2 = manager.CreateEntity();
  manager.CreateComponent<int>(e1, 123);
  manager.CreateComponent<int>(e2, 456);
  manager.RunClearNewlyCreatedEntities();
  // Nothing new after cleared
  EXPECT_EQ(0, newCount<int>(manager));

  // Create a new entity
  gazebo::Entity e3 = manager.CreateEntity();
  manager.CreateComponent<int>(e3, 789);
  // Add a new component to existing entities
  manager.CreateComponent<double>(e1, 0.0);
  manager.CreateComponent<double>(e2, 2.0);

  // e1 and e2 have a new double component, but they are not considered new
  // entities
  EXPECT_EQ(0, (newCount<int, double>(manager)));
  // Only e3 is considered new
  EXPECT_EQ(1, newCount<int>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveBasic)
{
  // Create an entities
  gazebo::Entity e1 = manager.CreateEntity();
  gazebo::Entity e2 = manager.CreateEntity();
  EXPECT_EQ(2u, manager.EntityCount());

  // Add components to each entity
  manager.CreateComponent<int>(e1, 123);
  manager.CreateComponent<int>(e2, 456);

  // Remove an entity.
  manager.RequestRemoveEntity(e1);
  EXPECT_EQ(1, removedCount<int>(manager));
  manager.RequestRemoveEntity(e2);
  EXPECT_EQ(2, removedCount<int>(manager));

  // This would normally be done after each simulation step after systems are
  // updated
  manager.RunClearNewlyCreatedEntities();
  // But it shouldn't affect removed entities
  EXPECT_EQ(2, removedCount<int>(manager));

  manager.ProcessEntityRemovals();
  EXPECT_EQ(0, removedCount<int>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveAlreadyRemove)
{
  // Create an entities
  gazebo::Entity e1 = manager.CreateEntity();
  gazebo::Entity e2 = manager.CreateEntity();
  EXPECT_EQ(2u, manager.EntityCount());

  // Add components to each entity
  manager.CreateComponent<int>(e1, 123);
  manager.CreateComponent<int>(e2, 456);
  manager.RequestRemoveEntity(e2);

  manager.ProcessEntityRemovals();

  // try erasing an already removed entity
  manager.RequestRemoveEntity(e2);
  EXPECT_EQ(0, removedCount<int>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveAfterRebuild)
{
  // Test after rebuild
  gazebo::Entity e1 = manager.CreateEntity();
  EXPECT_EQ(1u, manager.EntityCount());

  manager.CreateComponent<int>(e1, 123);
  EXPECT_EQ(1, newCount<int>(manager));
  manager.RunClearNewlyCreatedEntities();

  manager.RequestRemoveEntity(e1);
  EXPECT_EQ(1, removedCount<int>(manager));

  manager.RebuildViews();
  EXPECT_EQ(1, removedCount<int>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveAddComponentToRemoveEntity)
{
  gazebo::Entity e1 = manager.CreateEntity();
  manager.CreateComponent<int>(e1, 123);
  manager.RunClearNewlyCreatedEntities();
  manager.RequestRemoveEntity(e1);

  // Add a new component to an removed entity. This should be possible since the
  // entity is only scheduled to be removed.
  manager.CreateComponent<double>(e1, 0.0);
  EXPECT_EQ(1, removedCount<int>(manager));
  EXPECT_EQ(1, (removedCount<int, double>(manager)));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveAllRemove)
{
  // Test when all entities are removed
  gazebo::Entity e1 = manager.CreateEntity();
  gazebo::Entity e2 = manager.CreateEntity();
  manager.CreateComponent<int>(e1, 123);
  manager.CreateComponent<int>(e2, 456);
  EXPECT_EQ(2u, manager.EntityCount());

  manager.RequestRemoveEntities();
  EXPECT_EQ(2, removedCount<int>(manager));

  manager.ProcessEntityRemovals();
  EXPECT_EQ(0, removedCount<int>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewEachRemove)
{
  // Test EachNew and EachRemove together
  gazebo::Entity e1 = manager.CreateEntity();
  gazebo::Entity e2 = manager.CreateEntity();
  manager.CreateComponent<int>(e1, 123);
  manager.CreateComponent<int>(e2, 456);
  EXPECT_EQ(2u, manager.EntityCount());

  EXPECT_EQ(2, newCount<int>(manager));
  EXPECT_EQ(0, removedCount<int>(manager));

  // Remove an entity.
  manager.RequestRemoveEntity(e1);
  // An entity can be considered new even if there is a request to remove it.
  EXPECT_EQ(2, newCount<int>(manager));
  EXPECT_EQ(1, removedCount<int>(manager));

  // ProcessEntityRemovals and ClearNewlyCreatedEntities would be called
  // together after a simulation step
  manager.RunClearNewlyCreatedEntities();
  manager.ProcessEntityRemovals();

  EXPECT_EQ(0, newCount<int>(manager));
  EXPECT_EQ(0, removedCount<int>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachGetsNewOldRemove)
{
  // Test that an Each call gets new, old, and removed entities
  gazebo::Entity e1 = manager.CreateEntity();
  gazebo::Entity e2 = manager.CreateEntity();
  manager.CreateComponent<int>(e1, 123);
  manager.CreateComponent<int>(e2, 456);
  EXPECT_EQ(2u, manager.EntityCount());

  EXPECT_EQ(2, eachCount<int>(manager));
  EXPECT_EQ(2, newCount<int>(manager));
  EXPECT_EQ(0, removedCount<int>(manager));

  // Remove an entity.
  manager.RequestRemoveEntity(e1);
  // Each gets entities that removed
  EXPECT_EQ(2, eachCount<int>(manager));
  // An entity can be considered new even if there is a request to remove it.
  EXPECT_EQ(2, newCount<int>(manager));
  EXPECT_EQ(1, removedCount<int>(manager));

  // ProcessEntityRemovals and ClearNewlyCreatedEntities would be called
  // together after a simulation step
  manager.RunClearNewlyCreatedEntities();
  manager.ProcessEntityRemovals();

  // One entity is removed, one left
  EXPECT_EQ(1, eachCount<int>(manager));
  EXPECT_EQ(0, newCount<int>(manager));
  EXPECT_EQ(0, removedCount<int>(manager));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EntityByComponents)
{
  // Create some entities
  gazebo::Entity eInt = manager.CreateEntity();
  gazebo::Entity eUint = manager.CreateEntity();
  gazebo::Entity eIntUint = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<int>(eInt, -123);
  manager.CreateComponent<std::string>(eInt, "int");

  manager.CreateComponent<unsigned int>(eUint, 456u);
  manager.CreateComponent<std::string>(eUint, "uint");

  manager.CreateComponent<int>(eIntUint, 789);
  manager.CreateComponent<unsigned int>(eIntUint, 789u);
  manager.CreateComponent<std::string>(eIntUint, "int-uint");

  // Get entities by the value of their components
  EXPECT_EQ(eInt, manager.EntityByComponents(-123));
  EXPECT_EQ(eInt, manager.EntityByComponents(std::string("int")));
  EXPECT_EQ(eInt, manager.EntityByComponents(std::string("int"), -123));

  EXPECT_EQ(eUint, manager.EntityByComponents(456u));
  EXPECT_EQ(eUint, manager.EntityByComponents(std::string("uint")));
  EXPECT_EQ(eUint, manager.EntityByComponents(std::string("uint"), 456u));

  EXPECT_EQ(eIntUint, manager.EntityByComponents(789));
  EXPECT_EQ(eIntUint, manager.EntityByComponents(789u));
  EXPECT_EQ(eIntUint, manager.EntityByComponents(std::string("int-uint")));
  EXPECT_EQ(eIntUint, manager.EntityByComponents(789, 789u));
  EXPECT_EQ(eIntUint, manager.EntityByComponents(std::string("int-uint"),
      789, 789u));

  EXPECT_EQ(gazebo::kNullEntity, manager.EntityByComponents(123456));
  EXPECT_EQ(gazebo::kNullEntity, manager.EntityByComponents(123u));
  EXPECT_EQ(gazebo::kNullEntity, manager.EntityByComponents(true));
  EXPECT_EQ(gazebo::kNullEntity, manager.EntityByComponents("123456"));
  EXPECT_EQ(gazebo::kNullEntity, manager.EntityByComponents("int", 456u));
  EXPECT_EQ(gazebo::kNullEntity, manager.EntityByComponents(456u, 789u));
  EXPECT_EQ(gazebo::kNullEntity, manager.EntityByComponents(-123, 456u));
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EntityGraph)
{
  EXPECT_EQ(0u, manager.EntityCount());

  /*
   *        1
   *      /   \
   *     2     3
   *  / / \ \
   * 4 5   6 7
   */

  // Create a few entities
  auto e1 = manager.CreateEntity();
  auto e2 = manager.CreateEntity();
  auto e3 = manager.CreateEntity();
  auto e4 = manager.CreateEntity();
  auto e5 = manager.CreateEntity();
  auto e6 = manager.CreateEntity();
  auto e7 = manager.CreateEntity();
  EXPECT_EQ(7u, manager.EntityCount());

  // Set parents
  EXPECT_TRUE(manager.SetParentEntity(e2, e1));
  EXPECT_TRUE(manager.SetParentEntity(e3, e1));
  EXPECT_TRUE(manager.SetParentEntity(e4, e2));
  EXPECT_TRUE(manager.SetParentEntity(e5, e2));
  EXPECT_TRUE(manager.SetParentEntity(e6, e2));
  EXPECT_TRUE(manager.SetParentEntity(e7, e2));

  EXPECT_FALSE(manager.SetParentEntity(e1, gazebo::Entity(1000)));
  EXPECT_FALSE(manager.SetParentEntity(gazebo::Entity(1000), e1));

  // Check their parents
  EXPECT_EQ(gazebo::kNullEntity, manager.ParentEntity(e1));
  EXPECT_EQ(e1, manager.ParentEntity(e2));
  EXPECT_EQ(e1, manager.ParentEntity(e3));
  EXPECT_EQ(e2, manager.ParentEntity(e4));
  EXPECT_EQ(e2, manager.ParentEntity(e5));
  EXPECT_EQ(e2, manager.ParentEntity(e6));
  EXPECT_EQ(e2, manager.ParentEntity(e7));

  // Detach from graph
  EXPECT_TRUE(manager.SetParentEntity(e7, gazebo::kNullEntity));
  EXPECT_EQ(gazebo::kNullEntity, manager.ParentEntity(e7));

  // Reparent
  EXPECT_TRUE(manager.SetParentEntity(e5, e3));
  EXPECT_EQ(e3, manager.ParentEntity(e5));

  /*        1       7
   *      /   \
   *     2     3
   *    / \     \
   *   4   6     5
   */

  // Add components
  struct Even
  {
    bool operator!=(const Even &) const
    {
      return false;
    }
  };
  manager.CreateComponent<Even>(e2, {});
  manager.CreateComponent<Even>(e4, {});
  manager.CreateComponent<Even>(e6, {});

  struct Odd
  {
    bool operator!=(const Odd &) const
    {
      return false;
    }
  };
  manager.CreateComponent<Odd>(e1, {});
  manager.CreateComponent<Odd>(e3, {});
  manager.CreateComponent<Odd>(e5, {});
  manager.CreateComponent<Odd>(e7, {});

  // Get children by components
  {
    auto result = manager.ChildrenByComponents(e1, Even());
    ASSERT_EQ(1u, result.size());
    EXPECT_EQ(e2, result.front());
  }
  {
    auto result = manager.ChildrenByComponents(e1, Odd());
    ASSERT_EQ(1u, result.size());
    EXPECT_EQ(e3, result.front());
  }
  {
    auto result = manager.ChildrenByComponents(e2, Even());
    ASSERT_EQ(2u, result.size());
    EXPECT_TRUE(std::find(result.begin(), result.end(), e4) != result.end());
    EXPECT_TRUE(std::find(result.begin(), result.end(), e6) != result.end());
  }
  {
    auto result = manager.ChildrenByComponents(e2, Odd());
    ASSERT_TRUE(result.empty());
  }
  {
    auto result = manager.ChildrenByComponents(e3, Even());
    ASSERT_TRUE(result.empty());
  }
  {
    auto result = manager.ChildrenByComponents(e3, Odd());
    ASSERT_EQ(1u, result.size());
    EXPECT_EQ(e5, result.front());
  }
  {
    auto result = manager.ChildrenByComponents(e4, Even());
    ASSERT_TRUE(result.empty());
  }
  {
    auto result = manager.ChildrenByComponents(e4, Odd());
    ASSERT_TRUE(result.empty());
  }
  {
    auto result = manager.ChildrenByComponents(e5, Even());
    ASSERT_TRUE(result.empty());
  }
  {
    auto result = manager.ChildrenByComponents(e5, Odd());
    ASSERT_TRUE(result.empty());
  }
  {
    auto result = manager.ChildrenByComponents(e6, Even());
    ASSERT_TRUE(result.empty());
  }
  {
    auto result = manager.ChildrenByComponents(e6, Odd());
    ASSERT_TRUE(result.empty());
  }

  // Remove recursively (e2, e4, e6)
  manager.RequestRemoveEntity(e2);
  manager.ProcessEntityRemovals();
  EXPECT_EQ(4u, manager.EntityCount());
  EXPECT_FALSE(manager.HasEntity(e2));
  EXPECT_FALSE(manager.HasEntity(e4));
  EXPECT_FALSE(manager.HasEntity(e6));
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(EntityComponentManagerRepeat,
    EntityComponentManagerFixture, ::testing::Range(1, 10));
