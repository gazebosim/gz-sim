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

#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/config.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

namespace ignition
{
namespace gazebo
{
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace components
{
using IntComponent = components::Component<int, class IntComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.IntComponent",
    IntComponent)

using UIntComponent = components::Component<int, class IntComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.UIntComponent",
    UIntComponent)

using DoubleComponent = components::Component<double, class DoubleComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.DoubleComponent",
    DoubleComponent)

using StringComponent =
    components::Component<std::string, class StringComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.StringComponent",
    StringComponent)

using BoolComponent = components::Component<bool, class BoolComponentTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.BoolComponent",
    BoolComponent)

using Even = components::Component<components::NoData, class EvenTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.Even", Even)

using Odd = components::Component<components::NoData, class OddTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.Odd", Odd)
}
}
}
}

class EntityCompMgrTest : public EntityComponentManager
{
  public: void RunClearNewlyCreatedEntities()
  {
    this->ClearNewlyCreatedEntities();
  }
  public: void ProcessEntityRemovals()
  {
    this->ProcessRemoveEntityRequests();
  }
  public: void RunSetAllComponentsUnchanged()
  {
    this->SetAllComponentsUnchanged();
  }
};

class EntityComponentManagerFixture : public ::testing::TestWithParam<int>
{
  public: void SetUp() override
  {
    common::Console::SetVerbosity(4);
  }
  public: EntityCompMgrTest manager;
};

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, AdjacentMemorySingleComponentType)
{
  std::vector<components::Pose> poses;
  std::vector<ComponentKey> keys;

  int count = 10;

  Entity entity = manager.CreateEntity();
  EXPECT_EQ(1u, entity);

  // Create the components.
  for (int i = 0; i < count; ++i)
  {
    poses.push_back(components::Pose(math::Pose3d(
          math::Rand::IntNormal(10, 5),
          math::Rand::IntNormal(100, 50),
          math::Rand::IntNormal(-100, 30), 0, 0, 0)));
    keys.push_back(manager.CreateComponent(entity, poses.back()));

    // The component ids should increment by one for each component.
    EXPECT_EQ(keys.back().second, i);
  }

  ASSERT_EQ(count, static_cast<int>(poses.size()));
  ASSERT_EQ(count, static_cast<int>(keys.size()));

  // Check the component values.
  for (int i = 0; i < count; ++i)
  {
    EXPECT_EQ(poses[i], *(manager.Component<components::Pose>(keys[i])));
  }
  {
    uintptr_t poseSize = sizeof(components::Pose);
    const components::Pose *pose = nullptr, *prevPose = nullptr;

    // Check that each component is adjacent in memory
    for (int i = 0; i < count; ++i)
    {
      pose = manager.Component<components::Pose>(keys[i]);
      if (prevPose != nullptr)
      {
        EXPECT_EQ(poseSize, reinterpret_cast<uintptr_t>(pose) -
                            reinterpret_cast<uintptr_t>(prevPose));
      }
      prevPose = pose;
    }
  }
  {
    // Check that the data member of each Component is adjacent in memory
    const math::Pose3d *poseData = nullptr, *prevPoseData = nullptr;
    for (int i = 0; i < count; ++i)
    {
      poseData = &(manager.Component<components::Pose>(keys[i])->Data());
      uintptr_t poseDataSize = sizeof(math::Pose3d) +
        sizeof(components::BaseComponent);
      if (prevPoseData != nullptr)
      {
        EXPECT_EQ(poseDataSize, reinterpret_cast<uintptr_t>(poseData) -
                                reinterpret_cast<uintptr_t>(prevPoseData));
      }
      prevPoseData = poseData;
    }
  }
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, AdjacentMemoryTwoComponentTypes)
{
  std::vector<components::Pose> poses;
  std::vector<IntComponent> ints;
  std::vector<ComponentKey> poseKeys;
  std::vector<ComponentKey> intKeys;

  int count = 100000;

  Entity entity = manager.CreateEntity();
  EXPECT_EQ(1u, entity);

  // Create the components.
  for (int i = 0; i < count; ++i)
  {
    poses.push_back(components::Pose(math::Pose3d(1, 2, 3, 0, 0, 0)));
    ints.push_back(IntComponent(i));

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

  uintptr_t poseSize = sizeof(components::Pose);
  uintptr_t intSize = sizeof(IntComponent);
  const components::Pose *pose = nullptr, *prevPose = nullptr;
  const IntComponent *it = nullptr, *prevIt = nullptr;

  // Check that each component is adjacent in memory
  for (int i = 0; i < count; ++i)
  {
    pose = manager.Component<components::Pose>(poseKeys[i]);
    it = manager.Component<IntComponent>(intKeys[i]);
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
  ComponentKey key{999, 0};

  // Can't remove component from an nonexistent entity
  EXPECT_FALSE(manager.HasEntity(2));
  EXPECT_FALSE(manager.RemoveComponent(2, key));

  // Can't remove a component that doesn't exist.
  EXPECT_EQ(1u, manager.CreateEntity());
  EXPECT_EQ(2u, manager.CreateEntity());
  EXPECT_TRUE(manager.HasEntity(2));
  EXPECT_FALSE(manager.RemoveComponent(2, key));

  // We should get a nullptr if the component type doesn't exist.
  EXPECT_EQ(nullptr, manager.Component<IntComponent>(key));
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
  auto cIntEInt = manager.CreateComponent<IntComponent>(eInt,
      IntComponent(123));
  auto cDoubleEDouble = manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123));
  auto cIntEIntDouble = manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456));
  auto cDoubleEIntDouble = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));

  // Check entities have the components
  EXPECT_TRUE(manager.EntityHasComponent(eInt, cIntEInt));
  EXPECT_EQ(1u, manager.ComponentTypes(eInt).size());
  EXPECT_EQ(IntComponent::typeId, *manager.ComponentTypes(eInt).begin());

  EXPECT_TRUE(manager.EntityHasComponent(eDouble, cDoubleEDouble));
  EXPECT_EQ(1u, manager.ComponentTypes(eDouble).size());
  EXPECT_EQ(DoubleComponent::typeId, *manager.ComponentTypes(eDouble).begin());

  EXPECT_TRUE(manager.EntityHasComponent(eIntDouble, cIntEIntDouble));
  EXPECT_TRUE(manager.EntityHasComponent(eIntDouble, cDoubleEIntDouble));
  EXPECT_EQ(2u, manager.ComponentTypes(eIntDouble).size());
  auto types = manager.ComponentTypes(eIntDouble);
  EXPECT_NE(types.end(), types.find(IntComponent::typeId));
  EXPECT_NE(types.end(), types.find(DoubleComponent::typeId));

  // Remove component by key
  EXPECT_TRUE(manager.RemoveComponent(eInt, cIntEInt));
  EXPECT_FALSE(manager.EntityHasComponent(eInt, cIntEInt));
  EXPECT_TRUE(manager.ComponentTypes(eInt).empty());

  // Remove component by type id
  auto typeDouble = DoubleComponent::typeId;

  EXPECT_TRUE(manager.RemoveComponent(eDouble, typeDouble));
  EXPECT_FALSE(manager.EntityHasComponent(eDouble, cDoubleEDouble));
  EXPECT_TRUE(manager.ComponentTypes(eDouble).empty());

  // Remove component by type
  EXPECT_TRUE(manager.RemoveComponent<IntComponent>(eIntDouble));
  EXPECT_FALSE(manager.EntityHasComponent(eIntDouble, cIntEIntDouble));
  EXPECT_TRUE(manager.EntityHasComponent(eIntDouble, cDoubleEIntDouble));
  EXPECT_EQ(1u, manager.ComponentTypes(eIntDouble).size());

  EXPECT_TRUE(manager.RemoveComponent<DoubleComponent>(eIntDouble));
  EXPECT_FALSE(manager.EntityHasComponent(eIntDouble, cIntEIntDouble));
  EXPECT_FALSE(manager.EntityHasComponent(eIntDouble, cDoubleEIntDouble));
  EXPECT_EQ(0u, manager.ComponentTypes(eIntDouble).size());
}

/////////////////////////////////////////////////
// Removing a component should guarantee that existing components remain
// adjacent to each other.
TEST_P(EntityComponentManagerFixture, RemoveAdjacent)
{
  std::vector<components::Pose> poses;
  std::vector<ComponentKey> keys;

  Entity entity = manager.CreateEntity();

  int count = 3;

  // Create the components.
  for (int i = 0; i < count; ++i)
  {
    poses.push_back(components::Pose(math::Pose3d(1, 2, 3, 0, 0, 0)));
    keys.push_back(manager.CreateComponent(entity, poses.back()));
    EXPECT_EQ(keys.back().second, i);
  }
  ASSERT_EQ(poses.size(), keys.size());

  uintptr_t poseSize = sizeof(components::Pose);
  const components::Pose *pose = nullptr, *prevPose = nullptr;

  // Check that each component is adjacent in memory
  for (int i = 0; i < count; ++i)
  {
    pose = manager.Component<components::Pose>(keys[i]);
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
  const components::Pose *pose1 =
    manager.Component<components::Pose>(keys[0]);
  const components::Pose *pose3 =
    manager.Component<components::Pose>(keys[2]);
  EXPECT_EQ(poseSize,
      reinterpret_cast<uintptr_t>(pose3) - reinterpret_cast<uintptr_t>(pose1));
}

/////////////////////////////////////////////////
// Removing a component should guarantee that existing components remain
// adjacent to each other, and addition of a new component is adjacent to
// the last element.
TEST_P(EntityComponentManagerFixture, RemoveAddAdjacent)
{
  Entity entity = manager.CreateEntity();

  std::vector<ComponentKey> keys;

  keys.push_back(manager.CreateComponent(entity,
        components::Pose(math::Pose3d(1, 2, 3, 0, 0, 0))));
  keys.push_back(manager.CreateComponent(entity,
        components::Pose(math::Pose3d(3, 1, 2, 0, 0, 0))));
  keys.push_back(manager.CreateComponent(entity,
        components::Pose(math::Pose3d(0, 10, 20, 0, 0, 0))));

  uintptr_t poseSize = sizeof(components::Pose);

  // Remove the middle component.
  EXPECT_TRUE(manager.RemoveComponent(entity, keys[1]));

  // Add two more new component
  keys.push_back(manager.CreateComponent(entity,
        components::Pose(math::Pose3d(101, 51, 520, 0, 0, 0))));

  keys.push_back(manager.CreateComponent(entity,
        components::Pose(math::Pose3d(1010, 81, 821, 0, 0, 0))));

  // Check that the components are all adjacent in memory
  const components::Pose *pose1 =
    manager.Component<components::Pose>(keys[0]);
  const components::Pose *pose2 =
    manager.Component<components::Pose>(keys[2]);
  const components::Pose *pose3 =
    manager.Component<components::Pose>(keys[3]);
  const components::Pose *pose4 =
    manager.Component<components::Pose>(keys[4]);

  EXPECT_EQ(poseSize,
      reinterpret_cast<uintptr_t>(pose2) - reinterpret_cast<uintptr_t>(pose1));

  EXPECT_EQ(poseSize,
      reinterpret_cast<uintptr_t>(pose3) - reinterpret_cast<uintptr_t>(pose2));

  EXPECT_EQ(poseSize,
      reinterpret_cast<uintptr_t>(pose4) - reinterpret_cast<uintptr_t>(pose3));

  // Check the values of the components.
  EXPECT_EQ(components::Pose(math::Pose3d(1, 2, 3, 0, 0, 0)), *pose1);
  EXPECT_EQ(components::Pose(math::Pose3d(0, 10, 20, 0, 0, 0)), *pose2);
  EXPECT_EQ(components::Pose(math::Pose3d(101, 51, 520, 0, 0, 0)), *pose3);
  EXPECT_EQ(components::Pose(math::Pose3d(1010, 81, 821, 0, 0, 0)), *pose4);
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EntitiesAndComponents)
{
  EXPECT_EQ(0u, manager.EntityCount());

  // Create a few entities
  Entity entity = manager.CreateEntity();
  Entity entity2 = manager.CreateEntity();
  manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add a component to an entity
  ComponentKey cKey =  manager.CreateComponent<IntComponent>(entity,
      IntComponent(123));

  EXPECT_TRUE(manager.HasComponentType(IntComponent::typeId));
  EXPECT_TRUE(manager.EntityHasComponent(entity, cKey));
  EXPECT_TRUE(manager.EntityHasComponentType(entity, IntComponent::typeId));
  EXPECT_FALSE(manager.EntityHasComponentType(entity, DoubleComponent::typeId));
  EXPECT_FALSE(manager.EntityHasComponentType(entity2, IntComponent::typeId));

  // Remove all entities
  manager.RequestRemoveEntities();
  EXPECT_EQ(3u, manager.EntityCount());
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();

  EXPECT_EQ(0u, manager.EntityCount());
  EXPECT_FALSE(manager.HasEntity(entity));
  EXPECT_FALSE(manager.HasEntity(entity2));
  EXPECT_FALSE(manager.EntityHasComponent(entity, cKey));
  EXPECT_FALSE(manager.EntityHasComponentType(entity, IntComponent::typeId));

  // The type itself still exists
  EXPECT_TRUE(manager.HasComponentType(IntComponent::typeId));
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ComponentValues)
{
  // Create some entities
  Entity eInt = manager.CreateEntity();
  Entity eDouble = manager.CreateEntity();
  Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  manager.CreateComponent<DoubleComponent>(eDouble, DoubleComponent(0.123));
  manager.CreateComponent<IntComponent>(eIntDouble, IntComponent(456));
  manager.CreateComponent<DoubleComponent>(eIntDouble, DoubleComponent(0.456));

  // Get component values
  {
    const auto *value = manager.Component<IntComponent>(eInt);
    ASSERT_NE(nullptr, value);
    EXPECT_EQ(123, value->Data());
  }

  {
    const auto *value = manager.Component<DoubleComponent>(eDouble);
    ASSERT_NE(nullptr, value);
    EXPECT_DOUBLE_EQ(0.123, value->Data());
  }

  {
    const auto *value = manager.Component<IntComponent>(eIntDouble);
    ASSERT_NE(nullptr, value);
    EXPECT_EQ(456, value->Data());
  }

  {
    const auto *value = manager.Component<DoubleComponent>(eIntDouble);
    ASSERT_NE(nullptr, value);
    EXPECT_DOUBLE_EQ(0.456, value->Data());
  }

  // Failure cases
  {
    const auto *value = manager.Component<IntComponent>(eDouble);
    ASSERT_EQ(nullptr, value);
  }

  {
    const auto *value = manager.Component<DoubleComponent>(eInt);
    ASSERT_EQ(nullptr, value);
  }

  {
    const auto *value = manager.Component<IntComponent>(999);
    ASSERT_EQ(nullptr, value);
  }

  {
    const auto *value = manager.Component<DoubleComponent>(999);
    ASSERT_EQ(nullptr, value);
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, RebuildViews)
{
  // Create some entities
  Entity eInt = manager.CreateEntity();
  Entity eDouble = manager.CreateEntity();
  Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  manager.CreateComponent<DoubleComponent>(eDouble, DoubleComponent(0.123));
  manager.CreateComponent<IntComponent>(eIntDouble, IntComponent(456));
  manager.CreateComponent<DoubleComponent>(eIntDouble, DoubleComponent(0.456));

  // The first iteration of this loop builds views. At the end, views are
  // rebuilt. The second iteration should return the same values as the
  // first iteration.
  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    // The first call to each will create a view.
    manager.Each<IntComponent> ([&](const Entity &_entity,
          const IntComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, _value->Data());
          }
          if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, _value->Data());
          }
          ++count;
          return true;
        });
    EXPECT_EQ(2, count);

    count = 0;
    manager.Each<DoubleComponent> ([&](const Entity &_entity,
          const DoubleComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, _value->Data());
          }
          if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, _value->Data());
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
  Entity eInt = manager.CreateEntity();
  Entity eDouble = manager.CreateEntity();
  Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  manager.CreateComponent<DoubleComponent>(eDouble, DoubleComponent(0.123));
  manager.CreateComponent<IntComponent>(eIntDouble, IntComponent(456));
  manager.CreateComponent<DoubleComponent>(eIntDouble, DoubleComponent(0.456));

  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    manager.Each<IntComponent> ([&](const Entity &_entity,
          const IntComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, _value->Data());
          }
          if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, _value->Data());
          }
          ++count;
          return true;
        });
    EXPECT_EQ(2, count);

    count = 0;
    manager.Each<DoubleComponent> ([&](const Entity &_entity,
          const DoubleComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_DOUBLE_EQ(12.123, _value->Data());
          }
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, _value->Data());
          }
          if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, _value->Data());
          }
          ++count;
          return true;
        });
    if (i == 0)
      EXPECT_EQ(2, count);
    else
      EXPECT_EQ(3, count);

    manager.CreateComponent<DoubleComponent>(eInt, DoubleComponent(12.123));
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ViewsRemoveComponents)
{
  // Create some entities
  Entity eInt = manager.CreateEntity();
  Entity eDouble = manager.CreateEntity();
  Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  manager.CreateComponent<DoubleComponent>(eDouble, DoubleComponent(0.123));
  manager.CreateComponent<IntComponent>(eIntDouble, IntComponent(456));
  auto compToRemove = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));

  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    manager.Each<IntComponent> ([&](const Entity &_entity,
          const IntComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, _value->Data());
          }
          if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, _value->Data());
          }
          ++count;
          return true;
        });
    EXPECT_EQ(2, count);

    count = 0;
    manager.Each<DoubleComponent> ([&](const Entity &_entity,
          const DoubleComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_DOUBLE_EQ(12.123, _value->Data());
          }
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, _value->Data());
          }
          if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, _value->Data());
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
  Entity eInt = manager.CreateEntity();
  Entity eDouble = manager.CreateEntity();
  Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  manager.CreateComponent<DoubleComponent>(eDouble, DoubleComponent(0.123));
  manager.CreateComponent<IntComponent>(eIntDouble, IntComponent(456));
  manager.CreateComponent<DoubleComponent>(eIntDouble, DoubleComponent(0.456));

  Entity newEntity;

  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    manager.Each<IntComponent> ([&](const Entity &_entity,
          const IntComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, _value->Data());
          }
          else if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, _value->Data());
          }
          else if (_entity == newEntity)
          {
            EXPECT_EQ(789, _value->Data());
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
    manager.Each<DoubleComponent> ([&](const Entity &_entity,
          const DoubleComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, _value->Data());
          }
          else if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, _value->Data());
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
    manager.CreateComponent<IntComponent>(newEntity, IntComponent(789));
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ViewsRemoveEntities)
{
  // Create some entities
  Entity eInt = manager.CreateEntity();
  Entity eDouble = manager.CreateEntity();
  Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  manager.CreateComponent<DoubleComponent>(eDouble, DoubleComponent(0.123));
  manager.CreateComponent<IntComponent>(eIntDouble, IntComponent(456));
  manager.CreateComponent<DoubleComponent>(eIntDouble, DoubleComponent(0.456));

  for (int i = 0; i < 2; ++i)
  {
    int count = 0;
    manager.Each<IntComponent> ([&](const Entity &_entity,
          const IntComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_EQ(123, _value->Data());
          }
          if (_entity == eIntDouble)
          {
            EXPECT_EQ(456, _value->Data());
          }
          ++count;
          return true;
        });
    if (i == 0)
      EXPECT_EQ(2, count);
    else
      EXPECT_EQ(0, count);

    count = 0;
    manager.Each<DoubleComponent> ([&](const Entity &_entity,
          const DoubleComponent *_value)->bool
        {
          EXPECT_NE(nullptr, _value);
          if (_entity == eInt)
          {
            EXPECT_DOUBLE_EQ(12.123, _value->Data());
          }
          if (_entity == eDouble)
          {
            EXPECT_DOUBLE_EQ(0.123, _value->Data());
          }
          if (_entity == eIntDouble)
          {
            EXPECT_DOUBLE_EQ(0.456, _value->Data());
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
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_FALSE(manager.HasEntitiesMarkedForRemoval());
  EXPECT_EQ(2u, manager.EntityCount());
  EXPECT_FALSE(manager.HasEntity(e2));

  // Creating an new entity shouldn't reuse the previously deleted entity.
  auto e4 = manager.CreateEntity();
  EXPECT_EQ(4u, e4);
  EXPECT_EQ(3u, manager.EntityCount());

  // Can not delete an invalid entity, but it shows up as marked for removal.
  manager.RequestRemoveEntity(6);
  EXPECT_EQ(3u, manager.EntityCount());
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(3u, manager.EntityCount());

  // Delete another
  manager.RequestRemoveEntity(1);
  EXPECT_EQ(3u, manager.EntityCount());
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(2u, manager.EntityCount());

  // Delete another
  manager.RequestRemoveEntity(3);
  EXPECT_EQ(2u, manager.EntityCount());
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(1u, manager.EntityCount());

  // Delete last
  manager.RequestRemoveEntity(4);
  EXPECT_EQ(1u, manager.EntityCount());
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(0u, manager.EntityCount());
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, ViewsRemoveEntity)
{
  // Create some entities
  Entity eInt = manager.CreateEntity();
  Entity eDouble = manager.CreateEntity();
  Entity eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  manager.CreateComponent<DoubleComponent>(eDouble, DoubleComponent(0.123));
  manager.CreateComponent<IntComponent>(eIntDouble, IntComponent(456));
  manager.CreateComponent<DoubleComponent>(eIntDouble, DoubleComponent(0.456));

  int count = 0;
  manager.Each<IntComponent> ([&](const Entity &_entity,
        const IntComponent *_value)->bool
      {
        EXPECT_NE(nullptr, _value);
        if (_entity == eInt)
        {
          EXPECT_EQ(123, _value->Data());
        }
        if (_entity == eIntDouble)
        {
          EXPECT_EQ(456, _value->Data());
        }
        ++count;
        return true;
      });
  EXPECT_EQ(2, count);

  // Remove an entity.
  manager.RequestRemoveEntity(eIntDouble);
  manager.ProcessEntityRemovals();

  count = 0;
  manager.Each<IntComponent> ([&](const Entity &_entity,
        const IntComponent *_value)->bool
      {
        EXPECT_NE(nullptr, _value);
        EXPECT_NE(eIntDouble, _entity);
        if (_entity == eInt)
        {
          EXPECT_EQ(123, _value->Data());
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
      [&](const Entity &, Ts *... _values) -> bool
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
      [&](const Entity &, const Ts *... _values) -> bool
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
      [&](const Entity &, const Ts *... _values) -> bool
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
      [&](const Entity &, const Ts *... _values) -> bool
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
  Entity e1 = manager.CreateEntity();
  Entity e2 = manager.CreateEntity();
  EXPECT_EQ(2u, manager.EntityCount());

  // Add components to each entity
  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.CreateComponent<IntComponent>(e2, IntComponent(456));

  EXPECT_EQ(2, newCount<IntComponent>(manager));
  EXPECT_TRUE(manager.HasNewEntities());

  // This would normally be done after each simulation step after systems are
  // updated
  manager.RunClearNewlyCreatedEntities();
  EXPECT_EQ(0, newCount<IntComponent>(manager));
  EXPECT_FALSE(manager.HasNewEntities());
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewAfterRemoveComponent)
{
  // Create entities
  Entity e1 = manager.CreateEntity();
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.CreateComponent<DoubleComponent>(e1, DoubleComponent(0.0));

  EXPECT_EQ(1, newCount<IntComponent>(manager));

  manager.RemoveComponent(e1, comp1);
  EXPECT_EQ(1, newCount<DoubleComponent>(manager));

  manager.RunClearNewlyCreatedEntities();
  EXPECT_EQ(0, newCount<DoubleComponent>(manager));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewRemoveComponentFromRemoveEntity)
{
  // Create entities
  Entity e1 = manager.CreateEntity();
  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.RunClearNewlyCreatedEntities();
  // Nothing new after cleared
  EXPECT_EQ(0, newCount<IntComponent>(manager));

  Entity e2 = manager.CreateEntity();
  manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  EXPECT_EQ(1, newCount<IntComponent>(manager));
  // Check if this true after RebuildViews
  manager.RebuildViews();
  EXPECT_EQ(1, newCount<IntComponent>(manager));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewAddComponentToExistingEntity)
{
  // Create entities
  Entity e1 = manager.CreateEntity();
  Entity e2 = manager.CreateEntity();
  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  manager.RunClearNewlyCreatedEntities();
  // Nothing new after cleared
  EXPECT_EQ(0, newCount<IntComponent>(manager));

  // Create a new entity
  Entity e3 = manager.CreateEntity();
  manager.CreateComponent<IntComponent>(e3, IntComponent(789));
  // Add a new component to existing entities
  manager.CreateComponent<DoubleComponent>(e1, DoubleComponent(0.0));
  manager.CreateComponent<DoubleComponent>(e2, DoubleComponent(2.0));

  // e1 and e2 have a new double component, but they are not considered new
  // entities
  EXPECT_EQ(0, (newCount<IntComponent, DoubleComponent>(manager)));
  // Only e3 is considered new
  EXPECT_EQ(1, newCount<IntComponent>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveBasic)
{
  // Create an entities
  Entity e1 = manager.CreateEntity();
  Entity e2 = manager.CreateEntity();
  EXPECT_EQ(2u, manager.EntityCount());

  // Add components to each entity
  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.CreateComponent<IntComponent>(e2, IntComponent(456));

  // Remove an entity.
  manager.RequestRemoveEntity(e1);
  EXPECT_EQ(1, removedCount<IntComponent>(manager));
  manager.RequestRemoveEntity(e2);
  EXPECT_EQ(2, removedCount<IntComponent>(manager));

  // This would normally be done after each simulation step after systems are
  // updated
  EXPECT_TRUE(manager.HasNewEntities());
  manager.RunClearNewlyCreatedEntities();
  EXPECT_FALSE(manager.HasNewEntities());
  // But it shouldn't affect removed entities
  EXPECT_EQ(2, removedCount<IntComponent>(manager));

  manager.ProcessEntityRemovals();
  EXPECT_EQ(0, removedCount<IntComponent>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveAlreadyRemove)
{
  // Create an entities
  Entity e1 = manager.CreateEntity();
  Entity e2 = manager.CreateEntity();
  EXPECT_EQ(2u, manager.EntityCount());

  // Add components to each entity
  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  manager.RequestRemoveEntity(e2);

  manager.ProcessEntityRemovals();

  // try erasing an already removed entity
  manager.RequestRemoveEntity(e2);
  EXPECT_EQ(0, removedCount<IntComponent>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveAfterRebuild)
{
  // Test after rebuild
  Entity e1 = manager.CreateEntity();
  EXPECT_EQ(1u, manager.EntityCount());

  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  EXPECT_EQ(1, newCount<IntComponent>(manager));
  manager.RunClearNewlyCreatedEntities();

  manager.RequestRemoveEntity(e1);
  EXPECT_EQ(1, removedCount<IntComponent>(manager));

  manager.RebuildViews();
  EXPECT_EQ(1, removedCount<IntComponent>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveAddComponentToRemoveEntity)
{
  Entity e1 = manager.CreateEntity();
  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.RunClearNewlyCreatedEntities();
  manager.RequestRemoveEntity(e1);

  // Add a new component to an removed entity. This should be possible since the
  // entity is only scheduled to be removed.
  manager.CreateComponent<DoubleComponent>(e1, DoubleComponent(0.0));
  EXPECT_EQ(1, removedCount<IntComponent>(manager));
  EXPECT_EQ(1, (removedCount<IntComponent, DoubleComponent>(manager)));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveAllRemove)
{
  // Test when all entities are removed
  Entity e1 = manager.CreateEntity();
  Entity e2 = manager.CreateEntity();
  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  EXPECT_EQ(2u, manager.EntityCount());

  manager.RequestRemoveEntities();
  EXPECT_EQ(2, removedCount<IntComponent>(manager));

  manager.ProcessEntityRemovals();
  EXPECT_EQ(0, removedCount<IntComponent>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewEachRemove)
{
  // Test EachNew and EachRemove together
  Entity e1 = manager.CreateEntity();
  Entity e2 = manager.CreateEntity();
  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  EXPECT_EQ(2u, manager.EntityCount());

  EXPECT_EQ(2, newCount<IntComponent>(manager));
  EXPECT_EQ(0, removedCount<IntComponent>(manager));

  // Remove an entity.
  manager.RequestRemoveEntity(e1);
  // An entity can be considered new even if there is a request to remove it.
  EXPECT_EQ(2, newCount<IntComponent>(manager));
  EXPECT_EQ(1, removedCount<IntComponent>(manager));

  // ProcessEntityRemovals and ClearNewlyCreatedEntities would be called
  // together after a simulation step
  manager.RunClearNewlyCreatedEntities();
  manager.ProcessEntityRemovals();

  EXPECT_EQ(0, newCount<IntComponent>(manager));
  EXPECT_EQ(0, removedCount<IntComponent>(manager));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachGetsNewOldRemove)
{
  // Test that an Each call gets new, old, and removed entities
  Entity e1 = manager.CreateEntity();
  Entity e2 = manager.CreateEntity();
  manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  EXPECT_EQ(2u, manager.EntityCount());

  EXPECT_EQ(2, eachCount<IntComponent>(manager));
  EXPECT_EQ(2, newCount<IntComponent>(manager));
  EXPECT_EQ(0, removedCount<IntComponent>(manager));

  // Remove an entity.
  manager.RequestRemoveEntity(e1);
  // Each gets entities that removed
  EXPECT_EQ(2, eachCount<IntComponent>(manager));
  // An entity can be considered new even if there is a request to remove it.
  EXPECT_EQ(2, newCount<IntComponent>(manager));
  EXPECT_EQ(1, removedCount<IntComponent>(manager));

  // ProcessEntityRemovals and ClearNewlyCreatedEntities would be called
  // together after a simulation step
  manager.RunClearNewlyCreatedEntities();
  manager.ProcessEntityRemovals();

  // One entity is removed, one left
  EXPECT_EQ(1, eachCount<IntComponent>(manager));
  EXPECT_EQ(0, newCount<IntComponent>(manager));
  EXPECT_EQ(0, removedCount<IntComponent>(manager));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EntityByComponents)
{
  // Create some entities
  Entity eInt = manager.CreateEntity();
  Entity eUint = manager.CreateEntity();
  Entity eIntUint = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components of different types to each entity
  manager.CreateComponent<IntComponent>(eInt, IntComponent(-123));
  manager.CreateComponent<StringComponent>(eInt, StringComponent("int"));

  manager.CreateComponent<UIntComponent>(eUint, UIntComponent(456u));
  manager.CreateComponent<StringComponent>(eUint, StringComponent("uint"));

  manager.CreateComponent<IntComponent>(eIntUint, IntComponent(789));
  manager.CreateComponent<UIntComponent>(eIntUint, UIntComponent(789u));
  manager.CreateComponent<StringComponent>(eIntUint,
      StringComponent("int-uint"));

  // Get entities by the value of their components
  EXPECT_EQ(eInt, manager.EntityByComponents(IntComponent(-123)));
  EXPECT_EQ(eInt, manager.EntityByComponents(StringComponent("int")));
  EXPECT_EQ(eInt, manager.EntityByComponents(StringComponent("int"),
      IntComponent(-123)));

  EXPECT_EQ(eUint, manager.EntityByComponents(UIntComponent(456u)));
  EXPECT_EQ(eUint, manager.EntityByComponents(StringComponent("uint")));
  EXPECT_EQ(eUint, manager.EntityByComponents(StringComponent("uint"),
      UIntComponent(456u)));

  EXPECT_EQ(eIntUint, manager.EntityByComponents(IntComponent(789)));
  EXPECT_EQ(eIntUint, manager.EntityByComponents(UIntComponent(789u)));
  EXPECT_EQ(eIntUint, manager.EntityByComponents(StringComponent("int-uint")));
  EXPECT_EQ(eIntUint, manager.EntityByComponents(IntComponent(789),
      UIntComponent(789u)));
  EXPECT_EQ(eIntUint, manager.EntityByComponents(StringComponent("int-uint"),
      IntComponent(789), UIntComponent(789u)));

  EXPECT_EQ(kNullEntity, manager.EntityByComponents(IntComponent(123456)));
  EXPECT_EQ(kNullEntity, manager.EntityByComponents(UIntComponent(123u)));
  EXPECT_EQ(kNullEntity, manager.EntityByComponents(BoolComponent(true)));
  EXPECT_EQ(kNullEntity, manager.EntityByComponents(StringComponent("123456")));
  EXPECT_EQ(kNullEntity, manager.EntityByComponents(StringComponent("int"),
      UIntComponent(456u)));
  EXPECT_EQ(kNullEntity, manager.EntityByComponents(UIntComponent(456u),
      UIntComponent(789u)));
  EXPECT_EQ(kNullEntity, manager.EntityByComponents(IntComponent(-123),
      UIntComponent(456u)));

  // Multiple entities
  Entity eInt2 = manager.CreateEntity();
  EXPECT_EQ(4u, manager.EntityCount());

  manager.CreateComponent<IntComponent>(eInt2, IntComponent(-123));
  manager.CreateComponent<StringComponent>(eInt2, StringComponent("int2"));

  auto entities = manager.EntitiesByComponents(IntComponent(-123));
  EXPECT_EQ(2u, entities.size());

  entities = manager.EntitiesByComponents(StringComponent("int2"));
  EXPECT_EQ(1u, entities.size());
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
  manager.CreateComponent<Even>(e2, {});
  manager.CreateComponent<Even>(e4, {});
  manager.CreateComponent<Even>(e6, {});

  manager.CreateComponent<Odd>(e1, {});
  manager.CreateComponent<Odd>(e3, {});
  manager.CreateComponent<Odd>(e5, {});
  manager.CreateComponent<Odd>(e7, {});

  // Get children by components
  {
    auto result = manager.ChildrenByComponents(e1, Even());
    ASSERT_GE(1u, result.size());
    EXPECT_EQ(e2, result.front());
  }
  {
    auto result = manager.ChildrenByComponents(e1, Odd());
    ASSERT_GE(1u, result.size());
    EXPECT_EQ(e3, result.front());
  }
  {
    auto result = manager.ChildrenByComponents(e2, Even());
    ASSERT_GE(2u, result.size());
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
    ASSERT_GE(1u, result.size());
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

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, State)
{
  // Entities and components
  Entity e1{1};
  Entity e2{2};
  Entity e3{3};
  Entity e4{4};

  int e1c0{123};
  double e1c1{0.456};
  double e2c0{0.123};
  std::string e2c1{"string"};
  int e3c0{456};
  int e3c0New{654};
  int e4c0{789u};

  // Fill manager with entities and components
  {
    // Entities
    EXPECT_EQ(e1, manager.CreateEntity());
    EXPECT_EQ(e2, manager.CreateEntity());
    EXPECT_EQ(e3, manager.CreateEntity());
    EXPECT_EQ(3u, manager.EntityCount());

    // Components
    manager.CreateComponent<IntComponent>(e1, IntComponent(e1c0));
    manager.CreateComponent<DoubleComponent>(e2, DoubleComponent(e2c0));
    manager.CreateComponent<StringComponent>(e2, StringComponent(e2c1));
    manager.CreateComponent<IntComponent>(e3, IntComponent(e3c0));
  }

  // Serialize into a message
  msgs::SerializedStateMap stateMsg;
  manager.State(stateMsg);

  // Check message
  {
    ASSERT_EQ(3, stateMsg.entities_size());

    auto iter = stateMsg.entities().find(e1);
    const auto &e1Msg = iter->second;
    EXPECT_EQ(e1, e1Msg.id());
    ASSERT_EQ(1, e1Msg.components_size());

    auto compIter = e1Msg.components().begin();
    const auto &e1c0Msg = compIter->second;
    EXPECT_EQ(IntComponent::typeId, e1c0Msg.type());
    EXPECT_EQ(e1c0, std::stoi(e1c0Msg.component()));

    iter = stateMsg.entities().find(e2);
    const auto &e2Msg = iter->second;
    EXPECT_EQ(e2, e2Msg.id());
    ASSERT_EQ(2u, e2Msg.components().size());

    compIter = e2Msg.components().begin();
    if (compIter->second.type() == DoubleComponent::typeId)
    {
      const auto &e2c0Msg = compIter->second;
      EXPECT_EQ(DoubleComponent::typeId, e2c0Msg.type());
      EXPECT_DOUBLE_EQ(e2c0, std::stod(e2c0Msg.component()));
    }
    else
    {
      const auto &e2c1Msg = compIter->second;
      EXPECT_EQ(StringComponent::typeId, e2c1Msg.type());
      EXPECT_EQ(e2c1, e2c1Msg.component());
    }

    compIter++;
    if (compIter->second.type() == DoubleComponent::typeId)
    {
      const auto &e2c0Msg = compIter->second;
      EXPECT_EQ(DoubleComponent::typeId, e2c0Msg.type());
      EXPECT_DOUBLE_EQ(e2c0, std::stod(e2c0Msg.component()));
    }
    else
    {
      const auto &e2c1Msg = compIter->second;
      EXPECT_EQ(StringComponent::typeId, e2c1Msg.type());
      EXPECT_EQ(e2c1, e2c1Msg.component());
    }

    iter = stateMsg.entities().find(e3);
    const auto &e3Msg = iter->second;
    EXPECT_EQ(e3, e3Msg.id());
    ASSERT_EQ(1u, e1Msg.components().size());

    const auto &e3c0Msg = e3Msg.components().begin()->second;
    EXPECT_EQ(IntComponent::typeId, e3c0Msg.type());
    EXPECT_EQ(e3c0, std::stoi(e3c0Msg.component()));
  }

  // Serialize changed state into a message, it should be the same
  {
    auto changedStateMsg = manager.ChangedState();
    EXPECT_EQ(3, changedStateMsg.entities_size());
  }

  // Mark entities as not new
  manager.RunClearNewlyCreatedEntities();

  // Changed state should be empty
  {
    auto changedStateMsg = manager.ChangedState();
    EXPECT_EQ(0, changedStateMsg.entities_size());
  }

  // Deserialize into a new ECM
  EntityComponentManager newEcm;
  newEcm.SetState(stateMsg);

  // Check ECM
  {
    EXPECT_EQ(3u, newEcm.EntityCount());

    EXPECT_TRUE(newEcm.HasEntity(e1));

    EXPECT_TRUE(newEcm.HasComponentType(IntComponent::typeId));
    const auto &e1c0Comp = newEcm.Component<IntComponent>(e1);
    ASSERT_NE(nullptr, e1c0Comp);
    EXPECT_EQ(e1c0, e1c0Comp->Data());

    EXPECT_TRUE(newEcm.HasEntity(e2));

    EXPECT_TRUE(newEcm.HasComponentType(DoubleComponent::typeId));
    const auto &e2c0Comp = newEcm.Component<DoubleComponent>(e2);
    ASSERT_NE(nullptr, e2c0Comp);
    EXPECT_DOUBLE_EQ(e2c0, e2c0Comp->Data());

    EXPECT_TRUE(newEcm.HasComponentType(StringComponent::typeId));
    const auto &e2c1Comp = newEcm.Component<StringComponent>(e2);
    ASSERT_NE(nullptr, e2c1Comp);
    EXPECT_EQ(e2c1, e2c1Comp->Data());

    EXPECT_TRUE(newEcm.HasEntity(e3));

    EXPECT_TRUE(newEcm.HasComponentType(IntComponent::typeId));
    const auto &e3c0Comp = newEcm.Component<IntComponent>(e3);
    ASSERT_NE(nullptr, e3c0Comp);
    EXPECT_EQ(e3c0, e3c0Comp->Data());
  }

  // Update message to change entities / components
  {
    // e1 has a component removed and another added
    google::protobuf::Map<uint64_t, msgs::SerializedEntityMap>::iterator iter
      = stateMsg.mutable_entities()->find(e1);
    ASSERT_TRUE(iter != stateMsg.mutable_entities()->end());

    msgs::SerializedEntityMap &e1Msg = iter->second;

    EXPECT_EQ(1, e1Msg.components_size());
    msgs::SerializedComponent &e1c0Msg =
      e1Msg.mutable_components()->begin()->second;
    e1c0Msg.set_remove(true);

    msgs::SerializedComponent e1c1Msg;
    e1c1Msg.set_type(DoubleComponent::typeId);
    e1c1Msg.set_component(std::to_string(e1c1));
    (*e1Msg.mutable_components())[e1c1Msg.type()] = e1c1Msg;

    // e2 is removed
    iter = stateMsg.mutable_entities()->find(e2);
    msgs::SerializedEntityMap &e2Msg = iter->second;
    e2Msg.set_remove(true);

    // e3 has a component updated
    iter = stateMsg.mutable_entities()->find(e3);
    msgs::SerializedEntityMap &e3Msg = iter->second;

    ASSERT_EQ(1, e3Msg.components_size());
    msgs::SerializedComponent &e3c0Msg =
      e3Msg.mutable_components()->begin()->second;
    e3c0Msg.set_component(std::to_string(e3c0New));

    // e4 is a new entity
    msgs::SerializedEntityMap e4Msg;
    e4Msg.set_id(e4);
    msgs::SerializedComponent e4c0Msg;
    e4c0Msg.set_type(IntComponent::typeId);
    e4c0Msg.set_component(std::to_string(e4c0));
    (*e4Msg.mutable_components())[e4c0Msg.type()] = e4c0Msg;
    (*stateMsg.mutable_entities())[static_cast<int64_t>(e4)] = e4Msg;
  }

  // Set new state on top of previous one
  manager.SetState(stateMsg);

  // Check ECM was properly updated
  {
    // Check the changed state
    auto changedStateMsg = manager.ChangedState();
    EXPECT_EQ(2, changedStateMsg.entities_size());

    const auto &e4Msg = changedStateMsg.entities(0);
    EXPECT_EQ(e4, e4Msg.id());
    EXPECT_FALSE(e4Msg.remove());
    EXPECT_EQ(1, e4Msg.components().size());

    const auto &e2Msg = changedStateMsg.entities(1);
    EXPECT_EQ(e2, e2Msg.id());
    EXPECT_TRUE(e2Msg.remove());
    EXPECT_EQ(2, e2Msg.components().size());

    // Check that e2 has been marked for removal
    EXPECT_EQ(4u, manager.EntityCount());
    EXPECT_TRUE(manager.HasEntity(e2));
    EXPECT_EQ(1, removedCount<DoubleComponent>(manager));

    // Process removal
    manager.ProcessEntityRemovals();

    EXPECT_EQ(3u, manager.EntityCount());

    // e1 is still there
    EXPECT_TRUE(manager.HasEntity(e1));

    // e1c0 is gone
    const auto &e1c0Comp = manager.Component<IntComponent>(e1);
    EXPECT_EQ(nullptr, e1c0Comp);

    // e1c1 is new
    const auto &e1c1Comp = manager.Component<DoubleComponent>(e1);
    ASSERT_NE(nullptr, e1c1Comp);
    EXPECT_DOUBLE_EQ(e1c1, e1c1Comp->Data());

    // e2 was removed
    EXPECT_FALSE(manager.HasEntity(e2));

    // e3 is still there
    EXPECT_TRUE(manager.HasEntity(e3));

    // e3c0 is updated
    const auto &e3c0Comp = manager.Component<IntComponent>(e3);
    EXPECT_NE(nullptr, e3c0Comp);
    EXPECT_EQ(e3c0New, e3c0Comp->Data());

    // e4 was created
    EXPECT_TRUE(manager.HasEntity(e4));

    const auto &e4c0Comp = manager.Component<IntComponent>(e4);
    ASSERT_NE(nullptr, e4c0Comp);
    EXPECT_DOUBLE_EQ(e4c0, e4c0Comp->Data());
  }

  // Serialize into a message with selected entities and components
  {
    msgs::SerializedStateMap stateMsg2;
    manager.State(stateMsg2, {e3, e4}, {IntComponent::typeId});

    ASSERT_EQ(2, stateMsg2.entities_size());

    auto iter = stateMsg2.entities().find(e3);
    const auto &e3Msg = iter->second;
    EXPECT_EQ(e3, e3Msg.id());
    ASSERT_EQ(1u, e3Msg.components().size());

    auto compIter = e3Msg.components().begin();
    const auto &e3c0Msg = compIter->second;
    EXPECT_EQ(IntComponent::typeId, e3c0Msg.type());
    EXPECT_EQ(e3c0New, std::stoi(e3c0Msg.component()));

    iter = stateMsg2.entities().find(e4);
    const auto &e4Msg = iter->second;
    EXPECT_EQ(e4, e4Msg.id());
    ASSERT_EQ(1u, e4Msg.components().size());

    auto compIter4 = e4Msg.components().begin();
    const auto &e4c0Msg = compIter4->second;
    EXPECT_EQ(IntComponent::typeId, e4c0Msg.type());
    EXPECT_EQ(e4c0, std::stoi(e4c0Msg.component()));
  }
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, Descendants)
{
  // - 1
  //   - 2
  //   - 3
  //     - 4
  //       - 5
  //       - 6
  // - 7
  //   - 8

  auto e1 = manager.CreateEntity();

  auto e2 = manager.CreateEntity();
  manager.SetParentEntity(e2, e1);

  auto e3 = manager.CreateEntity();
  manager.SetParentEntity(e3, e1);

  {
    auto ds = manager.Descendants(e1);
    EXPECT_EQ(3u, ds.size());
    EXPECT_NE(ds.end(), ds.find(e1));
    EXPECT_NE(ds.end(), ds.find(e2));
    EXPECT_NE(ds.end(), ds.find(e3));
  }

  {
    auto ds = manager.Descendants(e2);
    EXPECT_EQ(1u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_NE(ds.end(), ds.find(e2));
    EXPECT_EQ(ds.end(), ds.find(e3));
  }

  {
    auto ds = manager.Descendants(e3);
    EXPECT_EQ(1u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_EQ(ds.end(), ds.find(e2));
    EXPECT_NE(ds.end(), ds.find(e3));
  }

  auto e4 = manager.CreateEntity();
  manager.SetParentEntity(e4, e3);

  auto e5 = manager.CreateEntity();
  manager.SetParentEntity(e5, e4);

  auto e6 = manager.CreateEntity();
  manager.SetParentEntity(e6, e4);

  auto e7 = manager.CreateEntity();

  auto e8 = manager.CreateEntity();
  manager.SetParentEntity(e8, e7);

  {
    auto ds = manager.Descendants(e1);
    EXPECT_EQ(6u, ds.size());
    EXPECT_NE(ds.end(), ds.find(e1));
    EXPECT_NE(ds.end(), ds.find(e2));
    EXPECT_NE(ds.end(), ds.find(e3));
    EXPECT_NE(ds.end(), ds.find(e4));
    EXPECT_NE(ds.end(), ds.find(e5));
    EXPECT_NE(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_EQ(ds.end(), ds.find(e8));
  }

  {
    // cached values
    auto ds = manager.Descendants(e1);
    EXPECT_EQ(6u, ds.size());
    EXPECT_NE(ds.end(), ds.find(e1));
    EXPECT_NE(ds.end(), ds.find(e2));
    EXPECT_NE(ds.end(), ds.find(e3));
    EXPECT_NE(ds.end(), ds.find(e4));
    EXPECT_NE(ds.end(), ds.find(e5));
    EXPECT_NE(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_EQ(ds.end(), ds.find(e8));
  }

  {
    auto ds = manager.Descendants(e2);
    EXPECT_EQ(1u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_NE(ds.end(), ds.find(e2));
    EXPECT_EQ(ds.end(), ds.find(e3));
    EXPECT_EQ(ds.end(), ds.find(e4));
    EXPECT_EQ(ds.end(), ds.find(e5));
    EXPECT_EQ(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_EQ(ds.end(), ds.find(e8));
  }

  {
    auto ds = manager.Descendants(e3);
    EXPECT_EQ(4u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_EQ(ds.end(), ds.find(e2));
    EXPECT_NE(ds.end(), ds.find(e3));
    EXPECT_NE(ds.end(), ds.find(e4));
    EXPECT_NE(ds.end(), ds.find(e5));
    EXPECT_NE(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_EQ(ds.end(), ds.find(e8));
  }

  {
    auto ds = manager.Descendants(e4);
    EXPECT_EQ(3u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_EQ(ds.end(), ds.find(e2));
    EXPECT_EQ(ds.end(), ds.find(e3));
    EXPECT_NE(ds.end(), ds.find(e4));
    EXPECT_NE(ds.end(), ds.find(e5));
    EXPECT_NE(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_EQ(ds.end(), ds.find(e8));
  }

  {
    auto ds = manager.Descendants(e5);
    EXPECT_EQ(1u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_EQ(ds.end(), ds.find(e2));
    EXPECT_EQ(ds.end(), ds.find(e3));
    EXPECT_EQ(ds.end(), ds.find(e4));
    EXPECT_NE(ds.end(), ds.find(e5));
    EXPECT_EQ(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_EQ(ds.end(), ds.find(e8));
  }

  {
    auto ds = manager.Descendants(e6);
    EXPECT_EQ(1u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_EQ(ds.end(), ds.find(e2));
    EXPECT_EQ(ds.end(), ds.find(e3));
    EXPECT_EQ(ds.end(), ds.find(e4));
    EXPECT_EQ(ds.end(), ds.find(e5));
    EXPECT_NE(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_EQ(ds.end(), ds.find(e8));
  }

  {
    auto ds = manager.Descendants(e7);
    EXPECT_EQ(2u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_EQ(ds.end(), ds.find(e2));
    EXPECT_EQ(ds.end(), ds.find(e3));
    EXPECT_EQ(ds.end(), ds.find(e4));
    EXPECT_EQ(ds.end(), ds.find(e5));
    EXPECT_EQ(ds.end(), ds.find(e6));
    EXPECT_NE(ds.end(), ds.find(e7));
    EXPECT_NE(ds.end(), ds.find(e8));
  }

  {
    auto ds = manager.Descendants(e8);
    EXPECT_EQ(1u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_EQ(ds.end(), ds.find(e2));
    EXPECT_EQ(ds.end(), ds.find(e3));
    EXPECT_EQ(ds.end(), ds.find(e4));
    EXPECT_EQ(ds.end(), ds.find(e5));
    EXPECT_EQ(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_NE(ds.end(), ds.find(e8));
  }

  manager.RequestRemoveEntity(e3);
  manager.ProcessEntityRemovals();

  {
    auto ds = manager.Descendants(e1);
    EXPECT_EQ(2u, ds.size());
    EXPECT_NE(ds.end(), ds.find(e1));
    EXPECT_NE(ds.end(), ds.find(e2));
    EXPECT_EQ(ds.end(), ds.find(e3));
    EXPECT_EQ(ds.end(), ds.find(e4));
    EXPECT_EQ(ds.end(), ds.find(e5));
    EXPECT_EQ(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_EQ(ds.end(), ds.find(e8));
  }

  {
    auto ds = manager.Descendants(e2);
    EXPECT_EQ(1u, ds.size());
    EXPECT_EQ(ds.end(), ds.find(e1));
    EXPECT_NE(ds.end(), ds.find(e2));
    EXPECT_EQ(ds.end(), ds.find(e3));
    EXPECT_EQ(ds.end(), ds.find(e4));
    EXPECT_EQ(ds.end(), ds.find(e5));
    EXPECT_EQ(ds.end(), ds.find(e6));
    EXPECT_EQ(ds.end(), ds.find(e7));
    EXPECT_EQ(ds.end(), ds.find(e8));
  }

  {
    auto ds = manager.Descendants(e3);
    EXPECT_TRUE(ds.empty());
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, SetChanged)
{
  // Create entities
  Entity e1 = manager.CreateEntity();
  Entity e2 = manager.CreateEntity();
  EXPECT_EQ(2u, manager.EntityCount());

  // Add components to each entity
  auto c1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  auto c2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));

  EXPECT_TRUE(manager.HasOneTimeComponentChanges());
  EXPECT_EQ(ComponentState::OneTimeChange,
      manager.ComponentState(e1, c1.first));
  EXPECT_EQ(ComponentState::OneTimeChange,
      manager.ComponentState(e2, c2.first));
  EXPECT_EQ(ComponentState::NoChange, manager.ComponentState(999, 888));
  EXPECT_EQ(ComponentState::NoChange, manager.ComponentState(e1, 888));

  // This would normally be done after each simulation step after systems are
  // updated
  manager.RunSetAllComponentsUnchanged();
  EXPECT_FALSE(manager.HasOneTimeComponentChanges());
  EXPECT_EQ(ComponentState::NoChange,
      manager.ComponentState(e1, c1.first));
  EXPECT_EQ(ComponentState::NoChange,
      manager.ComponentState(e2, c2.first));

  // Mark as changed
  manager.SetChanged(e1, c1.first, ComponentState::PeriodicChange);
  manager.SetChanged(e2, c2.first, ComponentState::OneTimeChange);

  EXPECT_TRUE(manager.HasOneTimeComponentChanges());
  EXPECT_EQ(ComponentState::PeriodicChange,
      manager.ComponentState(e1, c1.first));
  EXPECT_EQ(ComponentState::OneTimeChange,
      manager.ComponentState(e2, c2.first));

  // Remove components
  EXPECT_TRUE(manager.RemoveComponent(e1, c1.first));

  EXPECT_TRUE(manager.HasOneTimeComponentChanges());
  EXPECT_EQ(ComponentState::NoChange,
      manager.ComponentState(e1, c1.first));

  EXPECT_TRUE(manager.RemoveComponent(e2, c2.first));

  EXPECT_FALSE(manager.HasOneTimeComponentChanges());
  EXPECT_EQ(ComponentState::NoChange,
      manager.ComponentState(e2, c2.first));
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_CASE_P(EntityComponentManagerRepeat,
    EntityComponentManagerFixture, ::testing::Range(1, 10));
