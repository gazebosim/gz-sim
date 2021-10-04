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
#include <ignition/common/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <ignition/utils/SuppressWarning.hh>

#include "ignition/gazebo/components/CanonicalLink.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/config.hh"
#include "../test/helpers/EnvTestFixture.hh"

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

struct Custom
{
  int dummy{123};
};

using CustomComponent = components::Component<Custom, class CustomTag>;
IGN_GAZEBO_REGISTER_COMPONENT("ign_gazebo_components.CustomComponent",
    CustomComponent)
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
  public: void RunClearRemovedComponents()
  {
    this->ClearRemovedComponents();
  }
};

class EntityComponentManagerFixture
  : public InternalFixture<::testing::TestWithParam<int>>
{
  public: EntityCompMgrTest manager;
};

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, InvalidComponentType)
{
  // Can't remove component from an nonexistent entity
  EXPECT_FALSE(manager.HasEntity(2));
  EXPECT_FALSE(manager.RemoveComponent(2, IntComponent::typeId));

  // Can't remove a component that doesn't exist.
  EXPECT_EQ(1u, manager.CreateEntity());
  EXPECT_EQ(2u, manager.CreateEntity());
  EXPECT_TRUE(manager.HasEntity(2));
  EXPECT_FALSE(manager.RemoveComponent(2, IntComponent::typeId));

  // We should get a nullptr if the component type doesn't exist.
  EXPECT_TRUE(manager.HasEntity(1u));
  EXPECT_TRUE(manager.HasEntity(2u));
  EXPECT_EQ(nullptr, manager.Component<IntComponent>(1u));
  EXPECT_EQ(nullptr, manager.Component<IntComponent>(2u));
}

/////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, RemoveComponent)
{
  // Create some entities
  auto eInt = manager.CreateEntity();
  auto eDouble = manager.CreateEntity();
  auto eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components
  auto cIntEInt = manager.CreateComponent<IntComponent>(eInt,
      IntComponent(123));
  ASSERT_NE(nullptr, cIntEInt);
  auto cDoubleEDouble = manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123));
  ASSERT_NE(nullptr, cDoubleEDouble);
  auto cIntEIntDouble = manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456));
  ASSERT_NE(nullptr, cIntEIntDouble);
  auto cDoubleEIntDouble = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));
  ASSERT_NE(nullptr, cDoubleEIntDouble);

  // Check entities have the components
  EXPECT_TRUE(manager.EntityHasComponentType(eInt, IntComponent::typeId));
  EXPECT_EQ(1u, manager.ComponentTypes(eInt).size());
  EXPECT_EQ(IntComponent::typeId, *manager.ComponentTypes(eInt).begin());
  EXPECT_EQ(cIntEInt, manager.Component<IntComponent>(eInt));

  EXPECT_TRUE(manager.EntityHasComponentType(eDouble, DoubleComponent::typeId));
  EXPECT_EQ(1u, manager.ComponentTypes(eDouble).size());
  EXPECT_EQ(DoubleComponent::typeId, *manager.ComponentTypes(eDouble).begin());
  EXPECT_EQ(cDoubleEDouble, manager.Component<DoubleComponent>(eDouble));

  EXPECT_TRUE(manager.EntityHasComponentType(eIntDouble, IntComponent::typeId));
  EXPECT_TRUE(manager.EntityHasComponentType(eIntDouble,
        DoubleComponent::typeId));
  EXPECT_EQ(2u, manager.ComponentTypes(eIntDouble).size());
  auto types = manager.ComponentTypes(eIntDouble);
  EXPECT_NE(types.end(), types.find(IntComponent::typeId));
  EXPECT_NE(types.end(), types.find(DoubleComponent::typeId));
  EXPECT_EQ(cIntEIntDouble, manager.Component<IntComponent>(eIntDouble));
  EXPECT_EQ(cDoubleEIntDouble, manager.Component<DoubleComponent>(eIntDouble));

  // Remove component by type id
  EXPECT_TRUE(manager.RemoveComponent(eInt, IntComponent::typeId));
  EXPECT_FALSE(manager.EntityHasComponentType(eInt, IntComponent::typeId));
  EXPECT_TRUE(manager.ComponentTypes(eInt).empty());
  EXPECT_EQ(nullptr, manager.Component<IntComponent>(eInt));

  EXPECT_TRUE(manager.RemoveComponent(eDouble, DoubleComponent::typeId));
  EXPECT_FALSE(manager.EntityHasComponentType(eDouble,
      DoubleComponent::typeId));
  EXPECT_TRUE(manager.ComponentTypes(eDouble).empty());
  EXPECT_EQ(nullptr, manager.Component<DoubleComponent>(eDouble));

  // Remove component by type
  EXPECT_TRUE(manager.RemoveComponent<IntComponent>(eIntDouble));
  EXPECT_FALSE(manager.EntityHasComponentType(eIntDouble,
      IntComponent::typeId));
  EXPECT_TRUE(manager.EntityHasComponentType(eIntDouble,
      DoubleComponent::typeId));
  types = manager.ComponentTypes(eIntDouble);
  EXPECT_EQ(1u, types.size());
  EXPECT_EQ(types.end(), types.find(IntComponent::typeId));
  EXPECT_EQ(nullptr, manager.Component<IntComponent>(eIntDouble));

  EXPECT_TRUE(manager.RemoveComponent<DoubleComponent>(eIntDouble));
  EXPECT_FALSE(manager.EntityHasComponentType(eIntDouble,
      IntComponent::typeId));
  EXPECT_FALSE(manager.EntityHasComponentType(eIntDouble,
      DoubleComponent::typeId));
  EXPECT_EQ(0u, manager.ComponentTypes(eIntDouble).size());
  EXPECT_EQ(nullptr, manager.Component<DoubleComponent>(eIntDouble));
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
  auto compPtr =  manager.CreateComponent<IntComponent>(entity,
      IntComponent(123));
  ASSERT_NE(nullptr, compPtr);

  EXPECT_TRUE(manager.HasComponentType(IntComponent::typeId));
  EXPECT_TRUE(manager.EntityHasComponentType(entity, IntComponent::typeId));
  EXPECT_EQ(compPtr, manager.Component<IntComponent>(entity));
  EXPECT_FALSE(manager.EntityHasComponentType(entity, DoubleComponent::typeId));
  EXPECT_FALSE(manager.EntityHasComponentType(entity2, IntComponent::typeId));

  // Try to add a component to an entity that does not exist
  EXPECT_FALSE(manager.HasEntity(kNullEntity));
  EXPECT_FALSE(manager.EntityHasComponentType(kNullEntity,
        IntComponent::typeId));
  EXPECT_EQ(nullptr, manager.CreateComponent<IntComponent>(kNullEntity,
        IntComponent(123)));
  EXPECT_FALSE(manager.HasEntity(kNullEntity));
  EXPECT_FALSE(manager.EntityHasComponentType(kNullEntity,
        IntComponent::typeId));

  // Query non-existing component, the default value is default-constructed
  BoolComponent *boolComp = manager.ComponentDefault<BoolComponent>(entity);
  ASSERT_NE(nullptr, boolComp);
  EXPECT_TRUE(manager.HasComponentType(BoolComponent::typeId));
  EXPECT_TRUE(manager.EntityHasComponentType(entity, BoolComponent::typeId));
  EXPECT_EQ(false, boolComp->Data());

  // Query non-existing component, the default value is used
  DoubleComponent *doubleComp =
    manager.ComponentDefault<DoubleComponent>(entity, 1.0);
  ASSERT_NE(nullptr, doubleComp);
  EXPECT_TRUE(manager.HasComponentType(DoubleComponent::typeId));
  EXPECT_TRUE(manager.EntityHasComponentType(entity, IntComponent::typeId));
  EXPECT_TRUE(manager.EntityHasComponentType(entity, DoubleComponent::typeId));
  EXPECT_FALSE(
    manager.EntityHasComponentType(entity2, DoubleComponent::typeId));
  EXPECT_FLOAT_EQ(1.0, doubleComp->Data());

  // Query existing component, the default value is not used
  IntComponent *intComp = manager.ComponentDefault<IntComponent>(entity, 124);
  ASSERT_NE(nullptr, intComp);
  EXPECT_TRUE(manager.HasComponentType(IntComponent::typeId));
  EXPECT_TRUE(manager.EntityHasComponentType(entity, IntComponent::typeId));
  EXPECT_EQ(123, intComp->Data());

  // Try to create/query a component from an entity that does not exist. nullptr
  // should be returned since a component cannot be attached to a non-existent
  // entity
  EXPECT_FALSE(manager.HasEntity(kNullEntity));
  EXPECT_EQ(nullptr, manager.CreateComponent<IntComponent>(kNullEntity,
        IntComponent(123)));
  EXPECT_EQ(nullptr, manager.ComponentDefault<IntComponent>(kNullEntity, 123));
  EXPECT_EQ(nullptr, manager.Component<IntComponent>(kNullEntity));
  EXPECT_FALSE(manager.ComponentData<IntComponent>(kNullEntity).has_value());
  EXPECT_EQ(ComponentState::NoChange, manager.ComponentState(kNullEntity,
        IntComponent::typeId));
  // (make sure the entity wasn't implicitly created during the invalid
  // component calls)
  EXPECT_FALSE(manager.HasEntity(kNullEntity));

  // Remove all entities
  manager.RequestRemoveEntities();
  EXPECT_EQ(3u, manager.EntityCount());
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();

  EXPECT_EQ(0u, manager.EntityCount());
  EXPECT_FALSE(manager.HasEntity(entity));
  EXPECT_FALSE(manager.HasEntity(entity2));
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
  Entity ePose = manager.CreateEntity();
  Entity eCustom = manager.CreateEntity();
  EXPECT_EQ(5u, manager.EntityCount());

  // Add components of different types to each entity
  auto comp1 = manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123));
  ASSERT_NE(nullptr, comp2);
  auto comp3 = manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456));
  ASSERT_NE(nullptr, comp3);
  auto comp4 = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));
  ASSERT_NE(nullptr, comp4);
  auto comp5 = manager.CreateComponent<components::Pose>(ePose,
      components::Pose({1, 2, 3, 0, 0, 0}));
  ASSERT_NE(nullptr, comp5);
  auto comp6 = manager.CreateComponent<CustomComponent>(eCustom,
      CustomComponent(Custom()));
  ASSERT_NE(nullptr, comp6);

  // Get and set component values
  {
    const auto *value = manager.Component<IntComponent>(eInt);
    ASSERT_NE(nullptr, value);
    EXPECT_EQ(123, value->Data());

    auto data = manager.ComponentData<IntComponent>(eInt);
    EXPECT_EQ(123, data);

    EXPECT_TRUE(manager.SetComponentData<IntComponent>(eInt, 456));
    data = manager.ComponentData<IntComponent>(eInt);
    EXPECT_EQ(456, data);

    EXPECT_FALSE(manager.SetComponentData<IntComponent>(eInt, 456));
  }

  {
    const auto *value = manager.Component<DoubleComponent>(eDouble);
    ASSERT_NE(nullptr, value);
    EXPECT_DOUBLE_EQ(0.123, value->Data());

    auto data = manager.ComponentData<DoubleComponent>(eDouble);
    EXPECT_EQ(0.123, data);

    EXPECT_TRUE(manager.SetComponentData<DoubleComponent>(eDouble, 0.456));
    data = manager.ComponentData<DoubleComponent>(eDouble);
    EXPECT_EQ(0.456, data);

    EXPECT_FALSE(manager.SetComponentData<DoubleComponent>(eDouble, 0.456));
  }

  {
    const auto *value = manager.Component<IntComponent>(eIntDouble);
    ASSERT_NE(nullptr, value);
    EXPECT_EQ(456, value->Data());

    auto data = manager.ComponentData<IntComponent>(eIntDouble);
    EXPECT_EQ(456, data);

    EXPECT_TRUE(manager.SetComponentData<IntComponent>(eIntDouble, 789));
    data = manager.ComponentData<IntComponent>(eIntDouble);
    EXPECT_EQ(789, data);

    EXPECT_FALSE(manager.SetComponentData<IntComponent>(eIntDouble, 789));
  }

  {
    const auto *value = manager.Component<DoubleComponent>(eIntDouble);
    ASSERT_NE(nullptr, value);
    EXPECT_DOUBLE_EQ(0.456, value->Data());

    auto data = manager.ComponentData<DoubleComponent>(eIntDouble);
    EXPECT_EQ(0.456, data);

    EXPECT_TRUE(manager.SetComponentData<DoubleComponent>(eIntDouble, 0.789));
    data = manager.ComponentData<DoubleComponent>(eIntDouble);
    EXPECT_EQ(0.789, data);

    EXPECT_FALSE(manager.SetComponentData<DoubleComponent>(eIntDouble, 0.789));
  }

  {
    const auto *value = manager.Component<components::Pose>(ePose);
    ASSERT_NE(nullptr, value);
    EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 0), value->Data());

    auto data = manager.ComponentData<components::Pose>(ePose);
    EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 0), data);

    EXPECT_TRUE(manager.SetComponentData<components::Pose>(ePose,
        {4, 5, 6, 0, 0, 0}));
    data = manager.ComponentData<components::Pose>(ePose);
    EXPECT_EQ(math::Pose3d(4, 5, 6, 0, 0, 0), data);

    EXPECT_FALSE(manager.SetComponentData<components::Pose>(ePose,
        {4, 5, 6, 0, 0, 0}));
  }

  {
    const auto *value = manager.Component<CustomComponent>(eCustom);
    ASSERT_NE(nullptr, value);
    EXPECT_EQ(123, value->Data().dummy);

    auto data = manager.ComponentData<CustomComponent>(eCustom);
    EXPECT_EQ(123, data->dummy);

    EXPECT_TRUE(manager.SetComponentData<CustomComponent>(eCustom, {456}));
    data = manager.ComponentData<CustomComponent>(eCustom);
    EXPECT_EQ(456, data->dummy);

    // No equality operator, always returns true
    EXPECT_TRUE(manager.SetComponentData<CustomComponent>(eCustom, {456}));
  }

  // Failure cases
  {
    const auto *value = manager.Component<IntComponent>(eDouble);
    ASSERT_EQ(nullptr, value);

    auto data = manager.ComponentData<IntComponent>(eDouble);
    EXPECT_EQ(std::nullopt, data);
  }

  {
    const auto *value = manager.Component<DoubleComponent>(eInt);
    ASSERT_EQ(nullptr, value);

    auto data = manager.ComponentData<DoubleComponent>(eInt);
    EXPECT_EQ(std::nullopt, data);
  }

  {
    const auto *value = manager.Component<IntComponent>(999);
    ASSERT_EQ(nullptr, value);

    auto data = manager.ComponentData<IntComponent>(999);
    EXPECT_EQ(std::nullopt, data);
  }

  {
    const auto *value = manager.Component<DoubleComponent>(999);
    ASSERT_EQ(nullptr, value);

    auto data = manager.ComponentData<DoubleComponent>(999);
    EXPECT_EQ(std::nullopt, data);
  }

  // Set new component type
  {
    const auto *value = manager.Component<IntComponent>(eDouble);
    EXPECT_EQ(nullptr, value);

    auto data = manager.ComponentData<IntComponent>(eDouble);
    EXPECT_EQ(std::nullopt, data);

    EXPECT_TRUE(manager.SetComponentData<IntComponent>(eDouble, 123));

    value = manager.Component<IntComponent>(eDouble);
    ASSERT_NE(nullptr, value);

    data = manager.ComponentData<IntComponent>(eDouble);
    EXPECT_EQ(123, data);
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
  auto comp1 = manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123));
  ASSERT_NE(nullptr, comp2);
  auto comp3 = manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456));
  ASSERT_NE(nullptr, comp3);
  auto comp4 = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));
  ASSERT_NE(nullptr, comp4);

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
  auto comp1 = manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123));
  ASSERT_NE(nullptr, comp2);
  auto comp3 = manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456));
  ASSERT_NE(nullptr, comp3);
  auto comp4 = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));
  ASSERT_NE(nullptr, comp4);

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

    auto comp5 = manager.CreateComponent<DoubleComponent>(eInt,
        DoubleComponent(12.123));
    ASSERT_NE(nullptr, comp5);
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
  auto comp1 = manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123));
  ASSERT_NE(nullptr, comp2);
  auto comp3 = manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456));
  ASSERT_NE(nullptr, comp3);
  auto compToRemove = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));
  ASSERT_NE(nullptr, compToRemove);

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
      EXPECT_TRUE(manager.RemoveComponent(eIntDouble, compToRemove->TypeId()));
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
  auto comp1 = manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123));
  ASSERT_NE(nullptr, comp2);
  auto comp3 = manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456));
  ASSERT_NE(nullptr, comp3);
  auto comp4 = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));
  ASSERT_NE(nullptr, comp4);

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
    ASSERT_NE(kNullEntity, newEntity);
    auto createdComp = manager.CreateComponent<IntComponent>(newEntity,
        IntComponent(789));
    ASSERT_NE(nullptr, createdComp);
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
  auto comp1 = manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123));
  ASSERT_NE(nullptr, comp2);
  auto comp3 = manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456));
  ASSERT_NE(nullptr, comp3);
  auto comp4 = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));
  ASSERT_NE(nullptr, comp4);

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
  auto comp1 = manager.CreateComponent<IntComponent>(eInt, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123));
  ASSERT_NE(nullptr, comp2);
  auto comp3 = manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456));
  ASSERT_NE(nullptr, comp3);
  auto comp4 = manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456));
  ASSERT_NE(nullptr, comp4);

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
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  ASSERT_NE(nullptr, comp2);

  EXPECT_EQ(2, newCount<IntComponent>(manager));
  EXPECT_TRUE(manager.HasNewEntities());

  // This would normally be done after each simulation step after systems are
  // updated
  manager.RunClearNewlyCreatedEntities();
  EXPECT_EQ(0, newCount<IntComponent>(manager));
  EXPECT_FALSE(manager.HasNewEntities());

  // Below tests adding a new entity, but not using the view until the following
  // simulation step. This is to ensure that the views update the status of
  // their new entities correctly at the end of each simulation step, regardless
  // of whether the view is used in a given simulation step or not

  // Create a new entity and add a component to it that makes this entity a
  // part of the IntComponent View
  Entity e3 = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());
  EXPECT_TRUE(manager.HasNewEntities());
  auto comp3 = manager.CreateComponent<IntComponent>(e3, IntComponent(789));
  EXPECT_NE(nullptr, comp3);
  // Mimic the end of a simulation step
  manager.RunClearNewlyCreatedEntities();
  // Use the IntComponent View, checking that the view has no entities marked as
  // new since we are now in a new simulation step
  EXPECT_EQ(0, newCount<IntComponent>(manager));
  EXPECT_FALSE(manager.HasNewEntities());
  EXPECT_EQ(3, eachCount<IntComponent>(manager));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewAfterRemoveComponent)
{
  // Create entities
  Entity e1 = manager.CreateEntity();
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<DoubleComponent>(e1,
      DoubleComponent(0.0));
  ASSERT_NE(nullptr, comp2);

  EXPECT_EQ(1, newCount<IntComponent>(manager));

  EXPECT_TRUE(manager.RemoveComponent(e1, comp1->TypeId()));
  EXPECT_EQ(1, newCount<DoubleComponent>(manager));

  manager.RunClearNewlyCreatedEntities();
  EXPECT_EQ(0, newCount<DoubleComponent>(manager));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachNewRemoveComponentFromRemoveEntity)
{
  // Create entities
  Entity e1 = manager.CreateEntity();
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  manager.RunClearNewlyCreatedEntities();
  // Nothing new after cleared
  EXPECT_EQ(0, newCount<IntComponent>(manager));

  Entity e2 = manager.CreateEntity();
  auto comp2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  ASSERT_NE(nullptr, comp2);
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
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  ASSERT_NE(nullptr, comp2);
  manager.RunClearNewlyCreatedEntities();
  // Nothing new after cleared
  EXPECT_EQ(0, newCount<IntComponent>(manager));

  // Create a new entity
  Entity e3 = manager.CreateEntity();
  auto comp3 = manager.CreateComponent<IntComponent>(e3, IntComponent(789));
  ASSERT_NE(nullptr, comp3);
  // Add a new component to existing entities
  auto comp4 = manager.CreateComponent<DoubleComponent>(e1,
      DoubleComponent(0.0));
  ASSERT_NE(nullptr, comp4);
  auto comp5 = manager.CreateComponent<DoubleComponent>(e2,
      DoubleComponent(2.0));
  ASSERT_NE(nullptr, comp5);

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
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  ASSERT_NE(nullptr, comp2);

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
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  ASSERT_NE(nullptr, comp2);
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

  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
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
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  manager.RunClearNewlyCreatedEntities();
  manager.RequestRemoveEntity(e1);

  // Add a new component to an removed entity. This should be possible since the
  // entity is only scheduled to be removed.
  auto comp2 = manager.CreateComponent<DoubleComponent>(e1,
      DoubleComponent(0.0));
  ASSERT_NE(nullptr, comp2);
  EXPECT_EQ(1, removedCount<IntComponent>(manager));
  EXPECT_EQ(1, (removedCount<IntComponent, DoubleComponent>(manager)));
}

////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, EachRemoveAllRemove)
{
  // Test when all entities are removed
  Entity e1 = manager.CreateEntity();
  Entity e2 = manager.CreateEntity();
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  ASSERT_NE(nullptr, comp2);
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
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  ASSERT_NE(nullptr, comp2);
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
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  ASSERT_NE(nullptr, comp2);
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
TEST_P(EntityComponentManagerFixture, EachAddRemoveComponent)
{
  // test calling ecm.Each on entities that have components added/removed
  // frequently. This is common with *Cmd components

  Entity e1 = manager.CreateEntity();
  EXPECT_EQ(1u, manager.EntityCount());
  EXPECT_EQ(0, eachCount<IntComponent>(manager));
  auto comp = manager.Component<IntComponent>(e1);
  EXPECT_EQ(nullptr, comp);

  // add a component
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, comp1);
  EXPECT_EQ(1, eachCount<IntComponent>(manager));
  comp = manager.Component<IntComponent>(e1);
  ASSERT_NE(nullptr, comp);
  EXPECT_EQ(123, comp->Data());
  EXPECT_EQ(123, comp1->Data());
  EXPECT_EQ(comp, comp1);

  // remove a component
  EXPECT_TRUE(manager.RemoveComponent(e1, IntComponent::typeId));
  EXPECT_EQ(nullptr, manager.Component<IntComponent>(e1));
  EXPECT_EQ(0, eachCount<IntComponent>(manager));

  // add the same type of component back in
  auto comp2 = manager.CreateComponent<IntComponent>(e1, IntComponent(456));
  ASSERT_NE(nullptr, comp2);
  EXPECT_EQ(1, eachCount<IntComponent>(manager));
  comp = manager.Component<IntComponent>(e1);
  ASSERT_NE(nullptr, comp);
  EXPECT_EQ(456, comp->Data());
  EXPECT_EQ(456, comp2->Data());
  EXPECT_EQ(comp, comp2);

  // remove the component again
  EXPECT_TRUE(manager.RemoveComponent(e1, IntComponent::typeId));
  EXPECT_EQ(nullptr, manager.Component<IntComponent>(e1));
  EXPECT_EQ(0, eachCount<IntComponent>(manager));

  // add and remove the component before calling ecm.Each. This is to make sure
  // that the views remove any entities in their toAddEntities queue if required
  // components for entities in a view's toAddEntities queue are removed before
  // calling Each, since a view's toAddEntities queue is processed (and then
  // cleared) in an Each call
  auto comp3 = manager.CreateComponent<IntComponent>(e1, IntComponent(789));
  ASSERT_NE(nullptr, comp3);
  comp = manager.Component<IntComponent>(e1);
  ASSERT_NE(nullptr, comp);
  EXPECT_EQ(789, comp->Data());
  EXPECT_EQ(789, comp3->Data());
  EXPECT_EQ(comp, comp3);
  EXPECT_TRUE(manager.RemoveComponent(e1, IntComponent::typeId));
  EXPECT_EQ(nullptr, manager.Component<IntComponent>(e1));
  // call ecm.Each after adding/removing the component
  EXPECT_EQ(0, eachCount<IntComponent>(manager));
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
  auto comp1 = manager.CreateComponent<IntComponent>(eInt, IntComponent(-123));
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<StringComponent>(eInt,
      StringComponent("int"));
  ASSERT_NE(nullptr, comp2);

  auto comp3 = manager.CreateComponent<UIntComponent>(eUint,
      UIntComponent(456u));
  ASSERT_NE(nullptr, comp3);
  auto comp4 = manager.CreateComponent<StringComponent>(eUint,
      StringComponent("uint"));
  ASSERT_NE(nullptr, comp4);

  auto comp5 = manager.CreateComponent<IntComponent>(eIntUint,
      IntComponent(789));
  ASSERT_NE(nullptr, comp5);
  auto comp6 = manager.CreateComponent<UIntComponent>(eIntUint,
      UIntComponent(789u));
  ASSERT_NE(nullptr, comp6);
  auto comp7 = manager.CreateComponent<StringComponent>(eIntUint,
      StringComponent("int-uint"));
  ASSERT_NE(nullptr, comp7);

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

  auto comp8 = manager.CreateComponent<IntComponent>(eInt2, IntComponent(-123));
  ASSERT_NE(nullptr, comp8);
  auto comp9 = manager.CreateComponent<StringComponent>(eInt2,
      StringComponent("int2"));
  ASSERT_NE(nullptr, comp9);

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
  auto comp1 = manager.CreateComponent<Even>(e2, {});
  ASSERT_NE(nullptr, comp1);
  auto comp2 = manager.CreateComponent<Even>(e4, {});
  ASSERT_NE(nullptr, comp2);
  auto comp3 = manager.CreateComponent<Even>(e6, {});
  ASSERT_NE(nullptr, comp3);

  auto comp4 = manager.CreateComponent<Odd>(e1, {});
  ASSERT_NE(nullptr, comp4);
  auto comp5 = manager.CreateComponent<Odd>(e3, {});
  ASSERT_NE(nullptr, comp5);
  auto comp6 = manager.CreateComponent<Odd>(e5, {});
  ASSERT_NE(nullptr, comp6);
  auto comp7 = manager.CreateComponent<Odd>(e7, {});
  ASSERT_NE(nullptr, comp7);

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
    auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(e1c0));
    ASSERT_NE(nullptr, comp1);
    auto comp2 = manager.CreateComponent<DoubleComponent>(e2,
        DoubleComponent(e2c0));
    ASSERT_NE(nullptr, comp2);
    auto comp3 = manager.CreateComponent<StringComponent>(e2,
        StringComponent(e2c1));
    ASSERT_NE(nullptr, comp3);
    auto comp4 = manager.CreateComponent<IntComponent>(e3, IntComponent(e3c0));
    ASSERT_NE(nullptr, comp4);
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
    EXPECT_EQ(4, changedStateMsg.entities_size());

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
TEST_P(EntityComponentManagerFixture, ChangedStateComponents)
{
  // Entity and component
  Entity e1{1};
  int e1c0{123};
  std::string e1c1{"string"};

  // Fill manager with entity
  EXPECT_EQ(e1, manager.CreateEntity());
  EXPECT_EQ(1u, manager.EntityCount());

  // Component
  auto comp1 = manager.CreateComponent<IntComponent>(e1, IntComponent(e1c0));
  ASSERT_NE(nullptr, comp1);

  // Serialize into a message
  msgs::SerializedStateMap stateMsg;
  manager.State(stateMsg);
  ASSERT_EQ(1, stateMsg.entities_size());

  // Mark entities/components as not new
  manager.RunClearNewlyCreatedEntities();
  auto changedStateMsg = manager.ChangedState();
  EXPECT_EQ(0, changedStateMsg.entities_size());

  // create component
  auto compPtr =
      manager.CreateComponent<StringComponent>(e1, StringComponent(e1c1));
  ASSERT_NE(nullptr, compPtr);
  changedStateMsg = manager.ChangedState();
  EXPECT_EQ(1, changedStateMsg.entities_size());
  manager.State(stateMsg);

  // Mark components as not new
  manager.RunSetAllComponentsUnchanged();
  changedStateMsg = manager.ChangedState();
  EXPECT_EQ(0, changedStateMsg.entities_size());

  // modify component
  auto iter = stateMsg.mutable_entities()->find(e1);
  ASSERT_TRUE(iter != stateMsg.mutable_entities()->end());

  msgs::SerializedEntityMap &e1Msg = iter->second;

  auto compIter = e1Msg.mutable_components()->find(compPtr->TypeId());
  ASSERT_TRUE(compIter != e1Msg.mutable_components()->end());

  msgs::SerializedComponent &e1c1Msg = compIter->second;
  EXPECT_EQ(e1c1, e1c1Msg.component());
  e1c1Msg.set_component("test");
  EXPECT_EQ("test", e1c1Msg.component());
  (*e1Msg.mutable_components())[e1c1Msg.type()] = e1c1Msg;

  manager.SetState(stateMsg);
  changedStateMsg = manager.ChangedState();
  EXPECT_EQ(1, changedStateMsg.entities_size());

  // Mark components as not new
  manager.RunSetAllComponentsUnchanged();
  changedStateMsg = manager.ChangedState();
  EXPECT_EQ(0, changedStateMsg.entities_size());

  // remove component
  EXPECT_TRUE(manager.RemoveComponent(e1, StringComponent::typeId));

  changedStateMsg = manager.ChangedState();
  EXPECT_EQ(1, changedStateMsg.entities_size());
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
  ASSERT_NE(nullptr, c1);
  auto c2 = manager.CreateComponent<IntComponent>(e2, IntComponent(456));
  ASSERT_NE(nullptr, c2);

  EXPECT_TRUE(manager.HasOneTimeComponentChanges());
  EXPECT_EQ(0u, manager.ComponentTypesWithPeriodicChanges().size());
  EXPECT_EQ(ComponentState::OneTimeChange,
      manager.ComponentState(e1, c1->TypeId()));
  EXPECT_EQ(ComponentState::OneTimeChange,
      manager.ComponentState(e2, c2->TypeId()));
  EXPECT_EQ(ComponentState::NoChange, manager.ComponentState(999, 888));
  EXPECT_EQ(ComponentState::NoChange, manager.ComponentState(e1, 888));

  // This would normally be done after each simulation step after systems are
  // updated
  manager.RunSetAllComponentsUnchanged();
  EXPECT_FALSE(manager.HasOneTimeComponentChanges());
  EXPECT_EQ(0u, manager.ComponentTypesWithPeriodicChanges().size());
  EXPECT_EQ(ComponentState::NoChange,
      manager.ComponentState(e1, c1->TypeId()));
  EXPECT_EQ(ComponentState::NoChange,
      manager.ComponentState(e2, c2->TypeId()));

  // Mark as changed
  manager.SetChanged(e1, c1->TypeId(), ComponentState::PeriodicChange);

  // check that only e1 c1 is serialized into a message
  msgs::SerializedStateMap stateMsg;
  manager.State(stateMsg);
  {
    ASSERT_EQ(1, stateMsg.entities_size());

    auto iter = stateMsg.entities().find(e1);
    const auto &e1Msg = iter->second;
    EXPECT_EQ(e1, e1Msg.id());
    ASSERT_EQ(1, e1Msg.components_size());

    auto compIter = e1Msg.components().begin();
    const auto &e1c1Msg = compIter->second;
    EXPECT_EQ(IntComponent::typeId, e1c1Msg.type());
    EXPECT_EQ(123, std::stoi(e1c1Msg.component()));
  }

  manager.SetChanged(e2, c2->TypeId(), ComponentState::OneTimeChange);

  EXPECT_TRUE(manager.HasOneTimeComponentChanges());
  // Expect a single component type to be marked as PeriodicChange
  ASSERT_EQ(1u, manager.ComponentTypesWithPeriodicChanges().size());
  EXPECT_EQ(IntComponent().TypeId(),
      *manager.ComponentTypesWithPeriodicChanges().begin());
  EXPECT_EQ(ComponentState::PeriodicChange,
      manager.ComponentState(e1, c1->TypeId()));
  EXPECT_EQ(ComponentState::OneTimeChange,
      manager.ComponentState(e2, c2->TypeId()));

  // Remove components
  EXPECT_TRUE(manager.RemoveComponent(e1, c1->TypeId()));

  EXPECT_TRUE(manager.HasOneTimeComponentChanges());
  EXPECT_EQ(0u, manager.ComponentTypesWithPeriodicChanges().size());
  EXPECT_EQ(ComponentState::NoChange,
      manager.ComponentState(e1, c1->TypeId()));

  EXPECT_TRUE(manager.RemoveComponent(e2, c2->TypeId()));

  EXPECT_FALSE(manager.HasOneTimeComponentChanges());
  EXPECT_EQ(ComponentState::NoChange,
      manager.ComponentState(e2, c2->TypeId()));
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, SetEntityCreateOffset)
{
  // First entity should have a value of 1.
  Entity entity = manager.CreateEntity();
  EXPECT_EQ(1u, entity);

  // Apply an offset.
  manager.SetEntityCreateOffset(1000);
  Entity entity2 = manager.CreateEntity();
  EXPECT_EQ(1001u, entity2);

  // Apply a lower offset, prints warning but goes through.
  manager.SetEntityCreateOffset(500);
  Entity entity3 = manager.CreateEntity();
  EXPECT_EQ(501u, entity3);
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, SerializedStateMapMsgAfterRemoveComponent)
{
  // Create entity
  Entity e1 = manager.CreateEntity();
  auto e1c0 =
    manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, e1c0);
  auto e1c1 =
    manager.CreateComponent<DoubleComponent>(e1, DoubleComponent(0.0));
  ASSERT_NE(nullptr, e1c1);
  auto e1c2 =
    manager.CreateComponent<StringComponent>(e1, StringComponent("int"));
  ASSERT_NE(nullptr, e1c2);

  // We use this map because the order in which components are iterated
  // through depends on the (undetermined) order of unordered multimaps
  std::map<ComponentTypeId, bool> expectations;
  expectations.insert(std::make_pair(e1c0->TypeId(), false));
  expectations.insert(std::make_pair(e1c1->TypeId(), true));
  expectations.insert(std::make_pair(e1c2->TypeId(), true));

  EXPECT_TRUE(manager.RemoveComponent(e1, e1c1->TypeId()));
  EXPECT_TRUE(manager.RemoveComponent(e1, e1c2->TypeId()));

  // Serialize into a message
  msgs::SerializedStateMap stateMsg;
  manager.State(stateMsg);

  // Check message
  {
    auto iter = stateMsg.entities().find(e1);
    const auto &e1Msg = iter->second;
    auto compIter = e1Msg.components().begin();

    // First component
    const auto &c0 = compIter->second;
    compIter++;
    EXPECT_EQ(c0.remove(), expectations.find(c0.type())->second);

    // Second component
    const auto &c1 = compIter->second;
    compIter++;
    EXPECT_EQ(c1.remove(), expectations.find(c1.type())->second);

    // Third component
    const auto &c2 = compIter->second;
    EXPECT_EQ(c2.remove(), expectations.find(c2.type())->second);
  }

  // Check that removed components don't exist anymore after clearing them
  manager.RunClearRemovedComponents();
  msgs::SerializedStateMap newStateMsg;
  manager.State(newStateMsg);

  // Check message
  {
    auto iter = newStateMsg.entities().find(e1);
    const auto &e1Msg = iter->second;
    EXPECT_EQ(1, e1Msg.components_size());
    auto compIter = e1Msg.components().begin();

    // First component
    const auto &e1c0Msg = compIter->second;
    EXPECT_FALSE(e1c0Msg.remove());
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, SerializedStateMsgAfterRemoveComponent)
{
  // Create entity
  Entity e1 = manager.CreateEntity();
  auto e1c0 =
    manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, e1c0);
  auto e1c1 =
    manager.CreateComponent<DoubleComponent>(e1, DoubleComponent(0.0));
  ASSERT_NE(nullptr, e1c1);
  auto e1c2 =
    manager.CreateComponent<StringComponent>(e1, StringComponent("int"));
  ASSERT_NE(nullptr, e1c2);

  // We use this map because the order in which components are iterated
  // through depends on the (undetermined) order of unordered multimaps
  std::map<ComponentTypeId, bool> expectations;
  expectations.insert(std::make_pair(e1c0->TypeId(), false));
  expectations.insert(std::make_pair(e1c1->TypeId(), true));
  expectations.insert(std::make_pair(e1c2->TypeId(), true));

  EXPECT_TRUE(manager.RemoveComponent(e1, e1c1->TypeId()));
  EXPECT_TRUE(manager.RemoveComponent(e1, e1c2->TypeId()));

  // Serialize into a message
  msgs::SerializedState stateMsg;
  stateMsg = manager.State();

  // Check message
  {
    auto const &entityMsg = stateMsg.entities(0);

    // First component
    const auto &c0 = entityMsg.components(0);
    EXPECT_EQ(c0.remove(), expectations.find(c0.type())->second);

    // Second component
    const auto &c1 = entityMsg.components(1);
    EXPECT_EQ(c1.remove(), expectations.find(c1.type())->second);

    // Third component
    const auto &c2 = entityMsg.components(2);
    EXPECT_EQ(c2.remove(), expectations.find(c2.type())->second);
  }

  // Check that removed components don't exist anymore after clearing them
  manager.RunClearRemovedComponents();
  msgs::SerializedState newStateMsg;
  newStateMsg = manager.State();

  // Check message
  {
    auto const &entityMsg = newStateMsg.entities(0);
    EXPECT_EQ(1, entityMsg.components_size());

    // First component
    const auto &e1c0Msg = entityMsg.components(0);
    EXPECT_FALSE(e1c0Msg.remove());
  }
}

//////////////////////////////////////////////////
// Verify SerializedStateMap message with no changed components,
// but some removed components
TEST_P(EntityComponentManagerFixture, SerializedStateMapMsgCompsRemovedOnly)
{
  // Create entity
  Entity e1 = manager.CreateEntity();
  auto e1c0 =
    manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, e1c0);
  auto e1c1 = manager.CreateComponent<DoubleComponent>(e1,
      DoubleComponent(0.0));
  ASSERT_NE(nullptr, e1c1);
  auto e1c2 =
    manager.CreateComponent<StringComponent>(e1, StringComponent("int"));
  ASSERT_NE(nullptr, e1c2);

  manager.RunSetAllComponentsUnchanged();
  EXPECT_TRUE(manager.RemoveComponent(e1, e1c0->TypeId()));
  EXPECT_TRUE(manager.RemoveComponent(e1, e1c2->TypeId()));
  // Serialize into a message
  msgs::SerializedStateMap stateMsg;
  manager.State(stateMsg);

  // Check message
  {
    auto iter = stateMsg.entities().find(e1);
    const auto &e1Msg = iter->second;
    auto compIter = e1Msg.components().begin();

    // Check number of components
    ASSERT_EQ(e1Msg.components().size(), 2u);

    // First component
    const auto &c0 = compIter->second;
    compIter++;
    ASSERT_EQ(c0.remove(), true);

    // Second component
    const auto &c2 = compIter->second;
    ASSERT_EQ(c2.remove(), true);
  }
}

//////////////////////////////////////////////////
// Verify that removed components are correctly filtered when creating a
// SerializedStateMap message
TEST_P(EntityComponentManagerFixture, SetRemovedComponentsMsgTypesFilter)
{
  // Create entity
  Entity e1 = manager.CreateEntity();
  auto e1c0 =
    manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, e1c0);
  auto e1c1 =
    manager.CreateComponent<DoubleComponent>(e1, DoubleComponent(0.0));
  ASSERT_NE(nullptr, e1c1);
  auto e1c2 =
    manager.CreateComponent<StringComponent>(e1, StringComponent("foo"));
  ASSERT_NE(nullptr, e1c2);

  manager.RunSetAllComponentsUnchanged();
  EXPECT_TRUE(manager.RemoveComponent(e1, e1c0->TypeId()));
  EXPECT_TRUE(manager.RemoveComponent(e1, e1c2->TypeId()));

  // Serialize into a message, providing a list of types to be included
  msgs::SerializedStateMap stateMsg;
  std::unordered_set<Entity> entitySet{e1};
  std::unordered_set<ComponentTypeId> types{e1c0->TypeId(), e1c1->TypeId()};
  manager.State(stateMsg, entitySet, types, false);

  // Check message
  {
    auto iter = stateMsg.entities().find(e1);
    const auto &e1Msg = iter->second;
    auto compIter = e1Msg.components().begin();

    // Check number of components
    ASSERT_EQ(e1Msg.components().size(), 1u);

    // Only component in message should be e1c2
    const auto &c0 = compIter->second;
    EXPECT_EQ(c0.remove(), true);
    EXPECT_EQ(c0.type(), e1c0->TypeId());
  }
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, RemovedComponentsSyncBetweenServerAndGUI)
{
  // Simulate the GUI's ECM
  EntityCompMgrTest guiManager;

  // Create entity
  Entity e1 = manager.CreateEntity();
  auto e1c0 =
    manager.CreateComponent<IntComponent>(e1, IntComponent(123));
  ASSERT_NE(nullptr, e1c0);
  auto e1c1 =
    manager.CreateComponent<DoubleComponent>(e1, DoubleComponent(0.0));
  ASSERT_NE(nullptr, e1c1);
  auto e1c2 =
    manager.CreateComponent<StringComponent>(e1, StringComponent("int"));
  ASSERT_NE(nullptr, e1c2);

  // We use this map because the order in which components are iterated
  // through depends on the (undetermined) order of unordered multimaps
  std::map<ComponentTypeId, bool> expectationsBeforeRemoving;
  expectationsBeforeRemoving.insert(std::make_pair(e1c0->TypeId(), false));
  expectationsBeforeRemoving.insert(std::make_pair(e1c1->TypeId(), false));
  expectationsBeforeRemoving.insert(std::make_pair(e1c2->TypeId(), false));

  // Serialize server ECM into a message
  msgs::SerializedStateMap stateMsg;
  manager.State(stateMsg);

  // Set GUI's ECM and serialize into a message
  guiManager.SetState(stateMsg);
  msgs::SerializedStateMap guiStateMsg;
  guiManager.State(guiStateMsg);

  // Check sync message
  {
    auto iter = guiStateMsg.entities().find(e1);
    const auto &e1Msg = iter->second;
    auto compIter = e1Msg.components().begin();

    // First component
    const auto &c0 = compIter->second;
    compIter++;
    EXPECT_EQ(c0.remove(), expectationsBeforeRemoving.find(c0.type())->second);

    // Second component
    const auto &c1 = compIter->second;
    compIter++;
    EXPECT_EQ(c1.remove(), expectationsBeforeRemoving.find(c1.type())->second);

    // Third component
    const auto &c2 = compIter->second;
    EXPECT_EQ(c2.remove(), expectationsBeforeRemoving.find(c2.type())->second);
  }

  std::map<ComponentTypeId, bool> expectationsAfterRemoving;
  expectationsAfterRemoving.insert(std::make_pair(e1c0->TypeId(), false));
  expectationsAfterRemoving.insert(std::make_pair(e1c1->TypeId(), true));
  expectationsAfterRemoving.insert(std::make_pair(e1c2->TypeId(), true));

  // Remove components and synchronize again
  EXPECT_TRUE(manager.RemoveComponent(e1, e1c1->TypeId()));
  EXPECT_TRUE(manager.RemoveComponent(e1, e1c2->TypeId()));

  msgs::SerializedStateMap newStateMsg;
  manager.State(newStateMsg);

  EXPECT_TRUE(nullptr != guiManager.Component<IntComponent>(e1));
  EXPECT_TRUE(nullptr != guiManager.Component<DoubleComponent>(e1));
  EXPECT_TRUE(nullptr != guiManager.Component<StringComponent>(e1));
  guiManager.SetState(newStateMsg);
  EXPECT_TRUE(nullptr != guiManager.Component<IntComponent>(e1));
  EXPECT_TRUE(nullptr == guiManager.Component<DoubleComponent>(e1));
  EXPECT_TRUE(nullptr == guiManager.Component<StringComponent>(e1));

  msgs::SerializedStateMap newGuiStateMsg;
  guiManager.State(newGuiStateMsg);

  // Check message
  {
    auto iter = newGuiStateMsg.entities().find(e1);
    const auto &e1Msg = iter->second;
    auto compIter = e1Msg.components().begin();

    // First component
    const auto &c0 = compIter->second;
    compIter++;
    EXPECT_EQ(c0.remove(), expectationsAfterRemoving.find(c0.type())->second);

    // Second component
    const auto &c1 = compIter->second;
    compIter++;
    EXPECT_EQ(c1.remove(), expectationsAfterRemoving.find(c1.type())->second);

    // Third component
    const auto &c2 = compIter->second;
    EXPECT_EQ(c2.remove(), expectationsAfterRemoving.find(c2.type())->second);
  }
}

/// \brief Helper function for comparing the same type of component across two
/// different entities
/// \param[in] _ecm The entity component manager
/// \param[in] _entity1 The first entity
/// \param[in] _entity2 The second entity
/// \param[in] _equal Whether the component's data between _entity1 and
/// _entity2 should be equal (true) or not (false)
/// \tparam ComponentTypeT Component type
template<typename ComponentTypeT>
static void CompareEntityComponents(const EntityComponentManager &_ecm,
    const Entity _entity1, const Entity _entity2, bool _equal)
{
  auto comp1 = _ecm.Component<ComponentTypeT>(_entity1);
  ASSERT_NE(nullptr, comp1);

  auto comp2 = _ecm.Component<ComponentTypeT>(_entity2);
  ASSERT_NE(nullptr, comp2);

  if (_equal)
    EXPECT_EQ(*comp1, *comp2);
  else
    EXPECT_NE(*comp1, *comp2);
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, CloneEntities)
{
  // testing entity cloning with the following entity structure:
  // - topLevelEntity
  //    - childEntity1
  //       - grandChildEntity1 (canonical link for childEntity1)
  //    - childEntity2 (canonical link for topLevelEntity)

  const auto allowRename = true;
  const auto noAllowRename = false;

  Entity topLevelEntity = manager.CreateEntity();
  manager.CreateComponent(topLevelEntity, components::Name("topLevelEntity"));
  manager.CreateComponent(topLevelEntity, IntComponent(123));
  manager.CreateComponent(topLevelEntity, StringComponent("string0"));

  Entity childEntity1 = manager.CreateEntity();
  manager.CreateComponent(childEntity1, components::Name("childEntity1"));
  manager.CreateComponent(childEntity1,
      components::ParentEntity(topLevelEntity));
  manager.CreateComponent(childEntity1, IntComponent(456));
  manager.CreateComponent(childEntity1, StringComponent("string1"));

  Entity grandChildEntity1 = manager.CreateEntity();
  manager.CreateComponent(grandChildEntity1,
      components::Name("grandChildEntity1"));
  manager.CreateComponent(grandChildEntity1,
      components::ParentEntity(childEntity1));

  Entity childEntity2 = manager.CreateEntity();
  manager.CreateComponent(childEntity2, components::Name("childEntity2"));
  manager.CreateComponent(childEntity2,
      components::ParentEntity(topLevelEntity));
  manager.CreateComponent(childEntity2, IntComponent(789));
  manager.CreateComponent(childEntity2, StringComponent("string2"));

  manager.CreateComponent(topLevelEntity,
      components::ModelCanonicalLink(childEntity2));
  manager.CreateComponent(childEntity1,
      components::ModelCanonicalLink(grandChildEntity1));
  manager.CreateComponent(childEntity2, components::CanonicalLink());
  manager.CreateComponent(grandChildEntity1, components::CanonicalLink());

  EXPECT_EQ(4u, manager.EntityCount());

  std::unordered_set<Entity> clonedEntities;

  auto validateTopLevelClone =
    [&](const Entity _clonedEntity)
    {
      EXPECT_NE(kNullEntity, _clonedEntity);
      EXPECT_NE(_clonedEntity, topLevelEntity);
      EXPECT_EQ(manager.ComponentTypes(_clonedEntity),
          manager.ComponentTypes(topLevelEntity));
      EXPECT_FALSE(manager.EntityHasComponentType(_clonedEntity,
            components::ParentEntity::typeId));
      CompareEntityComponents<components::Name>(manager, topLevelEntity,
          _clonedEntity, false);
      CompareEntityComponents<IntComponent>(manager, topLevelEntity,
          _clonedEntity, true);
      CompareEntityComponents<StringComponent>(manager, topLevelEntity,
          _clonedEntity, true);
      CompareEntityComponents<components::ModelCanonicalLink>(manager,
          topLevelEntity, _clonedEntity, false);
    };

  // clone the topLevelEntity
  auto clonedTopLevelEntity =
    manager.Clone(topLevelEntity, kNullEntity, "", allowRename);
  EXPECT_EQ(8u, manager.EntityCount());
  clonedEntities.insert(clonedTopLevelEntity);
  validateTopLevelClone(clonedTopLevelEntity);

  auto validateChildClone =
    [&](const Entity _clonedChild, const Entity _originalChild)
    {
      EXPECT_NE(kNullEntity, _clonedChild);
      EXPECT_NE(_clonedChild, _originalChild);
      EXPECT_EQ(manager.ComponentTypes(_clonedChild),
          manager.ComponentTypes(_originalChild));
      auto parentComp =
        manager.Component<components::ParentEntity>(_clonedChild);
      ASSERT_NE(nullptr, parentComp);
      EXPECT_EQ(clonedTopLevelEntity, parentComp->Data());
      CompareEntityComponents<components::Name>(manager, _clonedChild,
          _originalChild, false);
      CompareEntityComponents<IntComponent>(manager, _clonedChild,
          _originalChild, true);
      CompareEntityComponents<StringComponent>(manager, _clonedChild,
          _originalChild, true);
    };

  auto validateGrandChildClone =
    [&](const Entity _clonedEntity, bool _sameParent)
    {
      EXPECT_NE(kNullEntity, _clonedEntity);
      EXPECT_EQ(manager.ComponentTypes(_clonedEntity),
          manager.ComponentTypes(grandChildEntity1));
      CompareEntityComponents<components::Name>(manager, _clonedEntity,
          grandChildEntity1, false);
      CompareEntityComponents<components::ParentEntity>(manager,
          _clonedEntity, grandChildEntity1, _sameParent);
      EXPECT_TRUE(manager.EntitiesByComponents(
            components::ParentEntity(_clonedEntity)).empty());
    };

  // Verify that all child entities were properly cloned
  auto clonedChildEntities = manager.EntitiesByComponents(
      components::ParentEntity(clonedTopLevelEntity));
  EXPECT_EQ(2u, clonedChildEntities.size());
  for (const auto &child : clonedChildEntities)
  {
    clonedEntities.insert(child);

    auto clonedGrandChildren = manager.EntitiesByComponents(
        components::ParentEntity(child));

    auto comparedToOriginalChild = false;
    auto intComp = manager.Component<IntComponent>(child);
    ASSERT_NE(nullptr, intComp);
    if (intComp->Data() == 456)
    {
      validateChildClone(child, childEntity1);
      CompareEntityComponents<components::ModelCanonicalLink>(manager, child,
          childEntity1, false);
      comparedToOriginalChild = true;

      ASSERT_EQ(1u, clonedGrandChildren.size());
      clonedEntities.insert(clonedGrandChildren[0]);
      validateGrandChildClone(clonedGrandChildren[0], false);
      auto parentComp =
        manager.Component<components::ParentEntity>(clonedGrandChildren[0]);
      ASSERT_NE(nullptr, parentComp);
      EXPECT_EQ(child, parentComp->Data());
      EXPECT_NE(nullptr, manager.Component<components::CanonicalLink>(
            clonedGrandChildren[0]));
    }
    else if (intComp->Data() == 789)
    {
      validateChildClone(child, childEntity2);
      EXPECT_NE(nullptr, manager.Component<components::CanonicalLink>(child));
      comparedToOriginalChild = true;

      EXPECT_TRUE(clonedGrandChildren.empty());
    }

    EXPECT_TRUE(comparedToOriginalChild);
  }

  // clone a child entity
  auto grandChildParentComp =
    manager.Component<components::ParentEntity>(grandChildEntity1);
  ASSERT_NE(nullptr, grandChildParentComp);
  auto clonedGrandChildEntity = manager.Clone(grandChildEntity1,
      grandChildParentComp->Data(), "", allowRename);
  EXPECT_EQ(9u, manager.EntityCount());
  clonedEntities.insert(clonedGrandChildEntity);
  validateGrandChildClone(clonedGrandChildEntity, true);

  // Try cloning an entity with a name that already exists, but allow renaming.
  // This should succeed and generate a cloned entity with a unique name.
  const auto existingName = "grandChildEntity1";
  EXPECT_NE(kNullEntity,
      manager.EntityByComponents(components::Name(existingName)));
  auto renamedClonedEntity = manager.Clone(grandChildEntity1,
      grandChildParentComp->Data(), existingName, allowRename);
  EXPECT_EQ(10u, manager.EntityCount());
  clonedEntities.insert(clonedGrandChildEntity);
  validateGrandChildClone(renamedClonedEntity, true);

  // Try cloning an entity with a name that already exists, without allowing
  // renaming. This should fail since entities should have unique names.
  auto failedClonedEntity = manager.Clone(grandChildEntity1,
      grandChildParentComp->Data(), existingName, noAllowRename);
  EXPECT_EQ(10u, manager.EntityCount());
  EXPECT_EQ(kNullEntity, failedClonedEntity);

  // make sure that the name given to each cloned entity is unique
  EXPECT_EQ(5u, clonedEntities.size());
  for (const auto &entity : clonedEntities)
  {
    auto nameComp = manager.Component<components::Name>(entity);
    ASSERT_NE(nullptr, nameComp);
    EXPECT_EQ(1u, manager.EntitiesByComponents(*nameComp).size());
  }

  // try to clone an entity that does not exist
  EXPECT_EQ(kNullEntity, manager.Clone(kNullEntity, topLevelEntity, "",
        allowRename));
  EXPECT_EQ(10u, manager.EntityCount());
}

/////////////////////////////////////////////////
// Check that some widely used deprecated APIs still work
TEST_P(EntityComponentManagerFixture, Deprecated)
{
  IGN_UTILS_WARN_IGNORE__DEPRECATED_DECLARATION

  // Fail to create component for inexistent entity
  EXPECT_EQ(nullptr, manager.CreateComponent<IntComponent>(789,
      IntComponent(123)));

  // Create some entities
  auto eInt = manager.CreateEntity();
  auto eDouble = manager.CreateEntity();
  auto eIntDouble = manager.CreateEntity();
  EXPECT_EQ(3u, manager.EntityCount());

  // Add components and keep their unique ComponentKeys
  EXPECT_NE(nullptr, manager.CreateComponent<IntComponent>(eInt,
      IntComponent(123)));
  ComponentKey cIntEInt = {IntComponent::typeId, eInt};

  EXPECT_NE(nullptr, manager.CreateComponent<DoubleComponent>(eDouble,
      DoubleComponent(0.123)));
  ComponentKey cDoubleEDouble = {DoubleComponent::typeId, eDouble};

  EXPECT_NE(nullptr, manager.CreateComponent<IntComponent>(eIntDouble,
      IntComponent(456)));
  ComponentKey cIntEIntDouble = {IntComponent::typeId, eIntDouble};

  EXPECT_NE(nullptr, manager.CreateComponent<DoubleComponent>(eIntDouble,
      DoubleComponent(0.456)));
  ComponentKey cDoubleEIntDouble = {DoubleComponent::typeId, eIntDouble};

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

  IGN_UTILS_WARN_RESUME__DEPRECATED_DECLARATION
}

//////////////////////////////////////////////////
TEST_P(EntityComponentManagerFixture, PinnedEntity)
{
  // Create some entities
  auto e1 = manager.CreateEntity();
  EXPECT_EQ(1u, e1);
  EXPECT_TRUE(manager.HasEntity(e1));

  auto e2 = manager.CreateEntity();
  EXPECT_TRUE(manager.SetParentEntity(e2, e1));
  EXPECT_EQ(2u, e2);
  EXPECT_TRUE(manager.HasEntity(e2));

  auto e3 = manager.CreateEntity();
  EXPECT_EQ(3u, e3);
  EXPECT_TRUE(manager.HasEntity(e3));

  EXPECT_EQ(3u, manager.EntityCount());

  // Mark e1 as unremovable, which should also lock its child entity e2
  manager.PinEntity(e1);

  // Try to remove e1, which is locked entity
  manager.RequestRemoveEntity(e1);
  EXPECT_EQ(3u, manager.EntityCount());
  EXPECT_FALSE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(3u, manager.EntityCount());

  // Try to remove e2, which has been locked recursively
  manager.RequestRemoveEntity(e2);
  EXPECT_EQ(3u, manager.EntityCount());
  EXPECT_FALSE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(3u, manager.EntityCount());

  // Try to remove all entities, which should leave just e1 and e2
  manager.RequestRemoveEntities();
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(2u, manager.EntityCount());

  // Unmark e2, and now it should be removable.
  manager.UnpinEntity(e2);
  manager.RequestRemoveEntity(e2);
  EXPECT_EQ(2u, manager.EntityCount());
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(1u, manager.EntityCount());

  // Unmark all entities, and now it should be removable.
  manager.UnpinAllEntities();
  manager.RequestRemoveEntities();
  EXPECT_TRUE(manager.HasEntitiesMarkedForRemoval());
  manager.ProcessEntityRemovals();
  EXPECT_EQ(0u, manager.EntityCount());
}

// Run multiple times. We want to make sure that static globals don't cause
// problems.
INSTANTIATE_TEST_SUITE_P(EntityComponentManagerRepeat,
    EntityComponentManagerFixture, ::testing::Range(1, 10));
