/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <memory>
#include <utility>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentStorage.hh"
#include "ignition/gazebo/Types.hh"
#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/Pose.hh"

using namespace ignition;
using namespace gazebo;

// define components that are used across various test cases
const auto poseComp = components::Pose({0, 1, 2, 3, 4, 5});
const auto poseComp2 = components::Pose({1, 1, 1, 2, 2, 2});
const auto linVelComp = components::LinearVelocity({0, 1, 2});
const auto linVelComp2 = components::LinearVelocity({5, 5, 5});

// define entities that are used across various test cases
const Entity entity3 = 3;
const Entity entity4 = 4;

/////////////////////////////////////////////////
class EntityComponentStorageTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    EXPECT_TRUE(this->storage.AddEntity(this->e1));
    EXPECT_TRUE(this->storage.AddEntity(this->e2));
  }

  /// \brief Helper function for adding a component to the storage
  /// \param[in] _entity The entity that owns _component
  /// \param[in] _typeId The type ID of _component
  /// \param[in] _component The component, which belongs to _entity
  /// \return The result of the component addition
  public: ComponentAdditionResult AddComponent(const Entity _entity,
              const ComponentTypeId _typeId,
              const components::BaseComponent *_component)
  {
    auto compPtr = components::Factory::Instance()->New(
        _typeId, _component);
    return this->storage.AddComponent(_entity, std::move(compPtr));
  }

  /// \brief Helper function that uses EntityComponentStorage::ValidComponent to
  /// get a component of a particular type that belongs to an entity.
  /// \param[in] _entity The entity
  /// \return A pointer to the component of the templated type. If no such
  /// component exists, nullptr is returned
  public: template<typename ComponentTypeT>
          ComponentTypeT *Component(const Entity _entity)
  {
    auto baseComp = this->storage.ValidComponent(_entity,
        ComponentTypeT::typeId);
    return static_cast<ComponentTypeT *>(baseComp);
  }

  /// \brief Const version of the Component helper function
  public: template<typename ComponentTypeT>
          const ComponentTypeT *ConstComponent(const Entity _entity) const
  {
    auto baseComp = this->storage.ValidComponent(_entity,
        ComponentTypeT::typeId);
    return static_cast<const ComponentTypeT *>(baseComp);
  }

  public: EntityComponentStorage storage;

  public: const Entity e1{1};

  public: const Entity e2{2};
};

/////////////////////////////////////////////////
TEST_F(EntityComponentStorageTest, AddEntity)
{
  // Entities were already added to the storage in the test fixture's SetUp
  // method. So, try to add entities that are already in the storage.
  // This should fail
  EXPECT_FALSE(this->storage.AddEntity(this->e1));
  EXPECT_FALSE(this->storage.AddEntity(this->e2));

  // Add entities that are not in the storage yet
  EXPECT_TRUE(this->storage.AddEntity(5));
  EXPECT_TRUE(this->storage.AddEntity(6));
}

/////////////////////////////////////////////////
TEST_F(EntityComponentStorageTest, RemoveEntity)
{
  // Try to remove entities that aren't in the storage
  EXPECT_FALSE(this->storage.RemoveEntity(3));
  EXPECT_FALSE(this->storage.RemoveEntity(4));

  // Remove entities that are in the storage
  EXPECT_TRUE(this->storage.RemoveEntity(this->e1));
  EXPECT_TRUE(this->storage.RemoveEntity(this->e2));

  // Try to remove entities that have already been removed from the storage
  EXPECT_FALSE(this->storage.RemoveEntity(this->e1));
  EXPECT_FALSE(this->storage.RemoveEntity(this->e2));
}

/////////////////////////////////////////////////
TEST_F(EntityComponentStorageTest, AddComponent)
{
  // Add components to entities in the storage
  EXPECT_EQ(ComponentAdditionResult::NEW_ADDITION,
      this->AddComponent(this->e1, components::Pose::typeId, &poseComp));
  EXPECT_EQ(ComponentAdditionResult::NEW_ADDITION,
      this->AddComponent(this->e2, components::LinearVelocity::typeId,
        &linVelComp));

  // Make sure the added components have the expected data
  auto storagePoseComp = this->Component<components::Pose>(this->e1);
  ASSERT_NE(nullptr, storagePoseComp);
  EXPECT_EQ(poseComp.Data(), storagePoseComp->Data());
  auto storageLinVelComp = this->Component<components::LinearVelocity>(
      this->e2);
  ASSERT_NE(nullptr, storageLinVelComp);
  EXPECT_EQ(linVelComp.Data(), storageLinVelComp->Data());

  // Try to add components to entities that aren't in the storage
  // (to make sure these entities aren't in the storage, we can try to remove
  // them from the storage. Since these entities are not in the storage, the
  // remove call should fail)
  EXPECT_FALSE(this->storage.RemoveEntity(entity3));
  EXPECT_FALSE(this->storage.RemoveEntity(entity4));
  EXPECT_EQ(ComponentAdditionResult::FAILED_ADDITION,
      this->AddComponent(entity3, components::Pose::typeId, &poseComp));
  EXPECT_EQ(ComponentAdditionResult::FAILED_ADDITION,
      this->AddComponent(entity4, components::LinearVelocity::typeId,
        &linVelComp));
  EXPECT_EQ(nullptr, this->Component<components::Pose>(entity3));
  EXPECT_EQ(nullptr, this->Component<components::LinearVelocity>(entity4));

  // Add components to entities that already have a component of the same type
  // (this has the same effect of modifying an existing component)
  EXPECT_EQ(ComponentAdditionResult::MODIFICATION,
      this->AddComponent(this->e1, components::Pose::typeId, &poseComp2));
  EXPECT_EQ(ComponentAdditionResult::MODIFICATION,
      this->AddComponent(this->e2, components::LinearVelocity::typeId,
        &linVelComp2));

  // We can't check if the modification actually took place since this requires
  // functionality beyond the EntityComponentStorage API (see the comments in
  // the EntityComponentStorage::AddComponent method definition for more
  // details), but we can at least check that the components still exist after
  // modification
  ASSERT_NE(nullptr, this->Component<components::Pose>(this->e1));
  ASSERT_NE(nullptr, this->Component<components::LinearVelocity>(this->e2));

  // Remove components of a particular type and then re-add them
  // (pose component)
  EXPECT_TRUE(this->storage.RemoveComponent(this->e1,
        components::Pose::typeId));
  EXPECT_EQ(nullptr, this->Component<components::Pose>(this->e1));
  EXPECT_EQ(ComponentAdditionResult::RE_ADDITION,
      this->AddComponent(this->e1, components::Pose::typeId, &poseComp));
  storagePoseComp = this->Component<components::Pose>(this->e1);
  ASSERT_NE(nullptr, storagePoseComp);
  EXPECT_EQ(poseComp.Data(), storagePoseComp->Data());
  // (linear velocity component)
  EXPECT_TRUE(this->storage.RemoveComponent(this->e2,
        components::LinearVelocity::typeId));
  EXPECT_EQ(nullptr, this->Component<components::LinearVelocity>(this->e2));
  EXPECT_EQ(ComponentAdditionResult::RE_ADDITION,
      this->AddComponent(this->e2, components::LinearVelocity::typeId,
        &linVelComp));
  storageLinVelComp = this->Component<components::LinearVelocity>(this->e2);
  ASSERT_NE(nullptr, storageLinVelComp);
  EXPECT_EQ(linVelComp.Data(), storageLinVelComp->Data());
}

/////////////////////////////////////////////////
TEST_F(EntityComponentStorageTest, RemoveComponent)
{
  // Add components to entities
  EXPECT_EQ(ComponentAdditionResult::NEW_ADDITION,
      this->AddComponent(this->e1, poseComp.TypeId(), &poseComp));
  EXPECT_NE(nullptr, this->Component<components::Pose>(this->e1));
  EXPECT_EQ(ComponentAdditionResult::NEW_ADDITION,
      this->AddComponent(this->e2, linVelComp.TypeId(), &linVelComp));
  EXPECT_NE(nullptr, this->Component<components::LinearVelocity>(this->e2));

  // Remove components that exist
  EXPECT_TRUE(this->storage.RemoveComponent(this->e1,
        components::Pose::typeId));
  EXPECT_EQ(nullptr, this->Component<components::Pose>(this->e1));
  EXPECT_TRUE(this->storage.RemoveComponent(this->e2,
        components::LinearVelocity::typeId));
  EXPECT_EQ(nullptr, this->Component<components::LinearVelocity>(this->e2));

  // Try to remove components that don't exist
  EXPECT_FALSE(this->storage.RemoveComponent(this->e1,
        components::Pose::typeId));
  EXPECT_EQ(nullptr, this->Component<components::Pose>(this->e1));
  EXPECT_FALSE(this->storage.RemoveComponent(this->e2,
        components::LinearVelocity::typeId));
  EXPECT_EQ(nullptr, this->Component<components::LinearVelocity>(this->e2));

  // Try to remove components from entities that don't exist
  EXPECT_FALSE(this->storage.RemoveComponent(entity3,
        components::Pose::typeId));
  EXPECT_EQ(nullptr, this->Component<components::Pose>(entity3));
  EXPECT_FALSE(this->storage.RemoveComponent(entity4,
        components::LinearVelocity::typeId));
  EXPECT_EQ(nullptr, this->Component<components::LinearVelocity>(entity4));
}

/////////////////////////////////////////////////
TEST_F(EntityComponentStorageTest, ValidComponent)
{
  // Attach a component to an entity
  EXPECT_EQ(ComponentAdditionResult::NEW_ADDITION,
      this->AddComponent(this->e1, poseComp.TypeId(), &poseComp));

  // Get a component that is attached to an entity
  auto derivedComp = this->Component<components::Pose>(this->e1);
  ASSERT_NE(nullptr, derivedComp);
  EXPECT_EQ(poseComp.Data(), derivedComp->Data());

  // Test modifying the component that was retrieved
  derivedComp->Data() = poseComp2.Data();
  derivedComp = this->Component<components::Pose>(this->e1);
  ASSERT_NE(nullptr, derivedComp);
  EXPECT_EQ(poseComp2.Data(), derivedComp->Data());

  // Try to get a component that does not exist
  ASSERT_EQ(nullptr, this->Component<components::LinearVelocity>(this->e2));

  // Remove a component and try to retrieve it
  EXPECT_NE(nullptr, this->Component<components::Pose>(this->e1));
  EXPECT_TRUE(this->storage.RemoveComponent(this->e1,
        components::Pose::typeId));
  ASSERT_EQ(nullptr, this->Component<components::Pose>(this->e1));
}

/////////////////////////////////////////////////
// Similar to the ValidComponent test, but this test covers the const version of
// the EntityComponentStorage::ValidComponent method (the ValidComponent test
// covered the non-const version of the EntityComponentStorage::ValidComponent)
TEST_F(EntityComponentStorageTest, ValidComponentConst)
{
  // Attach a component to an entity
  EXPECT_EQ(ComponentAdditionResult::NEW_ADDITION,
      this->AddComponent(this->e1, poseComp.TypeId(), &poseComp));

  // Get a component that is attached to an entity
  auto derivedComp = this->ConstComponent<components::Pose>(this->e1);
  ASSERT_NE(nullptr, derivedComp);
  EXPECT_EQ(poseComp.Data(), derivedComp->Data());

  // Try to get a component that does not exist
  ASSERT_EQ(nullptr,
      this->ConstComponent<components::LinearVelocity>(this->e2));

  // Remove a component and try to retrieve it
  EXPECT_NE(nullptr, this->ConstComponent<components::Pose>(this->e1));
  EXPECT_TRUE(this->storage.RemoveComponent(this->e1,
        components::Pose::typeId));
  ASSERT_EQ(nullptr, this->ConstComponent<components::Pose>(this->e1));
}
