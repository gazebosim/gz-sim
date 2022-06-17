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

#include <gz/common/Console.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/detail/BaseView.hh"
#include "gz/sim/detail/View.hh"

#include "../test/helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

class BaseViewTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(BaseViewTest, ComponentTypes)
{
  auto modelNameView =
      detail::View({components::Model::typeId, components::Name::typeId});

  // make sure that a view's required component types are initialized properly
  EXPECT_EQ(2u, modelNameView.ComponentTypes().size());
  EXPECT_NE(modelNameView.ComponentTypes().find(components::Model::typeId),
      modelNameView.ComponentTypes().end());
  EXPECT_NE(modelNameView.ComponentTypes().find(components::Name::typeId),
      modelNameView.ComponentTypes().end());
  EXPECT_TRUE(modelNameView.RequiresComponent(components::Model::typeId));
  EXPECT_TRUE(modelNameView.RequiresComponent(components::Name::typeId));
  EXPECT_FALSE(modelNameView.RequiresComponent(components::Visual::typeId));
}

/////////////////////////////////////////////////
TEST_F(BaseViewTest, ToAddEntities)
{
  auto modelNameView =
      detail::View({components::Model::typeId, components::Name::typeId});
  const Entity e1 = 1;
  auto e1IsNew = true;

  // by default, the view shouldn't have any entities marked as ToAdd
  EXPECT_EQ(0u, modelNameView.ToAddEntities().size());
  EXPECT_FALSE(modelNameView.IsEntityMarkedForAddition(e1));

  // Calling ClearToAddEntities should have no effect since no entities are
  // marked as ToAdd
  modelNameView.ClearToAddEntities();
  EXPECT_EQ(0u, modelNameView.ToAddEntities().size());
  EXPECT_FALSE(modelNameView.IsEntityMarkedForAddition(e1));

  // Mark an entity as ToAdd (and as a "newly created" entity)
  EXPECT_TRUE(modelNameView.MarkEntityToAdd(e1, e1IsNew));
  EXPECT_TRUE(modelNameView.IsEntityMarkedForAddition(e1));
  EXPECT_EQ(1u, modelNameView.ToAddEntities().size());
  auto e1ToAddIter = modelNameView.ToAddEntities().find(e1);
  ASSERT_NE(e1ToAddIter, modelNameView.ToAddEntities().end());
  EXPECT_EQ(e1ToAddIter->second, e1IsNew);
  // (entities marked as ToAdd aren't considered as entities that are a part
  // of the view yet)
  EXPECT_FALSE(modelNameView.HasEntity(e1));

  // Try to mark an entity as ToAdd that is already marked as ToAdd.
  // This time around, the entity is not flagged as a "newly created" entity
  // when the entity is marked as ToAdd, so we should see this update when
  // inspecting the entity in the toAddEntities queue
  e1IsNew = false;
  EXPECT_TRUE(modelNameView.MarkEntityToAdd(e1, e1IsNew));
  EXPECT_TRUE(modelNameView.IsEntityMarkedForAddition(e1));
  EXPECT_EQ(1u, modelNameView.ToAddEntities().size());
  e1ToAddIter = modelNameView.ToAddEntities().find(e1);
  ASSERT_NE(e1ToAddIter, modelNameView.ToAddEntities().end());
  EXPECT_EQ(e1ToAddIter->second, e1IsNew);
  // (entities marked as ToAdd aren't considered as entities that are a part
  // of the view yet)
  EXPECT_FALSE(modelNameView.HasEntity(e1));

  // Remove an entity's ToAdd status and then re-mark it as ToAdd
  modelNameView.ClearToAddEntities();
  EXPECT_EQ(0u, modelNameView.ToAddEntities().size());
  EXPECT_FALSE(modelNameView.IsEntityMarkedForAddition(e1));
  e1IsNew = true;
  EXPECT_TRUE(modelNameView.MarkEntityToAdd(e1, e1IsNew));
  EXPECT_TRUE(modelNameView.IsEntityMarkedForAddition(e1));
  EXPECT_EQ(1u, modelNameView.ToAddEntities().size());
  e1ToAddIter = modelNameView.ToAddEntities().find(e1);
  ASSERT_NE(e1ToAddIter, modelNameView.ToAddEntities().end());
  EXPECT_EQ(e1ToAddIter->second, e1IsNew);
  // (entities marked as ToAdd aren't considered as entities that are a part
  // of the view yet)
  EXPECT_FALSE(modelNameView.HasEntity(e1));
}

/////////////////////////////////////////////////
TEST_F(BaseViewTest, AddEntities)
{
  auto modelNameView =
      detail::View({components::Model::typeId, components::Name::typeId});

  // Initially, the view should have no entities
  EXPECT_EQ(0u, modelNameView.Entities().size());
  EXPECT_EQ(0u, modelNameView.NewEntities().size());

  // Create a few entities with components for the view
  const Entity e1 = 1;
  const auto e1IsNew = false;
  auto e1ModelComp = components::Model();
  auto e1NameComp = components::Name("e1");

  const Entity e2 = 2;
  const auto e2IsNew = true;
  auto e2ModelComp = components::Model();
  auto e2NameComp = components::Name("e2");

  // Add the entities and their components to the view.
  EXPECT_FALSE(modelNameView.HasEntity(e1));
  EXPECT_FALSE(modelNameView.HasEntity(e2));
  EXPECT_FALSE(modelNameView.HasCachedComponentData(e1));
  EXPECT_FALSE(modelNameView.HasCachedComponentData(e2));
  modelNameView.AddEntityWithComps(e1, e1IsNew, &e1ModelComp, &e1NameComp);
  modelNameView.AddEntityWithConstComps(e1, e1IsNew, &e1ModelComp, &e1NameComp);
  modelNameView.AddEntityWithComps(e2, e2IsNew, &e2ModelComp, &e2NameComp);
  modelNameView.AddEntityWithConstComps(e2, e2IsNew, &e2ModelComp, &e2NameComp);
  EXPECT_TRUE(modelNameView.HasEntity(e1));
  EXPECT_TRUE(modelNameView.HasEntity(e2));
  EXPECT_TRUE(modelNameView.HasCachedComponentData(e1));
  EXPECT_TRUE(modelNameView.HasCachedComponentData(e2));
  EXPECT_EQ(2u, modelNameView.Entities().size());
  EXPECT_NE(modelNameView.Entities().find(e1), modelNameView.Entities().end());
  EXPECT_NE(modelNameView.Entities().find(e2), modelNameView.Entities().end());
  EXPECT_EQ(1u, modelNameView.NewEntities().size());
  EXPECT_NE(modelNameView.NewEntities().find(e2),
      modelNameView.NewEntities().end());

  auto e1ConstData = modelNameView.EntityComponentConstData(e1);
  ASSERT_EQ(2u, e1ConstData.size());
  EXPECT_EQ(&e1ModelComp, e1ConstData[0]);
  EXPECT_EQ(&e1NameComp, e1ConstData[1]);

  auto e1Data = modelNameView.EntityComponentData(e1);
  ASSERT_EQ(2u, e1Data.size());
  EXPECT_EQ(&e1ModelComp, e1Data[0]);
  EXPECT_EQ(&e1NameComp, e1Data[1]);

  auto e2ConstData = modelNameView .EntityComponentConstData(e2);
  ASSERT_EQ(2u, e2ConstData.size());
  EXPECT_EQ(&e2ModelComp, e2ConstData[0]);
  EXPECT_EQ(&e2NameComp, e2ConstData[1]);

  auto e2Data = modelNameView.EntityComponentData(e2);
  ASSERT_EQ(2u, e2Data.size());
  EXPECT_EQ(&e2ModelComp, e2Data[0]);
  EXPECT_EQ(&e2NameComp, e2Data[1]);
}

/////////////////////////////////////////////////
TEST_F(BaseViewTest, RemoveEntities)
{
  auto view = detail::View({components::Model::typeId});

  const Entity e1 = 1;
  auto e1ModelComp = components::Model();
  const Entity e2 = 2;
  auto e2ModelComp = components::Model();
  const auto isNewEntity = false;

  // add entities to the view
  view.AddEntityWithComps(e1, isNewEntity, &e1ModelComp);
  view.AddEntityWithConstComps(e1, isNewEntity, &e1ModelComp);
  view.AddEntityWithComps(e2, isNewEntity, &e2ModelComp);
  view.AddEntityWithConstComps(e2, isNewEntity, &e2ModelComp);
  EXPECT_TRUE(view.HasEntity(e1));
  EXPECT_TRUE(view.HasEntity(e2));
  EXPECT_TRUE(view.HasCachedComponentData(e1));
  EXPECT_TRUE(view.HasCachedComponentData(e2));

  // remove entities from the view
  EXPECT_EQ(0u, view.ToRemoveEntities().size());
  EXPECT_TRUE(view.RemoveEntity(e1));
  EXPECT_FALSE(view.HasEntity(e1));
  EXPECT_FALSE(view.HasCachedComponentData(e1));
  EXPECT_EQ(0u, view.ToRemoveEntities().size());
  EXPECT_TRUE(view.RemoveEntity(e2));
  EXPECT_FALSE(view.HasEntity(e2));
  EXPECT_FALSE(view.HasCachedComponentData(e2));
  EXPECT_EQ(0u, view.ToRemoveEntities().size());

  // try to remove an entity that is not in the view
  EXPECT_FALSE(view.RemoveEntity(e1));

  // add entities to the view's toAdd and toRemove queue
  EXPECT_TRUE(view.MarkEntityToAdd(e1));
  EXPECT_TRUE(view.MarkEntityToRemove(e1));
  EXPECT_EQ(1u, view.ToAddEntities().size());
  EXPECT_NE(view.ToAddEntities().end(), view.ToAddEntities().find(e1));
  EXPECT_EQ(1u, view.ToRemoveEntities().size());
  EXPECT_NE(view.ToRemoveEntities().end(), view.ToRemoveEntities().find(e1));

  // remove entities e1 and e2 from the view and make sure that the toAdd and
  // toRemove queues are updated to no longer have the removed entities
  EXPECT_TRUE(view.RemoveEntity(e1));
  EXPECT_EQ(0u, view.ToAddEntities().size());
  EXPECT_EQ(0u, view.ToRemoveEntities().size());
}

/////////////////////////////////////////////////
TEST_F(BaseViewTest, Reset)
{
  auto view = detail::View({components::Model::typeId});

  // initially, the view should be completely empty, except for its component
  // types (the view's component types are defined at object instantiation time
  // and never change)
  EXPECT_EQ(0u, view.Entities().size());
  EXPECT_EQ(0u, view.NewEntities().size());
  EXPECT_EQ(0u, view.ToAddEntities().size());
  EXPECT_EQ(0u, view.ToRemoveEntities().size());
  EXPECT_EQ(1u, view.ComponentTypes().size());
  EXPECT_NE(view.ComponentTypes().end(),
      view.ComponentTypes().find(components::Model::typeId));

  // populate the view with entity/component information
  const auto isNewEntity = true;
  const Entity e1 = 1;
  auto e1ModelComp = components::Model();
  view.AddEntityWithComps(e1, isNewEntity, &e1ModelComp);
  view.AddEntityWithConstComps(e1, isNewEntity, &e1ModelComp);
  EXPECT_TRUE(view.HasEntity(e1));
  EXPECT_TRUE(view.HasCachedComponentData(e1));
  EXPECT_TRUE(view.MarkEntityToRemove(e1));
  const Entity e2 = 2;
  EXPECT_TRUE(view.MarkEntityToAdd(e2));
  EXPECT_TRUE(view.IsEntityMarkedForAddition(e2));

  // make sure the view doesn't have empty entity-related data structures
  EXPECT_EQ(1u, view.Entities().size());
  EXPECT_EQ(1u, view.NewEntities().size());
  EXPECT_EQ(1u, view.ToAddEntities().size());
  EXPECT_EQ(1u, view.ToRemoveEntities().size());

  // reset the view and make sure that it's back in its original state
  view.Reset();
  EXPECT_FALSE(view.HasEntity(e1));
  EXPECT_FALSE(view.HasCachedComponentData(e1));
  EXPECT_FALSE(view.IsEntityMarkedForAddition(e2));
  EXPECT_EQ(0u, view.Entities().size());
  EXPECT_EQ(0u, view.NewEntities().size());
  EXPECT_EQ(0u, view.ToAddEntities().size());
  EXPECT_EQ(0u, view.ToRemoveEntities().size());
  EXPECT_EQ(1u, view.ComponentTypes().size());
  EXPECT_NE(view.ComponentTypes().end(),
      view.ComponentTypes().find(components::Model::typeId));

  // add newly created entities to the view
  view.AddEntityWithComps(e1, isNewEntity, &e1ModelComp);
  view.AddEntityWithConstComps(e1, isNewEntity, &e1ModelComp);
  EXPECT_TRUE(view.HasEntity(e1));
  EXPECT_TRUE(view.HasCachedComponentData(e1));
  EXPECT_TRUE(view.MarkEntityToRemove(e1));
  EXPECT_EQ(1u, view.Entities().size());
  EXPECT_EQ(1u, view.NewEntities().size());
  EXPECT_TRUE(view.MarkEntityToAdd(e2, isNewEntity));
  EXPECT_TRUE(view.IsEntityMarkedForAddition(e2));
  EXPECT_EQ(1u, view.ToAddEntities().size());
  auto e2ToAddIter = view.ToAddEntities().find(e2);
  ASSERT_NE(view.ToAddEntities().end(), e2ToAddIter);
  EXPECT_EQ(e2ToAddIter->second, isNewEntity);

  // reset the newly created entity state of the view
  view.ResetNewEntityState();
  EXPECT_TRUE(view.HasEntity(e1));
  EXPECT_TRUE(view.HasCachedComponentData(e1));
  EXPECT_TRUE(view.MarkEntityToRemove(e1));
  EXPECT_TRUE(view.IsEntityMarkedForAddition(e2));
  EXPECT_EQ(1u, view.Entities().size());
  EXPECT_EQ(0u, view.NewEntities().size());
  EXPECT_EQ(1u, view.ToAddEntities().size());
  e2ToAddIter = view.ToAddEntities().find(e2);
  ASSERT_NE(view.ToAddEntities().end(), e2ToAddIter);
  // entity e2 should still be an entity marked as ToAdd, but it shouldn't
  // be a newly created entity anymore
  EXPECT_FALSE(e2ToAddIter->second);
}

/////////////////////////////////////////////////
TEST_F(BaseViewTest, CachedComponentData)
{
  auto view = detail::View({components::Model::typeId});

  const Entity e1 = 1;
  const auto e1IsNew = true;
  auto e1ModelComp = components::Model();

  EXPECT_FALSE(view.HasCachedComponentData(e1));

  // add both const and non-const component data for e1 to the view
  view.AddEntityWithComps(e1, e1IsNew, &e1ModelComp);
  view.AddEntityWithConstComps(e1, e1IsNew, &e1ModelComp);
  EXPECT_TRUE(view.HasCachedComponentData(e1));

  // reset the view and add only const component data this time
  view.Reset();
  EXPECT_FALSE(view.HasCachedComponentData(e1));
  view.AddEntityWithConstComps(e1, e1IsNew, &e1ModelComp);
  EXPECT_FALSE(view.HasCachedComponentData(e1));

  // reset the view and add only non-const component data this time
  view.Reset();
  EXPECT_FALSE(view.HasCachedComponentData(e1));
  view.AddEntityWithComps(e1, e1IsNew, &e1ModelComp);
  EXPECT_FALSE(view.HasCachedComponentData(e1));
}

/////////////////////////////////////////////////
TEST_F(BaseViewTest, ComponentChangeNotification)
{
  auto view = detail::View({components::Model::typeId,
      components::Visual::typeId});

  const Entity e1 = 1;
  const auto e1IsNew = true;
  auto e1ModelComp = components::Model();
  auto e1VisualComp = components::Visual();

  // add the entity and its component data to the view
  view.AddEntityWithComps(e1, e1IsNew, &e1ModelComp, &e1VisualComp);
  view.AddEntityWithConstComps(e1, e1IsNew, &e1ModelComp, &e1VisualComp);
  EXPECT_TRUE(view.HasCachedComponentData(e1));
  EXPECT_TRUE(view.HasEntity(e1));
  EXPECT_EQ(1u, view.Entities().size());
  EXPECT_NE(view.Entities().end(), view.Entities().find(e1));
  EXPECT_EQ(1u, view.NewEntities().size());
  EXPECT_NE(view.NewEntities().end(), view.NewEntities().find(e1));

  // mimic a removal of e1's model component by notifying the view that this
  // component was removed
  EXPECT_TRUE(view.NotifyComponentRemoval(e1, components::Model::typeId));
  EXPECT_FALSE(view.HasEntity(e1));
  EXPECT_EQ(0u, view.Entities().size());
  EXPECT_EQ(0u, view.NewEntities().size());
  EXPECT_TRUE(view.HasCachedComponentData(e1));

  // mimic a removal of e1's visual component by notifying the view that this
  // component was removed
  EXPECT_TRUE(view.NotifyComponentRemoval(e1, components::Visual::typeId));
  EXPECT_FALSE(view.HasEntity(e1));
  EXPECT_EQ(0u, view.Entities().size());
  EXPECT_EQ(0u, view.NewEntities().size());
  EXPECT_TRUE(view.HasCachedComponentData(e1));

  // mimic re-addition of e1's model component by notifying the view that this
  // component was added. At this point, the visual component is still missing,
  // so e1 shouldn't be a part of the view yet
  EXPECT_TRUE(view.NotifyComponentAddition(e1, e1IsNew,
        components::Model::typeId));
  EXPECT_TRUE(view.HasCachedComponentData(e1));
  EXPECT_FALSE(view.HasEntity(e1));
  EXPECT_EQ(0u, view.Entities().size());
  EXPECT_EQ(0u, view.NewEntities().size());

  // mimic re-addition of e1's visual component by notifying the view that this
  // component was added. Now that both the model and visual components have
  // been re-added to e1, e1 should be a part of the view
  EXPECT_TRUE(view.NotifyComponentAddition(e1, e1IsNew,
        components::Visual::typeId));
  EXPECT_TRUE(view.HasCachedComponentData(e1));
  EXPECT_TRUE(view.HasEntity(e1));
  EXPECT_EQ(1u, view.Entities().size());
  EXPECT_NE(view.Entities().end(), view.Entities().find(e1));
  EXPECT_EQ(1u, view.NewEntities().size());
  EXPECT_NE(view.NewEntities().end(), view.NewEntities().find(e1));

  // try to call NotifyComponent* methods with component types that don't
  // belong to the view
  EXPECT_TRUE(view.HasEntity(e1));
  EXPECT_TRUE(view.HasCachedComponentData(e1));
  EXPECT_FALSE(view.RequiresComponent(components::Name::typeId));
  EXPECT_FALSE(view.NotifyComponentRemoval(e1, components::Name::typeId));
  EXPECT_FALSE(view.NotifyComponentAddition(e1, e1IsNew,
        components::Name::typeId));
  EXPECT_TRUE(view.HasEntity(e1));
  EXPECT_TRUE(view.HasCachedComponentData(e1));

  // try to call NotifyComponent* methods with entities that aren't a part of
  // the view
  const Entity e2 = 2;
  const auto e2IsNew = false;
  EXPECT_FALSE(view.HasEntity(e2));
  EXPECT_FALSE(view.HasCachedComponentData(e2));
  EXPECT_TRUE(view.RequiresComponent(components::Model::typeId));
  EXPECT_FALSE(view.NotifyComponentRemoval(e2, components::Model::typeId));
  EXPECT_FALSE(view.NotifyComponentAddition(e2, e2IsNew,
        components::Model::typeId));

  // add another entity to the view that isn't a newly created entity to make
  // sure that calling NotifyComponent* methods on this entity doesn't modify
  // the view's new entity data
  EXPECT_FALSE(view.HasEntity(e2));
  EXPECT_FALSE(view.HasCachedComponentData(e2));
  auto e2ModelComp = components::Model();
  auto e2VisualComp = components::Visual();
  view.AddEntityWithComps(e2, e2IsNew, &e2ModelComp, &e2VisualComp);
  view.AddEntityWithConstComps(e2, e2IsNew, &e2ModelComp, &e2VisualComp);
  EXPECT_TRUE(view.HasCachedComponentData(e2));
  EXPECT_TRUE(view.HasEntity(e2));
  EXPECT_EQ(2u, view.Entities().size());
  EXPECT_NE(view.Entities().end(), view.Entities().find(e1));
  EXPECT_NE(view.Entities().end(), view.Entities().find(e2));
  EXPECT_EQ(1u, view.NewEntities().size());
  EXPECT_EQ(view.NewEntities().end(), view.NewEntities().find(e2));

  // call NotifyComponentRemoval on the entity that was just added to the view
  EXPECT_TRUE(view.NotifyComponentRemoval(e2, components::Model::typeId));
  EXPECT_FALSE(view.HasEntity(e2));
  EXPECT_EQ(1u, view.Entities().size());
  EXPECT_EQ(view.Entities().end(), view.Entities().find(e2));
  EXPECT_EQ(1u, view.NewEntities().size());
  EXPECT_TRUE(view.HasCachedComponentData(e2));

  // call NotifyComponentRemoval on a component that was already notified of
  // removal. While the notification should still take place, it will have no
  // effect since this component was already removed
  EXPECT_TRUE(view.NotifyComponentRemoval(e2, components::Model::typeId));
  EXPECT_FALSE(view.HasEntity(e2));
  EXPECT_EQ(1u, view.Entities().size());
  EXPECT_EQ(view.Entities().end(), view.Entities().find(e2));
  EXPECT_EQ(1u, view.NewEntities().size());
  EXPECT_TRUE(view.HasCachedComponentData(e2));

  // call NotifyComponentAddition on the entity that was just added to the view
  EXPECT_TRUE(view.NotifyComponentAddition(e2, e2IsNew,
        components::Model::typeId));
  EXPECT_TRUE(view.HasCachedComponentData(e2));
  EXPECT_TRUE(view.HasEntity(e2));
  EXPECT_EQ(2u, view.Entities().size());
  EXPECT_NE(view.Entities().end(), view.Entities().find(e1));
  EXPECT_NE(view.Entities().end(), view.Entities().find(e2));
  EXPECT_EQ(1u, view.NewEntities().size());
  EXPECT_EQ(view.NewEntities().end(), view.NewEntities().find(e2));

  // call NotifyComponentAddition on a component that was already notified of
  // addition. While the notification should still take place, it will have no
  // effect since this component was already added
  EXPECT_TRUE(view.NotifyComponentAddition(e2, e2IsNew,
        components::Model::typeId));
  EXPECT_TRUE(view.HasCachedComponentData(e2));
  EXPECT_TRUE(view.HasEntity(e2));
  EXPECT_EQ(2u, view.Entities().size());
  EXPECT_NE(view.Entities().end(), view.Entities().find(e1));
  EXPECT_NE(view.Entities().end(), view.Entities().find(e2));
  EXPECT_EQ(1u, view.NewEntities().size());
  EXPECT_EQ(view.NewEntities().end(), view.NewEntities().find(e2));
}

/////////////////////////////////////////////////
TEST_F(BaseViewTest, ComponentTypeHasher)
{
  // Test the hash function for a std::vector<ComponentTypeId> to make
  // sure that views with different component types (either a different type
  // ordering or a different set of types alltogether) are considered unique
  using ComponentVec = std::vector<ComponentTypeId>;

  // Create vectors with the same types, but different order
  ComponentVec vec1 = {
    components::Model::typeId,
    components::Name::typeId,
    components::Visual::typeId
  };
  ComponentVec vec2 = {
    components::Name::typeId,
    components::Model::typeId,
    components::Visual::typeId
  };
  ComponentVec vec3 = {
    components::Model::typeId,
    components::Visual::typeId,
    components::Name::typeId
  };

  // Create vectors with different types
  ComponentVec vec4 = {components::Model::typeId};
  ComponentVec vec5 = {components::Name::typeId};
  ComponentVec vec6 = {components::Visual::typeId};
  ComponentVec vec7 = {
    components::Model::typeId,
    components::Name::typeId
  };

  // Test the hash function. Each vector defined above should be unique, which
  // means that the std::unordered_set should have 7 elements in it after all
  // insertions have been attempted (7 unique vectors were created)
  std::unordered_set<ComponentVec, detail::ComponentTypeHasher> uniqueVecs;
  uniqueVecs.insert(vec1);
  uniqueVecs.insert(vec2);
  uniqueVecs.insert(vec3);
  uniqueVecs.insert(vec4);
  uniqueVecs.insert(vec5);
  uniqueVecs.insert(vec6);
  uniqueVecs.insert(vec7);
  EXPECT_EQ(7u, uniqueVecs.size());
}
