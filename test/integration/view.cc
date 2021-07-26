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

#include <cstddef>
#include <memory>
#include <set>
#include <string>
#include <tuple>

#include <ignition/common/Console.hh>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentStorage.hh"
#include "ignition/gazebo/Types.hh"
#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/detail/View.hh"

using namespace ignition;
using namespace gazebo;

// Alias for a set of views
using ViewSet = std::set<std::shared_ptr<detail::BaseView>>;

// Entities used across various test cases
const Entity e1 = 1;
const Entity e2 = 2;
const Entity e3 = 3;
const Entity e4 = 4;
const Entity e5 = 5;
const Entity e6 = 6;

const auto kNewEntity = true;
const auto kNotNewEntity = false;

class ViewTest : public::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }
};

/// \brief Helper class that wraps EntityComponentStorage to provide a
/// simplified API for working with entities and their components. This class
/// also stores a set of user-defined views and handles updating views as
/// needed whenever an entity's components are modified.
class StorageViewWrapper
{
  /// \brief Constructor
  /// \param[in] _views The set of views that will be updated whenever
  /// components are modified
  public: explicit StorageViewWrapper(const ViewSet &_views)
          : views(_views)
  {
  }

  /// \brief Add an entity to the internal storage, so that it can have
  /// components assigned to it and be used in views.
  /// \param[in] _entity The entity
  /// \return True if the entity was added, false otherwise
  public: bool AddEntity(const Entity _entity)
  {
    return this->storage.AddEntity(_entity);
  }

  /// \brief Remove an entity and all of its components from the internal
  /// storage
  /// \param[in] _entity The entity
  /// \return True if the entity and its components were removed, false
  /// otherwise
  public: bool RemoveEntity(const Entity _entity)
  {
    for (auto &view : this->views)
      view->RemoveEntity(_entity);

    return this->storage.RemoveEntity(_entity);
  }

  /// \brief Process all entities marked as toAdd in a view by caching each
  /// toAdd entity's component data in the view. The view should have no
  /// entities marked as toAdd after calling this method.
  /// \param[in] _view The view
  public: template<typename ...ComponentTypeTs>
          void UpdateView(
              std::shared_ptr<detail::View<ComponentTypeTs...>> _view)
  {
    for (const auto &[entity, isNew] : _view->ToAddEntities())
    {
      _view->AddEntityWithComps(entity, isNew,
          this->Component<ComponentTypeTs>(entity)...);
      _view->AddEntityWithConstComps(entity, isNew,
          this->Component<ComponentTypeTs>(entity)...);
    }
    _view->ClearToAddEntities();
  }

  /// \brief Add a component to a (possibly new) entity
  /// \param[in] _entity The entity that will own the new component
  /// \param[in] _component The new component, which belongs to _entity
  /// \param[in] _isNewEntity Whether _entity is considered "newly created"
  /// (true) or not (false)
  public: template<typename ComponentTypeT>
          void AddNewComponent(const Entity _entity,
              const ComponentTypeT &_component, bool _isNewEntity = false)
  {
    EXPECT_EQ(ComponentAdditionResult::NEW_ADDITION,
        this->AddComponent(_entity, _component));

    for (auto &view : this->views)
    {
      if (this->ComponentsMatch(_entity, view))
        view->MarkEntityToAdd(_entity, _isNewEntity);
    }
  }

  /// \brief Remove a component of a particular type from an entity
  /// \param[in] _entity The entity to remove a component from
  /// \param[in] _typeId The type ID of the component to be removed
  public: void RemoveComponent(const Entity _entity,
              const ComponentTypeId _typeId)
  {
    EXPECT_TRUE(this->storage.RemoveComponent(_entity, _typeId));

    for (auto &view : this->views)
      view->NotifyComponentRemoval(_entity, _typeId);
  }

  /// \brief Add a component to an entity that had a component of the same type
  /// previously, but was removed
  /// \param[in] _entity The entity that will own the component
  /// \param[in] _kNewEntity Whether the entity is "newly created" or not
  /// \param[in] _component The component, which belongs to _entity
  public: template<typename ComponentTypeT>
          void ReAddComponent(const Entity _entity, bool _kNewEntity,
              const ComponentTypeT &_component)
  {
    EXPECT_EQ(ComponentAdditionResult::RE_ADDITION,
        this->AddComponent(_entity, _component));

    // after the component has been "re-added", the actual data of the component
    // still needs to be updated (re-adding updates the component's internal
    // "removed" flag, which is used by Views)
    this->ModifyComponent(_entity, _component);

    for (auto &view : this->views)
    {
      view->NotifyComponentAddition(_entity, _kNewEntity,
          ComponentTypeT::typeId);
    }
  }

  /// \brief Modify a component that belongs to an entity
  /// \param[in] _entity The entity that has the component to be modified
  /// \param[in] _component The new component that should be used to modify
  /// _entity's existing component of the same type
  public: template<typename ComponentTypeT>
          void ModifyComponent(const Entity _entity,
              const ComponentTypeT &_component)
  {
    EXPECT_EQ(ComponentAdditionResult::MODIFICATION,
        this->AddComponent(_entity, _component));
    auto entityComp = this->Component<ComponentTypeT>(_entity);
    ASSERT_NE(nullptr, entityComp);
    entityComp->Data() = _component.Data();
    EXPECT_EQ(entityComp->Data(), _component.Data());
  }

  /// \brief Helper function that uses EntityComponentStorage::ValidComponent
  /// to get a component of a particular type that belongs to an entity.
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

  /// \brief See if an entity has all of the component types that are required
  /// by a view
  /// \param[in] _entity The entity
  /// \param[in] _view The view
  /// \return True if _entity has components of all types required by _view.
  /// False otherwise
  private: bool ComponentsMatch(const Entity _entity,
               const std::shared_ptr<detail::BaseView> &_view) const
  {
    for (const auto &compType : _view->ComponentTypes())
    {
      if (!this->storage.ValidComponent(_entity, compType))
        return false;
    }
    return true;
  }

  /// \brief Helper function for adding a component to the storage
  /// \param[in] _entity The entity that owns _component
  /// \param[in] _component The component, which belongs to _entity
  /// \return The result of the component addition
  private: template<typename ComponentTypeT>
           ComponentAdditionResult AddComponent(const Entity _entity,
               const ComponentTypeT &_component)
  {
    auto compPtr = components::Factory::Instance()->New(
        ComponentTypeT::typeId, &_component);
    return this->storage.AddComponent(_entity, std::move(compPtr));
  }

  /// \brief The actual EntityComponentStorage instance
  private: EntityComponentStorage storage;

  /// \brief All of the views that will be updated as needed based on the
  /// changes being made to this->storage
  private: ViewSet views;
};

/////////////////////////////////////////////////
// Unit test the StorageViewWrapper class to make sure it behaves correctly
// before using it in the View integration tests
TEST_F(ViewTest, StorageViewWrapperTest)
{
  ViewSet noViews = {};
  StorageViewWrapper wrapper(noViews);

  // define comonents for entities
  const std::string e1nameCompStr = "e1comp";
  auto e1comp = components::Name(e1nameCompStr);
  const std::string e2nameCompStr = "e2comp";
  auto e2comp = components::Name(e2nameCompStr);

  // create entities and attach components to them
  EXPECT_TRUE(wrapper.AddEntity(e1));
  wrapper.AddNewComponent(e1, e1comp);
  EXPECT_TRUE(wrapper.AddEntity(e2));
  wrapper.AddNewComponent(e2, e2comp);
  // (try to create entities that were already created)
  EXPECT_FALSE(wrapper.AddEntity(e1));

  // modify each entity's component data
  const std::string modifiedSuffix = "_modified";
  wrapper.ModifyComponent(e1, components::Name(e1nameCompStr + modifiedSuffix));
  wrapper.ModifyComponent(e2, components::Name(e2nameCompStr + modifiedSuffix));

  // remove each entity's components
  wrapper.RemoveComponent(e1, components::Name::typeId);
  wrapper.RemoveComponent(e2, components::Name::typeId);

  // re-add components to entities
  wrapper.ReAddComponent(e1, kNotNewEntity, e1comp);
  wrapper.ReAddComponent(e2, kNewEntity, e2comp);

  // check that an enity's component instance can be retrieved
  auto e1compInstance = wrapper.Component<components::Name>(e1);
  ASSERT_NE(nullptr, e1compInstance);
  EXPECT_EQ(e1comp.Data(), e1compInstance->Data());
  auto e2compInstance = wrapper.Component<components::Name>(e2);
  ASSERT_NE(nullptr, e2compInstance);
  EXPECT_EQ(e2comp.Data(), e2compInstance->Data());

  // remove entities
  EXPECT_TRUE(wrapper.RemoveEntity(e1));
  EXPECT_TRUE(wrapper.RemoveEntity(e2));
  // (try to remove entities that were never added)
  EXPECT_FALSE(wrapper.RemoveEntity(e3));
  // (try to remove entities that were already removed)
  EXPECT_FALSE(wrapper.RemoveEntity(e1));
}

/////////////////////////////////////////////////
TEST_F(ViewTest, UseViews)
{
  // create several different types of views
  auto modelView = std::make_shared<detail::View<components::Model>>();
  auto sensorView = std::make_shared<detail::View<components::Sensor>>();
  auto visualView = std::make_shared<detail::View<components::Visual>>();
  auto nameView = std::make_shared<detail::View<components::Name>>();
  auto modelNameView = std::make_shared<
    detail::View<components::Model, components::Name>>();
  auto sensorNameView = std::make_shared<
    detail::View<components::Sensor, components::Name>>();
  auto modelVisualView = std::make_shared<
    detail::View<components::Model, components::Visual>>();
  auto visualNameView = std::make_shared<
    detail::View<components::Visual, components::Name>>();

  ViewSet views = {
    modelView,
    sensorView,
    visualView,
    nameView,
    modelNameView,
    sensorNameView,
    modelVisualView,
    visualNameView
  };

  StorageViewWrapper wrapper(views);

  // create entities with various components
  EXPECT_TRUE(wrapper.AddEntity(e1));
  wrapper.AddNewComponent(e1, components::Model());
  wrapper.AddNewComponent(e1, components::Name("modelEntity1"));

  EXPECT_TRUE(wrapper.AddEntity(e2));
  wrapper.AddNewComponent(e2, components::Model(), kNewEntity);
  wrapper.AddNewComponent(e2, components::Name("modelEntity2"), kNewEntity);

  const std::string kOriginalNameCompE3 = "sensorEntity3";
  EXPECT_TRUE(wrapper.AddEntity(e3));
  wrapper.AddNewComponent(e3, components::Sensor());
  wrapper.AddNewComponent(e3, components::Name(kOriginalNameCompE3));

  EXPECT_TRUE(wrapper.AddEntity(e4));
  wrapper.AddNewComponent(e4, components::Sensor(), kNewEntity);
  wrapper.AddNewComponent(e4, components::Name("sensorEntity4"), kNewEntity);

  EXPECT_TRUE(wrapper.AddEntity(e5));
  wrapper.AddNewComponent(e5, components::Visual());

  EXPECT_TRUE(wrapper.AddEntity(e6));
  wrapper.AddNewComponent(e6, components::Visual(), kNewEntity);

  std::size_t numViewChecks = 0;
  for (const auto &view : views)
  {
    // at this point, the views have entities in their toAddEntities queue,
    // but the entities and their component data haven't been stored in the
    // views yet. So, the views should have no entities (an entity in the
    // toAddEntities queue isn't an entity that's a part of the view)
    EXPECT_EQ(0u, view->Entities().size());
    EXPECT_EQ(0u, view->NewEntities().size());
    EXPECT_EQ(0u, view->ToRemoveEntities().size());
    EXPECT_FALSE(view->HasEntity(e1));
    EXPECT_FALSE(view->HasEntity(e2));
    EXPECT_FALSE(view->HasEntity(e3));
    EXPECT_FALSE(view->HasEntity(e4));
    EXPECT_FALSE(view->HasEntity(e5));
    EXPECT_FALSE(view->HasEntity(e6));
    EXPECT_FALSE(view->HasCachedComponentData(e1));
    EXPECT_FALSE(view->HasCachedComponentData(e2));
    EXPECT_FALSE(view->HasCachedComponentData(e3));
    EXPECT_FALSE(view->HasCachedComponentData(e4));
    EXPECT_FALSE(view->HasCachedComponentData(e5));
    EXPECT_FALSE(view->HasCachedComponentData(e6));

    // the model view and modelName view should have entities e1 and e2 in
    // the toAddEntities queue
    if (view == modelView || view == modelNameView)
    {
      numViewChecks++;

      EXPECT_TRUE(view->IsEntityMarkedForAddition(e1));
      EXPECT_TRUE(view->IsEntityMarkedForAddition(e2));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e3));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e4));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e5));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e6));

      EXPECT_EQ(2u, view->ToAddEntities().size());

      auto e1Iter = view->ToAddEntities().find(e1);
      ASSERT_NE(view->ToAddEntities().end(), e1Iter);
      EXPECT_EQ(e1Iter->second, kNotNewEntity);

      auto e2Iter = view->ToAddEntities().find(e2);
      ASSERT_NE(view->ToAddEntities().end(), e2Iter);
      EXPECT_EQ(e2Iter->second, kNewEntity);
    }

    // the sensor view and sensorName view should have entities e3 and e4 in
    // the toAddEntities queue
    if (view == sensorView || view == sensorNameView)
    {
      numViewChecks++;

      EXPECT_EQ(2u, view->ToAddEntities().size());

      EXPECT_FALSE(view->IsEntityMarkedForAddition(e1));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e2));
      EXPECT_TRUE(view->IsEntityMarkedForAddition(e3));
      EXPECT_TRUE(view->IsEntityMarkedForAddition(e4));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e5));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e6));

      auto e3Iter = view->ToAddEntities().find(e3);
      ASSERT_NE(view->ToAddEntities().end(), e3Iter);
      EXPECT_EQ(e3Iter->second, kNotNewEntity);

      auto e4Iter = view->ToAddEntities().find(e4);
      ASSERT_NE(view->ToAddEntities().end(), e4Iter);
      EXPECT_EQ(e4Iter->second, kNewEntity);
    }

    // the visual view should have entities e5 and e6 in the toAddEntities
    // queue
    if (view == visualView)
    {
      numViewChecks++;

      EXPECT_EQ(2u, view->ToAddEntities().size());

      EXPECT_FALSE(view->IsEntityMarkedForAddition(e1));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e2));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e3));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e4));
      EXPECT_TRUE(view->IsEntityMarkedForAddition(e5));
      EXPECT_TRUE(view->IsEntityMarkedForAddition(e6));

      auto e5Iter = view->ToAddEntities().find(e5);
      ASSERT_NE(view->ToAddEntities().end(), e5Iter);
      EXPECT_EQ(e5Iter->second, kNotNewEntity);

      auto e6Iter = view->ToAddEntities().find(e6);
      ASSERT_NE(view->ToAddEntities().end(), e6Iter);
      EXPECT_EQ(e6Iter->second, kNewEntity);
    }

    // the name view should have entities e1 through e4 in the toAddEntities
    // queue
    if (view == nameView)
    {
      numViewChecks++;

      EXPECT_EQ(4u, view->ToAddEntities().size());

      EXPECT_TRUE(view->IsEntityMarkedForAddition(e1));
      EXPECT_TRUE(view->IsEntityMarkedForAddition(e2));
      EXPECT_TRUE(view->IsEntityMarkedForAddition(e3));
      EXPECT_TRUE(view->IsEntityMarkedForAddition(e4));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e5));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e6));

      auto e1Iter = view->ToAddEntities().find(e1);
      ASSERT_NE(view->ToAddEntities().end(), e1Iter);
      EXPECT_EQ(e1Iter->second, kNotNewEntity);

      auto e2Iter = view->ToAddEntities().find(e2);
      ASSERT_NE(view->ToAddEntities().end(), e2Iter);
      EXPECT_EQ(e2Iter->second, kNewEntity);

      auto e3Iter = view->ToAddEntities().find(e3);
      ASSERT_NE(view->ToAddEntities().end(), e3Iter);
      EXPECT_EQ(e3Iter->second, kNotNewEntity);

      auto e4Iter = view->ToAddEntities().find(e4);
      ASSERT_NE(view->ToAddEntities().end(), e4Iter);
      EXPECT_EQ(e4Iter->second, kNewEntity);
    }

    // the modelVisual view and visualName view should have an empty
    // toAddEntities queue
    if (view == modelVisualView || view == visualNameView)
    {
      numViewChecks++;
      EXPECT_EQ(0u, view->ToAddEntities().size());

      EXPECT_FALSE(view->IsEntityMarkedForAddition(e1));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e2));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e3));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e4));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e5));
      EXPECT_FALSE(view->IsEntityMarkedForAddition(e6));
    }
  }
  EXPECT_EQ(views.size(), numViewChecks);

  // update/process views and check view contents
  wrapper.UpdateView(modelView);
  wrapper.UpdateView(sensorView);
  wrapper.UpdateView(visualView);
  wrapper.UpdateView(nameView);
  wrapper.UpdateView(modelNameView);
  wrapper.UpdateView(sensorNameView);
  wrapper.UpdateView(modelVisualView);
  wrapper.UpdateView(visualNameView);

  numViewChecks = 0;
  for (const auto &view : views)
  {
    numViewChecks++;
    EXPECT_EQ(0u, view->ToAddEntities().size());
    EXPECT_EQ(0u, view->ToRemoveEntities().size());
  }
  EXPECT_EQ(views.size(), numViewChecks);

  EXPECT_TRUE(modelView->HasEntity(e1));
  EXPECT_TRUE(modelView->HasEntity(e2));
  EXPECT_FALSE(modelView->HasEntity(e3));
  EXPECT_FALSE(modelView->HasEntity(e4));
  EXPECT_FALSE(modelView->HasEntity(e5));
  EXPECT_FALSE(modelView->HasEntity(e6));
  EXPECT_EQ(2u, modelView->Entities().size());
  EXPECT_NE(modelView->Entities().end(), modelView->Entities().find(e1));
  EXPECT_NE(modelView->Entities().end(), modelView->Entities().find(e2));
  EXPECT_EQ(1u, modelView->NewEntities().size());
  EXPECT_NE(modelView->NewEntities().end(), modelView->NewEntities().find(e2));

  EXPECT_FALSE(sensorView->HasEntity(e1));
  EXPECT_FALSE(sensorView->HasEntity(e2));
  EXPECT_TRUE(sensorView->HasEntity(e3));
  EXPECT_TRUE(sensorView->HasEntity(e4));
  EXPECT_FALSE(sensorView->HasEntity(e5));
  EXPECT_FALSE(sensorView->HasEntity(e6));
  EXPECT_EQ(2u, sensorView->Entities().size());
  EXPECT_NE(sensorView->Entities().end(), sensorView->Entities().find(e3));
  EXPECT_NE(sensorView->Entities().end(), sensorView->Entities().find(e4));
  EXPECT_EQ(1u, sensorView->NewEntities().size());
  EXPECT_NE(sensorView->NewEntities().end(),
      sensorView->NewEntities().find(e4));

  EXPECT_FALSE(visualView->HasEntity(e1));
  EXPECT_FALSE(visualView->HasEntity(e2));
  EXPECT_FALSE(visualView->HasEntity(e3));
  EXPECT_FALSE(visualView->HasEntity(e4));
  EXPECT_TRUE(visualView->HasEntity(e5));
  EXPECT_TRUE(visualView->HasEntity(e6));
  EXPECT_EQ(2u, visualView->Entities().size());
  EXPECT_NE(visualView->Entities().end(), visualView->Entities().find(e5));
  EXPECT_NE(visualView->Entities().end(), visualView->Entities().find(e6));
  EXPECT_EQ(1u, visualView->NewEntities().size());
  EXPECT_NE(visualView->NewEntities().end(),
      visualView->NewEntities().find(e6));

  EXPECT_TRUE(nameView->HasEntity(e1));
  EXPECT_TRUE(nameView->HasEntity(e2));
  EXPECT_TRUE(nameView->HasEntity(e3));
  EXPECT_TRUE(nameView->HasEntity(e4));
  EXPECT_FALSE(nameView->HasEntity(e5));
  EXPECT_FALSE(nameView->HasEntity(e6));
  EXPECT_EQ(4u, nameView->Entities().size());
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e1));
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e2));
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e3));
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e4));
  EXPECT_EQ(2u, nameView->NewEntities().size());
  EXPECT_NE(nameView->NewEntities().end(), nameView->NewEntities().find(e2));
  EXPECT_NE(nameView->NewEntities().end(), nameView->NewEntities().find(e4));

  EXPECT_TRUE(modelNameView->HasEntity(e1));
  EXPECT_TRUE(modelNameView->HasEntity(e2));
  EXPECT_FALSE(modelNameView->HasEntity(e3));
  EXPECT_FALSE(modelNameView->HasEntity(e4));
  EXPECT_FALSE(modelNameView->HasEntity(e5));
  EXPECT_FALSE(modelNameView->HasEntity(e6));
  EXPECT_EQ(2u, modelNameView->Entities().size());
  EXPECT_NE(modelNameView->Entities().end(),
      modelNameView->Entities().find(e1));
  EXPECT_NE(modelNameView->Entities().end(),
      modelNameView->Entities().find(e2));
  EXPECT_EQ(1u, modelNameView->NewEntities().size());
  EXPECT_NE(modelNameView->NewEntities().end(),
      modelNameView->NewEntities().find(e2));

  EXPECT_FALSE(sensorNameView->HasEntity(e1));
  EXPECT_FALSE(sensorNameView->HasEntity(e2));
  EXPECT_TRUE(sensorNameView->HasEntity(e3));
  EXPECT_TRUE(sensorNameView->HasEntity(e4));
  EXPECT_FALSE(sensorNameView->HasEntity(e5));
  EXPECT_FALSE(sensorNameView->HasEntity(e6));
  EXPECT_EQ(2u, sensorNameView->Entities().size());
  EXPECT_NE(sensorNameView->Entities().end(),
      sensorNameView->Entities().find(e3));
  EXPECT_NE(sensorNameView->Entities().end(),
      sensorNameView->Entities().find(e4));
  EXPECT_EQ(1u, sensorNameView->NewEntities().size());
  EXPECT_NE(sensorNameView->NewEntities().end(),
      sensorNameView->NewEntities().find(e4));

  EXPECT_FALSE(modelVisualView->HasEntity(e1));
  EXPECT_FALSE(modelVisualView->HasEntity(e2));
  EXPECT_FALSE(modelVisualView->HasEntity(e3));
  EXPECT_FALSE(modelVisualView->HasEntity(e4));
  EXPECT_FALSE(modelVisualView->HasEntity(e5));
  EXPECT_FALSE(modelVisualView->HasEntity(e6));
  EXPECT_EQ(0u, modelVisualView->Entities().size());
  EXPECT_EQ(0u, modelVisualView->Entities().size());

  EXPECT_FALSE(visualNameView->HasEntity(e1));
  EXPECT_FALSE(visualNameView->HasEntity(e2));
  EXPECT_FALSE(visualNameView->HasEntity(e3));
  EXPECT_FALSE(visualNameView->HasEntity(e4));
  EXPECT_FALSE(visualNameView->HasEntity(e5));
  EXPECT_FALSE(visualNameView->HasEntity(e6));
  EXPECT_EQ(0u, visualNameView->Entities().size());
  EXPECT_EQ(0u, visualNameView->Entities().size());

  // remove components and then check view contents
  wrapper.RemoveComponent(e1, components::Name::typeId);
  wrapper.RemoveComponent(e2, components::Name::typeId);

  EXPECT_FALSE(nameView->HasEntity(e1));
  EXPECT_FALSE(nameView->HasEntity(e2));
  EXPECT_EQ(2u, nameView->Entities().size());
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e3));
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e4));
  EXPECT_EQ(1u, nameView->NewEntities().size());
  EXPECT_NE(nameView->NewEntities().end(), nameView->NewEntities().find(e4));

  EXPECT_FALSE(modelNameView->HasEntity(e1));
  EXPECT_FALSE(modelNameView->HasEntity(e2));
  EXPECT_EQ(0u, modelNameView->Entities().size());
  EXPECT_EQ(0u, modelNameView->NewEntities().size());

  // re-add components and then check view contents
  wrapper.ReAddComponent(e1, kNotNewEntity, components::Name("reAdded_e1"));
  wrapper.ReAddComponent(e2, kNewEntity, components::Name("reAdded_e2"));

  EXPECT_TRUE(nameView->HasEntity(e1));
  EXPECT_TRUE(nameView->HasEntity(e2));
  EXPECT_EQ(4u, nameView->Entities().size());
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e1));
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e2));
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e3));
  EXPECT_NE(nameView->Entities().end(), nameView->Entities().find(e4));
  EXPECT_EQ(2u, nameView->NewEntities().size());
  EXPECT_NE(nameView->NewEntities().end(), nameView->NewEntities().find(e2));
  EXPECT_NE(nameView->NewEntities().end(), nameView->NewEntities().find(e4));

  EXPECT_TRUE(modelNameView->HasEntity(e1));
  EXPECT_TRUE(modelNameView->HasEntity(e2));
  EXPECT_EQ(2u, modelNameView->Entities().size());
  EXPECT_NE(modelNameView->Entities().end(),
      modelNameView->Entities().find(e1));
  EXPECT_NE(modelNameView->Entities().end(),
      modelNameView->Entities().find(e2));
  EXPECT_EQ(1u, modelNameView->NewEntities().size());
  EXPECT_NE(modelNameView->NewEntities().end(),
      modelNameView->NewEntities().find(e2));

  // change component data and then check view contents
  auto wrapperCompPtr = wrapper.Component<components::Name>(e3);
  ASSERT_NE(nullptr, wrapperCompPtr);
  EXPECT_EQ(kOriginalNameCompE3, wrapperCompPtr->Data());
  ASSERT_TRUE(nameView->HasEntity(e3));
  auto viewCompPtr = std::get<components::Name *>(
      nameView->EntityComponentData(e3));
  EXPECT_EQ(wrapperCompPtr, viewCompPtr);
  auto viewCompConstPtr = std::get<const components::Name *>(
      nameView->EntityComponentConstData(e3));
  EXPECT_EQ(wrapperCompPtr, viewCompConstPtr);

  const std::string kNewNameCompE3 = "sensorEntity3_modified";
  EXPECT_NE(kOriginalNameCompE3, kNewNameCompE3);
  wrapper.ModifyComponent(e3, components::Name(kNewNameCompE3));
  wrapperCompPtr = wrapper.Component<components::Name>(e3);
  ASSERT_NE(nullptr, wrapperCompPtr);
  EXPECT_EQ(kNewNameCompE3, wrapperCompPtr->Data());
  ASSERT_TRUE(nameView->HasEntity(e3));
  viewCompPtr = std::get<components::Name *>(nameView->EntityComponentData(e3));
  EXPECT_EQ(wrapperCompPtr, viewCompPtr);
  viewCompConstPtr = std::get<const components::Name *>(
      nameView->EntityComponentConstData(e3));
  EXPECT_EQ(wrapperCompPtr, viewCompConstPtr);

  // remove entities and then check view contents
  EXPECT_TRUE(wrapper.RemoveEntity(e4));

  EXPECT_FALSE(sensorView->HasEntity(e4));
  EXPECT_EQ(1u, sensorView->Entities().size());
  EXPECT_EQ(sensorView->Entities().end(), sensorView->Entities().find(e4));
  EXPECT_EQ(0u, sensorView->NewEntities().size());

  EXPECT_FALSE(nameView->HasEntity(e4));
  EXPECT_EQ(3u, nameView->Entities().size());
  EXPECT_EQ(nameView->Entities().end(), nameView->Entities().find(e4));
  EXPECT_EQ(1u, nameView->NewEntities().size());
  EXPECT_EQ(nameView->NewEntities().end(), nameView->NewEntities().find(e4));

  EXPECT_FALSE(sensorNameView->HasEntity(e4));
  EXPECT_EQ(1u, sensorNameView->Entities().size());
  EXPECT_EQ(sensorNameView->Entities().end(),
      sensorNameView->Entities().find(e4));
  EXPECT_EQ(0u, sensorNameView->NewEntities().size());

  EXPECT_FALSE(visualNameView->HasEntity(e4));
  EXPECT_EQ(0u, visualNameView->Entities().size());
  EXPECT_EQ(0u, visualNameView->Entities().size());

  // add entities (and components) and then check view contents
  EXPECT_TRUE(wrapper.AddEntity(e4));

  // (at this point, e4 has no components attached to it, so this entity
  // shouldn't be in any view)
  numViewChecks = 0;
  for (const auto &view : views)
  {
    numViewChecks++;

    EXPECT_FALSE(view->HasEntity(e4));
    EXPECT_EQ(view->ToAddEntities().end(), view->ToAddEntities().find(e4));
    EXPECT_EQ(view->ToRemoveEntities().end(),
        view->ToRemoveEntities().find(e4));
    EXPECT_EQ(view->Entities().end(), view->Entities().find(e4));
    EXPECT_EQ(view->NewEntities().end(), view->NewEntities().find(e4));
  }
  EXPECT_EQ(numViewChecks, views.size());

  wrapper.AddNewComponent(e4, components::Sensor(), kNewEntity);
  wrapper.UpdateView(modelView);
  wrapper.UpdateView(sensorView);
  wrapper.UpdateView(visualView);
  wrapper.UpdateView(nameView);
  wrapper.UpdateView(modelNameView);
  wrapper.UpdateView(sensorNameView);
  wrapper.UpdateView(modelVisualView);
  wrapper.UpdateView(visualNameView);

  EXPECT_FALSE(modelView->HasEntity(e4));
  EXPECT_FALSE(visualView->HasEntity(e4));
  EXPECT_FALSE(nameView->HasEntity(e4));
  EXPECT_FALSE(modelNameView->HasEntity(e4));
  EXPECT_FALSE(sensorNameView->HasEntity(e4));
  EXPECT_FALSE(modelVisualView->HasEntity(e4));
  EXPECT_FALSE(visualNameView->HasEntity(e4));

  EXPECT_TRUE(sensorView->HasEntity(e4));
  EXPECT_NE(sensorView->Entities().end(), sensorView->Entities().find(e4));
  EXPECT_NE(sensorView->NewEntities().end(),
      sensorView->NewEntities().find(e4));
}
