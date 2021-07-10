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
#ifndef IGNITION_GAZEBO_DETAIL_ENTITYCOMPONENTMANAGER_HH_
#define IGNITION_GAZEBO_DETAIL_ENTITYCOMPONENTMANAGER_HH_

#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <ignition/math/Helpers.hh>

#include "ignition/gazebo/EntityComponentManager.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
//////////////////////////////////////////////////
namespace traits
{
  /// \brief Helper struct to determine if an equality operator is present.
  struct TestEqualityOperator
  {
  };
  template<typename T>
  TestEqualityOperator operator == (const T&, const T&);

  /// \brief Type trait that determines if an operator== is defined for `T`.
  template<typename T>
  struct HasEqualityOperator
  {
    enum
    {
      // False positive codecheck "Using C-style cast"
      value = !std::is_same<decltype(*(T*)(0) == *(T*)(0)), TestEqualityOperator>::value // NOLINT
    };
  };
}

//////////////////////////////////////////////////
/// \brief Helper function to compare two objects of the same type using its
/// equality operator.
/// If `DataType` doesn't have an equality operator defined, it will return
/// false.
/// For doubles, `ignition::math::equal` will be used.
template<typename DataType>
auto CompareData = [](const DataType &_a, const DataType &_b) -> bool
{
  // cppcheck-suppress syntaxError
  if constexpr (std::is_same<DataType, double>::value)
  {
    return math::equal(_a, _b);
  }
  else if constexpr (traits::HasEqualityOperator<DataType>::value)
  {
    return _a == _b;
  }

  return false;
};

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::CreateComponent(const Entity _entity,
            const ComponentTypeT &_data)
{
  auto updateData = this->CreateComponentImplementation(_entity,
      ComponentTypeT::typeId, &_data);
  auto comp = this->Component<ComponentTypeT>(_entity);
  if (updateData)
  {
    if (!comp)
    {
      ignerr << "Internal error. Failure to create a component of type "
        << ComponentTypeT::typeId << " for entity " << _entity
        << ". This should never happen!\n";
      return comp;
    }
    *comp = _data;
    comp->removed = false;
  }
  return comp;
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
const ComponentTypeT *EntityComponentManager::Component(
    const Entity _entity) const
{
  // Get a unique identifier to the component type
  const ComponentTypeId typeId = ComponentTypeT::typeId;

  return static_cast<const ComponentTypeT *>(
      this->ComponentImplementation(_entity, typeId));
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::Component(const Entity _entity)
{
  // Get a unique identifier to the component type
  const ComponentTypeId typeId = ComponentTypeT::typeId;

  return static_cast<ComponentTypeT *>(
      this->ComponentImplementation(_entity, typeId));
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
const ComponentTypeT *EntityComponentManager::Component(
    const ComponentKey &_key) const
{
  return static_cast<const ComponentTypeT *>(
      this->ComponentImplementation(_key.second, _key.first));
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::Component(const ComponentKey &_key)
{
  return static_cast<ComponentTypeT *>(
      this->ComponentImplementation(_key.second, _key.first));
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::ComponentDefault(Entity _entity,
    const typename ComponentTypeT::Type &_default)
{
  auto comp = this->Component<ComponentTypeT>(_entity);
  if (!comp)
  {
    this->CreateComponent(_entity, ComponentTypeT(_default));
    comp = this->Component<ComponentTypeT>(_entity);
  }
  return comp;
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
std::optional<typename ComponentTypeT::Type>
    EntityComponentManager::ComponentData(const Entity _entity) const
{
  auto comp = this->Component<ComponentTypeT>(_entity);
  if (!comp)
    return std::nullopt;

  return std::make_optional(comp->Data());
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
bool EntityComponentManager::SetComponentData(const Entity _entity,
    const typename ComponentTypeT::Type &_data)
{
  auto comp = this->Component<ComponentTypeT>(_entity);

  if (nullptr == comp)
  {
    this->CreateComponent(_entity, ComponentTypeT(_data));
    return true;
  }

  auto changed = comp->SetData(_data,
      CompareData<typename ComponentTypeT::Type>);
  comp->removed = false;
  return changed;
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
const ComponentTypeT *EntityComponentManager::First() const
{
  ignwarn << "EntityComponentManager::First is now deprecated and will always "
    << "return nullptr.\n";
  return nullptr;
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::First()
{
  ignwarn << "EntityComponentManager::First is now deprecated and will always "
    << "return nullptr.\n";
  return nullptr;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
Entity EntityComponentManager::EntityByComponents(
    const ComponentTypeTs &..._desiredComponents) const
{
  // Get all entities which have components of the desired types
  const auto &view = this->FindView<ComponentTypeTs...>();

  // Iterate over entities
  Entity result{kNullEntity};
  for (const Entity entity : view->Entities())
  {
    bool different{false};

    // Iterate over desired components, comparing each of them to the
    // equivalent component in the entity.
    ForEach([&](const auto &_desiredComponent)
    {
      auto entityComponent = this->Component<
          std::remove_cv_t<std::remove_reference_t<
              decltype(_desiredComponent)>>>(entity);

      if (*entityComponent != _desiredComponent)
      {
        different = true;
      }
    }, _desiredComponents...);

    if (!different)
    {
      result = entity;
      break;
    }
  }

  return result;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
std::vector<Entity> EntityComponentManager::EntitiesByComponents(
    const ComponentTypeTs &..._desiredComponents) const
{
  // Get all entities which have components of the desired types
  const auto &view = this->FindView<ComponentTypeTs...>();

  // Iterate over entities
  std::vector<Entity> result;
  for (const Entity entity : view->Entities())
  {
    bool different{false};

    // Iterate over desired components, comparing each of them to the
    // equivalent component in the entity.
    ForEach([&](const auto &_desiredComponent)
    {
      auto entityComponent = this->Component<
          std::remove_cv_t<std::remove_reference_t<
              decltype(_desiredComponent)>>>(entity);

      if (*entityComponent != _desiredComponent)
      {
        different = true;
      }
    }, _desiredComponents...);

    if (!different)
    {
      result.push_back(entity);
    }
  }

  return result;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
std::vector<Entity> EntityComponentManager::ChildrenByComponents(Entity _parent,
     const ComponentTypeTs &..._desiredComponents) const
{
  // Get all entities which have components of the desired types
  const auto &view = this->FindView<ComponentTypeTs...>();

  // Get all entities which are immediate children of the given parent
  auto children = this->Entities().AdjacentsFrom(_parent);

  // Iterate over entities
  std::vector<Entity> result;
  for (const Entity entity : view->Entities())
  {
    if (children.find(entity) == children.end())
    {
      continue;
    }

    // Iterate over desired components, comparing each of them to the
    // equivalent component in the entity.
    bool different{false};
    ForEach([&](const auto &_desiredComponent)
    {
      auto entityComponent = this->Component<
          std::remove_cv_t<std::remove_reference_t<
              decltype(_desiredComponent)>>>(entity);

      if (*entityComponent != _desiredComponent)
      {
        different = true;
      }
    }, _desiredComponents...);

    if (!different)
    {
      result.push_back(entity);
    }
  }

  return result;
}

//////////////////////////////////////////////////
template <typename T>
struct EntityComponentManager::identity  // NOLINT
{
  using type = T;
};

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EntityComponentManager::EachNoCache(typename identity<std::function<
    bool(const Entity &_entity, const ComponentTypeTs *...)>>::type _f) const
{
  for (const auto &vertex : this->Entities().Vertices())
  {
    Entity entity = vertex.first;
    auto types = std::set<ComponentTypeId>{ComponentTypeTs::typeId...};

    if (this->EntityMatches(entity, types))
    {
      if (!_f(entity,
              this->Component<ComponentTypeTs>(entity)...))
      {
        break;
      }
    }
  }
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EntityComponentManager::EachNoCache(typename identity<std::function<
    bool(const Entity &_entity, ComponentTypeTs *...)>>::type _f)
{
  for (const auto &vertex : this->Entities().Vertices())
  {
    Entity entity = vertex.first;
    auto types = std::set<ComponentTypeId>{ComponentTypeTs::typeId...};

    if (this->EntityMatches(entity, types))
    {
      if (!_f(entity,
              this->Component<ComponentTypeTs>(entity)...))
      {
        break;
      }
    }
  }
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EntityComponentManager::Each(typename identity<std::function<
    bool(const Entity &_entity, const ComponentTypeTs *...)>>::type _f) const
{
  // Get the view. This will create a new view if one does not already
  // exist.
  auto view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const Entity entity : view->Entities())
  {
    if (!std::apply(_f, view->EntityComponentConstData(entity)))
    {
      break;
    }
  }
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EntityComponentManager::Each(typename identity<std::function<
    bool(const Entity &_entity, ComponentTypeTs *...)>>::type _f)
{
  // Get the view. This will create a new view if one does not already
  // exist.
  auto view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const Entity entity : view->Entities())
  {
    if (!std::apply(_f, view->EntityComponentData(entity)))
    {
      break;
    }
  }
}

//////////////////////////////////////////////////
template <class Function, class... ComponentTypeTs>
void EntityComponentManager::ForEach(Function _f,
    const ComponentTypeTs &... _components)
{
  (_f(_components), ...);
}

//////////////////////////////////////////////////
template <typename... ComponentTypeTs>
void EntityComponentManager::EachNew(typename identity<std::function<
    bool(const Entity &_entity, ComponentTypeTs *...)>>::type _f)
{
  // Get the view. This will create a new view if one does not already
  // exist.
  auto view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view and in the newly created
  // entities list, and invoke the callback
  // function.
  for (const Entity entity : view->NewEntities())
  {
    if (!std::apply(_f, view->EntityComponentData(entity)))
    {
      break;
    }
  }
}

//////////////////////////////////////////////////
template <typename... ComponentTypeTs>
void EntityComponentManager::EachNew(typename identity<std::function<
    bool(const Entity &_entity, const ComponentTypeTs *...)>>::type _f) const
{
  // Get the view. This will create a new view if one does not already
  // exist.
  auto view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view and in the newly created
  // entities list, and invoke the callback
  // function.
  for (const Entity entity : view->NewEntities())
  {
    if (!std::apply(_f, view->EntityComponentConstData(entity)))
    {
      break;
    }
  }
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EntityComponentManager::EachRemoved(typename identity<std::function<
    bool(const Entity &_entity, const ComponentTypeTs *...)>>::type _f) const
{
  // Get the view. This will create a new view if one does not already
  // exist.
  auto view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view and in the newly created
  // entities list, and invoke the callback
  // function.
  for (const Entity entity : view->ToRemoveEntities())
  {
    if (!std::apply(_f, view->EntityComponentConstData(entity)))
    {
      break;
    }
  }
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
detail::View<ComponentTypeTs...> *EntityComponentManager::FindView() const
{
  auto viewKey = std::vector<ComponentTypeId>{ComponentTypeTs::typeId...};

  auto baseViewPtr = this->FindView(viewKey);
  if (nullptr != baseViewPtr)
  {
    auto view = static_cast<detail::View<ComponentTypeTs...>*>(baseViewPtr);

    // add any new entities to the view before using it
    for (const auto &[entity, isNew] : view->ToAddEntities())
    {
      view->AddEntityWithConstComps(entity, isNew,
          this->Component<ComponentTypeTs>(entity)...);
      view->AddEntityWithComps(entity, isNew,
          const_cast<EntityComponentManager*>(this)->Component<ComponentTypeTs>(
            entity)...);
    }
    view->ClearToAddEntities();

    return view;
  }

  // create a new view if one wasn't found
  detail::View<ComponentTypeTs...> view;

  for (const auto &vertex : this->Entities().Vertices())
  {
    Entity entity = vertex.first;

    // only add entities to the view that have all of the components in viewKey
    if (!this->EntityMatches(entity, view.ComponentTypes()))
      continue;

    view.AddEntityWithConstComps(entity, this->IsNewEntity(entity),
        this->Component<ComponentTypeTs>(entity)...);
    view.AddEntityWithComps(entity, this->IsNewEntity(entity),
        const_cast<EntityComponentManager*>(this)->Component<ComponentTypeTs>(
            entity)...);
    if (this->IsMarkedForRemoval(entity))
      view.MarkEntityToRemove(entity);
  }

  baseViewPtr = this->AddView(viewKey,
      std::make_unique<detail::View<ComponentTypeTs...>>(view));
  return static_cast<detail::View<ComponentTypeTs...>*>(baseViewPtr);
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
bool EntityComponentManager::RemoveComponent(Entity _entity)
{
  const auto typeId = ComponentTypeT::typeId;
  return this->RemoveComponent(_entity, typeId);
}
}
}
}

#endif
