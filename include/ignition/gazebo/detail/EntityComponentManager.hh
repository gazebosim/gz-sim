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
#include <set>
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
ComponentKey EntityComponentManager::CreateComponent(const Entity _entity,
            const ComponentTypeT &_data)
{
  return this->CreateComponentImplementation(_entity, ComponentTypeT::typeId,
      &_data);
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
      this->ComponentImplementation(_key));
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::Component(const ComponentKey &_key)
{
  return static_cast<ComponentTypeT *>(
      this->ComponentImplementation(_key));
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

  return comp->SetData(_data, CompareData<typename ComponentTypeT::Type>);
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
const ComponentTypeT *EntityComponentManager::First() const
{
  return static_cast<const ComponentTypeT *>(
      this->First(ComponentTypeT::typeId));
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::First()
{
  return static_cast<ComponentTypeT *>(
      this->First(ComponentTypeT::typeId));
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
  for (const Entity entity : view.entities)
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
  for (const Entity entity : view.entities)
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
  for (const Entity entity : view.entities)
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
  detail::View &view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const Entity entity : view.entities)
  {
    if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
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
  detail::View &view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const Entity entity : view.entities)
  {
    if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
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
  detail::View &view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view and in the newly created
  // entities list, and invoke the callback
  // function.
  for (const Entity entity : view.newEntities)
  {
    if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
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
  detail::View &view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view and in the newly created
  // entities list, and invoke the callback
  // function.
  for (const Entity entity : view.newEntities)
  {
    if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
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
  detail::View &view = this->FindView<ComponentTypeTs...>();

  // Iterate over the entities in the view and in the newly created
  // entities list, and invoke the callback
  // function.
  for (const Entity entity : view.toRemoveEntities)
  {
    if (!_f(entity, view.Component<ComponentTypeTs>(entity, this)...))
    {
      break;
    }
  }
}

//////////////////////////////////////////////////
template<typename FirstComponent,
         typename ...RemainingComponents,
         typename std::enable_if<
           sizeof...(RemainingComponents) == 0, int>::type>
void EntityComponentManager::AddComponentsToView(detail::View &_view,
    const Entity _entity) const
{
  const ComponentTypeId typeId = FirstComponent::typeId;

  const ComponentId compId =
      this->EntityComponentIdFromType(_entity, typeId);
  if (compId >= 0)
  {
    // Add the component to the view.
    _view.AddComponent(_entity, typeId, compId);
  }
  else
  {
    ignerr << "Entity[" << _entity << "] has no component of type["
      << typeId << "]. This should never happen.\n";
  }
}

//////////////////////////////////////////////////
template<typename FirstComponent,
         typename ...RemainingComponents,
         typename std::enable_if<
           sizeof...(RemainingComponents) != 0, int>::type>
void EntityComponentManager::AddComponentsToView(detail::View &_view,
    const Entity _entity) const
{
  const ComponentTypeId typeId = FirstComponent::typeId;
  const ComponentId compId =
      this->EntityComponentIdFromType(_entity, typeId);
  if (compId >= 0)
  {
    // Add the component to the view.
    _view.AddComponent(_entity, typeId, compId);
  }
  else
  {
    ignerr << "Entity[" << _entity << "] has no component of type["
      << typeId << "]. This should never happen.\n";
  }

  // Add the remaining components to the view.
  this->AddComponentsToView<RemainingComponents...>(_view, _entity);
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
detail::View &EntityComponentManager::FindView() const
{
  auto types = std::set<ComponentTypeId>{ComponentTypeTs::typeId...};

  std::map<detail::ComponentTypeKey, detail::View>::iterator viewIter;

  // Find the view. If the view doesn't exist, then create a new view.
  if (!this->FindView(types, viewIter))
  {
    detail::View view;
    // Add all the entities that match the component types to the
    // view.
    for (const auto &vertex : this->Entities().Vertices())
    {
      Entity entity = vertex.first;
      if (this->EntityMatches(entity, types))
      {
        view.AddEntity(entity, this->IsNewEntity(entity));
        // If there is a request to delete this entity, update the view as
        // well
        if (this->IsMarkedForRemoval(entity))
        {
          view.AddEntityToRemoved(entity);
        }

        // Store pointers to all the components. This recursively adds
        // all the ComponentTypeTs that belong to the entity to the view.
        this->AddComponentsToView<ComponentTypeTs...>(view, entity);
      }
    }

    // Store the view.
    return this->AddView(types, std::move(view))->second;
  }

  return viewIter->second;
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
