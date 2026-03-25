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
#ifndef GZ_SIM_DETAIL_ENTITYCOMPONENTMANAGER_HH_
#define GZ_SIM_DETAIL_ENTITYCOMPONENTMANAGER_HH_

#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <gz/math/Helpers.hh>

#include "gz/sim/EntityComponentManager.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
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
#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
#endif
    enum
    {
      // False positive codecheck "Using C-style cast"
      value = !std::is_same<decltype(*(T*)(0) == *(T*)(0)), TestEqualityOperator>::value // NOLINT
    };
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
  };
}

//////////////////////////////////////////////////
/// \brief Helper function to compare two objects of the same type using its
/// equality operator.
/// If `DataType` doesn't have an equality operator defined, it will return
/// false.
/// For doubles, `gz::math::equal` will be used.
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
  if (!this->HasEntity(_entity))
    return nullptr;
  this->registry.emplace_or_replace<ComponentTypeT>(_entity, _data);
  this->SetChanged(_entity, ComponentTypeT::typeId, ComponentState::OneTimeChange);
  this->MarkComponentAsRemoved(_entity, ComponentTypeT::typeId, false);
  return &this->registry.get<ComponentTypeT>(_entity);
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
const ComponentTypeT *EntityComponentManager::Component(
    const Entity _entity) const
{
  return this->registry.try_get<const ComponentTypeT>(_entity);
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::Component(const Entity _entity)
{
  return this->registry.try_get<ComponentTypeT>(_entity);
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EntityComponentManager::ComponentDefault(Entity _entity,
    const typename ComponentTypeT::Type &_default)
{
  // TODO(luca) a get or emplace?
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
  // TODO(luca) registry operation for more efficiency?
  auto comp = this->Component<ComponentTypeT>(_entity);

  if (nullptr == comp)
  {
    this->CreateComponent(_entity, ComponentTypeT(_data));
    return true;
  }

  return comp->SetData(_data, CompareData<typename ComponentTypeT::Type>);
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
Entity EntityComponentManager::EntityByComponents(
    const ComponentTypeTs &..._desiredComponents) const
{
  auto view = this->registry.template view<const ComponentTypeTs...>();
  for (auto e : view)
  {
    bool match = ((*this->registry.template try_get<ComponentTypeTs>(e) == _desiredComponents) && ...);
    if (match)
      return e;
  }

  return kNullEntity;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
std::vector<Entity> EntityComponentManager::EntitiesByComponents(
    const ComponentTypeTs &..._desiredComponents) const
{
  std::vector<Entity> result;
  auto view = this->registry.template view<const ComponentTypeTs...>();
  for (auto e : view)
  {
    bool match = ((view.template get<ComponentTypeTs>(e) == _desiredComponents) && ...);
    if (match)
      result.push_back(e);
  }

  return result;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
std::vector<Entity> EntityComponentManager::ChildrenByComponents(Entity _parent,
     const ComponentTypeTs &..._desiredComponents) const
{
  std::vector<Entity> result;
  const auto& children = this->registry.template get<Children>(_parent);

  for (const Entity e : children.data)
  {
    bool match = (this->registry.template all_of<ComponentTypeTs>(e) && ...);
    if (match)
    {
      match = ((*this->registry.template try_get<ComponentTypeTs>(e) == _desiredComponents) && ...);
    }

    if (match)
      result.push_back(e);
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
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::EachNoCache(Func &&_f) const
{
  // Caching doesn't exist anymore so this is functionally equivalent
  // TODO(luca) deprecate
  this->Each<ComponentTypeTs...>(std::forward<Func>(_f));
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::EachNoCache(Func &&_f)
{
  this->Each<ComponentTypeTs...>(std::forward<Func>(_f));
}

//////////////////////////////////////////////////
namespace detail
{
  /// \brief Base class for enqueuing group creation.
  class GroupQueuer
  {
    public: virtual ~GroupQueuer() = default;
    public: virtual void CreateGroup(entt::basic_registry<Entity> &_registry) = 0;
  };

  /// \brief Implementation of GroupQueuer for a specific set of components.
  template<typename ...ComponentTypeTs>
  class GroupQueuerImpl : public GroupQueuer
  {
    public: void CreateGroup(entt::basic_registry<Entity> &_registry) override
    {
      _registry.template group<>(entt::get<ComponentTypeTs...>);
    }
  };
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::Each(Func &&_f) const
{
  if (auto group = this->registry.template group_if_exists<>(
      entt::get<const ComponentTypeTs...>); group)
  {
    for (const auto entity : group)
    {
      if (!_f(entity, std::addressof(group.template get<const ComponentTypeTs>(entity))...))
        break;
    }
    return;
  }

  // Enqueue group creation for next iteration
  this->EnqueueGroup({ComponentTypeTs::typeId...},
      std::make_unique<detail::GroupQueuerImpl<std::remove_const_t<ComponentTypeTs>...>>());

  auto view = this->registry.template view<const ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const auto entity : view)
  {
    if (!_f(entity, std::addressof(view.template get<const ComponentTypeTs>(entity))...))
      break;
  }
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::Each(Func &&_f)
{
  auto view = this->registry.template group<>(entt::get<ComponentTypeTs...>);

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const auto entity : view)
  {
    if (!_f(entity, std::addressof(view.template get<ComponentTypeTs>(entity))...))
      break;
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
template <typename... ComponentTypeTs, typename Func>
void EntityComponentManager::EachNew(Func &&_f)
{
  // TODO(luca) make all of these non owning groups for perf
  auto view = this->registry.template group<>(entt::get<NewEntity, ComponentTypeTs...>);

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const auto entity : view)
  {
    if (!_f(entity, (std::addressof(view.template get<ComponentTypeTs>(entity)))...))
      break;
  }
}

//////////////////////////////////////////////////
template <typename... ComponentTypeTs, typename Func>
void EntityComponentManager::EachNew(Func &&_f) const
{
  // TODO(luca) make all of these non owning groups for perf
  auto view = this->registry.template view<const NewEntity, const ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const auto entity : view)
  {
    if (!_f(entity, std::addressof(view.template get<const ComponentTypeTs>(entity))...))
      break;
  }
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs, typename Func>
void EntityComponentManager::EachRemoved(Func &&_f) const
{
  // TODO(luca) make all of these non owning groups for perf
  auto view = this->registry.template view<const RemoveEntity, const ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const auto entity : view)
  {
    if (!_f(entity, std::addressof(view.template get<const ComponentTypeTs>(entity))...))
      break;
  }
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
