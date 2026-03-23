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
#ifndef GZ_SIM_DETAIL_ENTTCOMPONENTMANAGER_HH_
#define GZ_SIM_DETAIL_ENTTCOMPONENTMANAGER_HH_

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

#include "gz/sim/EnttComponentManager.hh"

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
ComponentTypeT *EnttComponentManager::CreateComponent(const Entity _entity,
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
const ComponentTypeT *EnttComponentManager::Component(
    const Entity _entity) const
{
  return this->registry.try_get<const ComponentTypeT>(_entity);
  /*
  // Get a unique identifier to the component type
  const ComponentTypeId typeId = ComponentTypeT::typeId;

  return static_cast<const ComponentTypeT *>(
      this->ComponentImplementation(_entity, typeId));
      */
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EnttComponentManager::Component(const Entity _entity)
{
  return this->registry.try_get<ComponentTypeT>(_entity);
  // Get a unique identifier to the component type
  /*`
  const ComponentTypeId typeId = ComponentTypeT::typeId;

  return static_cast<ComponentTypeT *>(
      this->ComponentImplementation(_entity, typeId));
      */
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
ComponentTypeT *EnttComponentManager::ComponentDefault(Entity _entity,
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
    EnttComponentManager::ComponentData(const Entity _entity) const
{
  auto comp = this->Component<ComponentTypeT>(_entity);
  if (!comp)
    return std::nullopt;

  return std::make_optional(comp->Data());
}

//////////////////////////////////////////////////
template<typename ComponentTypeT>
bool EnttComponentManager::SetComponentData(const Entity _entity,
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
Entity EnttComponentManager::EntityByComponents(
    const ComponentTypeTs &..._desiredComponents) const
{
  for (auto entity : this->registry.template storage<Entity>()->each())
  {
    const Entity e = std::get<0>(entity);
    bool match = ((this->registry.template all_of<ComponentTypeTs>(e) &&
                   *this->registry.template try_get<ComponentTypeTs>(e) == _desiredComponents) && ...);
    if (match)
      return e;
  }

  return kNullEntity;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
std::vector<Entity> EnttComponentManager::EntitiesByComponents(
    const ComponentTypeTs &..._desiredComponents) const
{
  std::vector<Entity> result;
  for (auto entity : this->registry.template storage<Entity>()->each())
  {
    const Entity e = std::get<0>(entity);
    bool match = ((this->registry.template all_of<ComponentTypeTs>(e) &&
                   *this->registry.template try_get<ComponentTypeTs>(e) == _desiredComponents) && ...);
    if (match)
      result.push_back(e);
  }

  return result;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
std::vector<Entity> EnttComponentManager::ChildrenByComponents(Entity _parent,
     const ComponentTypeTs &..._desiredComponents) const
{
  std::vector<Entity> result;
  const auto *children = this->registry.template try_get<Children>(_parent);
  if (!children)
    return result;

  for (const Entity e : children->data)
  {
    bool match = ((this->registry.template all_of<ComponentTypeTs>(e) &&
                   *this->registry.template try_get<ComponentTypeTs>(e) == _desiredComponents) && ...);
    if (match)
      result.push_back(e);
  }

  return result;
}

//////////////////////////////////////////////////
template <typename T>
struct EnttComponentManager::identity  // NOLINT
{
  using type = T;
};

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EnttComponentManager::EachNoCache(typename identity<std::function<
    bool(const Entity &_entity, const ComponentTypeTs *...)>>::type _f) const
{
  this->Each<ComponentTypeTs...>(_f);
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EnttComponentManager::EachNoCache(typename identity<std::function<
    bool(const Entity &_entity, ComponentTypeTs *...)>>::type _f)
{
  this->Each<ComponentTypeTs...>(_f);
}

namespace detail
{
/// \brief Helper template to call a callback function with each of the
/// components in the _data vector expanded as arguments to the callback
/// function.
/// \tparam ComponentTypeTs The actual types of each of the components.
/// \tparam FuncT The type of the callback function.
/// \tparam BaseComponentT Either "BaseComponent" or "const BaseComponent"
/// \tparam Is Index sequence that will be used to iterate through the vector
/// _data.
/// \param[in] _f The callback function
/// \param[in] _entity The entity associated with the components.
/// \param[in] _data A vector of component pointers that will be expanded to
/// become the arguments of the callback function _f.
/// \return The value of return by the function _f.
template <typename... ComponentTypeTs, typename FuncT, typename BaseComponentT,
          std::size_t... Is>
constexpr bool applyFunctionImpl(const FuncT &_f, const Entity &_entity,
                       const std::vector<BaseComponentT *> &_data,
                       std::index_sequence<Is...>)
{
  return _f(_entity, static_cast<ComponentTypeTs *>(_data[Is])...);
}

/// \brief Helper template to call a callback function with each of the
/// components in the _data vector expanded as arguments to the callback
/// function.
/// \tparam ComponentTypeTs The actual types of each of the components.
/// \tparam FuncT The type of the callback function.
/// \tparam BaseComponentT Either "BaseComponent" or "const BaseComponent"
/// \param[in] _f The callback function
/// \param[in] _entity The entity associated with the components.
/// \param[in] _data A vector of component pointers that will be expanded to
/// become the arguments of the callback function _f.
/// \return The value of return by the function _f.
template <typename... ComponentTypeTs, typename FuncT, typename BaseComponentT>
constexpr bool applyFunction(const FuncT &_f, const Entity &_entity,
                   const std::vector<BaseComponentT *> &_data)
{
  return applyFunctionImpl<ComponentTypeTs...>(
      _f, _entity, _data, std::index_sequence_for<ComponentTypeTs...>{});
}
}  // namespace detail

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EnttComponentManager::Each(typename identity<std::function<
    bool(const Entity &_entity, const ComponentTypeTs *...)>>::type _f) const
{
  // TODO(luca) make all of these non owning groups for perf
  auto view = this->registry.view<const ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const auto pack : view.each())
  {
    bool done = false;
    std::apply([&](const Entity _entity, const auto &... _comps) {
        if (!_f(_entity, &_comps...))
            done = true;
    }, pack);
    if (done) break;
  }
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EnttComponentManager::Each(typename identity<std::function<
    bool(const Entity &_entity, ComponentTypeTs *...)>>::type _f)
{
  auto view = this->registry.view<ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (auto pack : view.each())
  {
    bool done = false;
    std::apply([&](const Entity _entity, auto &... _comps) {
        if (!_f(_entity, &_comps...))
            done = true;
    }, pack);
    if (done) break;
  }
}

//////////////////////////////////////////////////
template <class Function, class... ComponentTypeTs>
void EnttComponentManager::ForEach(Function _f,
    const ComponentTypeTs &... _components)
{
  (_f(_components), ...);
}

// Note check on ENTT_DISABLE_ETO, might need to do
//////////////////////////////////////////////////
template <typename... ComponentTypeTs>
void EnttComponentManager::EachNew(typename identity<std::function<
    bool(const Entity &_entity, ComponentTypeTs *...)>>::type _f)
{
  // TODO(luca) make all of these non owning groups for perf
  auto view = this->registry.view<NewEntity, ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const auto pack : view.each())
  {
    bool done = false;
    std::apply([&](const Entity _entity, NewEntity&, auto &... _comps) {
        if (!_f(_entity, &_comps...))
            done = true;
    }, pack);
    if (done) break;
  }
}

//////////////////////////////////////////////////
template <typename... ComponentTypeTs>
void EnttComponentManager::EachNew(typename identity<std::function<
    bool(const Entity &_entity, const ComponentTypeTs *...)>>::type _f) const
{
  // TODO(luca) make all of these non owning groups for perf
  auto view = this->registry.view<const NewEntity, const ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const auto pack : view.each())
  {
    bool done = false;
    std::apply([&](const Entity _entity, const NewEntity&, const auto &... _comps) {
        if (!_f(_entity, &_comps...))
            done = true;
    }, pack);
    if (done) break;
  }
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void EnttComponentManager::EachRemoved(typename identity<std::function<
    bool(const Entity &_entity, const ComponentTypeTs *...)>>::type _f) const
{
  // TODO(luca) make all of these non owning groups for perf
  auto view = this->registry.view<const RemoveEntity, const ComponentTypeTs...>();

  // Iterate over the entities in the view, and invoke the callback
  // function.
  for (const auto pack : view.each())
  {
    bool done = false;
    std::apply([&](const Entity _entity, const RemoveEntity&, const auto &... _comps) {
        if (!_f(_entity, &_comps...))
            done = true;
    }, pack);
    if (done) break;
  }
}

/*
//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
detail::View *EnttComponentManager::FindView() const
{
  auto viewKey = std::vector<ComponentTypeId>{ComponentTypeTs::typeId...};

  auto baseViewMutexPair = this->FindView(viewKey);
  auto baseViewPtr = baseViewMutexPair.first;
  if (nullptr != baseViewPtr)
  {
    auto view = static_cast<detail::View*>(baseViewPtr);

    std::unique_ptr<std::lock_guard<std::mutex>> viewLock;
    if (this->LockAddingEntitiesToViews())
    {
      // lock the mutex unique to this view in order to prevent multiple threads
      // from concurrently reading/modifying the view's toAddEntities data
      // (for example, this is useful in system PostUpdates since they are run
      // in parallel)
      auto mutexPtr = baseViewMutexPair.second;
      if (nullptr == mutexPtr)
      {
        gzerr << "Internal error: requested to lock a view, but no mutex "
          << "exists for this view. This should never happen!" << std::endl;
        return view;
      }
      viewLock = std::make_unique<std::lock_guard<std::mutex>>(*mutexPtr);
    }

    // add any new entities to the view before using it
    for (const auto &[entity, isNew] : view->ToAddEntities())
    {
      view->AddEntityWithConstComps(entity, isNew,
          this->Component<ComponentTypeTs>(entity)...);
      view->AddEntityWithComps(entity, isNew,
          const_cast<EnttComponentManager*>(this)->Component<ComponentTypeTs>(
            entity)...);
    }
    view->ClearToAddEntities();

    return view;
  }

  // create a new view if one wasn't found
  detail::View view(std::set<ComponentTypeId>{ComponentTypeTs::typeId...});

  for (const auto &vertex : this->Entities().Vertices())
  {
    Entity entity = vertex.first;

    // only add entities to the view that have all of the components in viewKey
    if (!this->EntityMatches(entity, view.ComponentTypes()))
      continue;

    view.AddEntityWithConstComps(entity, this->IsNewEntity(entity),
        this->Component<ComponentTypeTs>(entity)...);
    view.AddEntityWithComps(entity, this->IsNewEntity(entity),
        const_cast<EnttComponentManager*>(this)->Component<ComponentTypeTs>(
            entity)...);
    if (this->IsMarkedForRemoval(entity))
      view.MarkEntityToRemove(entity);
  }

  baseViewPtr = this->AddView(viewKey,
      std::make_unique<detail::View>(std::move(view)));
  return static_cast<detail::View *>(baseViewPtr);
}

*/
//////////////////////////////////////////////////
template<typename ComponentTypeT>
bool EnttComponentManager::RemoveComponent(Entity _entity)
{
  const auto typeId = ComponentTypeT::typeId;
  return this->RemoveComponent(_entity, typeId);
}
}
}
}

#endif
