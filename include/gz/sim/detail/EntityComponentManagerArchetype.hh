/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 *
 * Archetype-backed template bodies for gz::sim::EntityComponentManager.
 *
 * Paired with src/EntityComponentManagerArchetype.cc. Selected over
 * the legacy include/gz/sim/detail/EntityComponentManager.hh at compile
 * time when GZ_SIM_USE_ARCHETYPE_ECM is defined. See
 * docs/design/phase-0b-ecm-integration.md §3.1 for rationale.
 *
 * The templates here are NOT a line-for-line port of the legacy detail
 * header — they implement the same public API against gz::sim::ecs::World
 * directly, without any detail::View machinery. Simple type-erased
 * forwards (Component<T>, CreateComponent<T>, SetComponentData<T>, etc.)
 * share the legacy bodies verbatim; only the view-driven family (Each,
 * EachNew, EachRemoved, EntityByComponents, …) is rewritten.
 */
#ifndef GZ_SIM_DETAIL_ENTITYCOMPONENTMANAGERARCHETYPE_HH_
#define GZ_SIM_DETAIL_ENTITYCOMPONENTMANAGERARCHETYPE_HH_

// Phase 0b: mirror-image of the guard in
// detail/EntityComponentManager.hh — the auto-globbed gz/sim.hh
// umbrella includes both headers unconditionally; each is only
// active under its matching backend flag. See design §3.1.
#if defined(GZ_SIM_USE_ARCHETYPE_ECM)

#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <type_traits>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Helpers.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/ecs/World.hh"

namespace gz
{
namespace sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE
{
  //
  // Shared bits copied from the legacy detail header. These utilities
  // are backend-agnostic; duplicating them here keeps the archetype
  // header self-contained (no cross-include to the legacy detail).
  //
  namespace traits
  {
    struct TestEqualityOperator {};
    template<typename T>
    TestEqualityOperator operator == (const T&, const T&);

    template<typename T>
    struct HasEqualityOperator
    {
#if !defined(_MSC_VER)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"
#endif
      enum
      {
        value = !std::is_same<
            decltype(*(T*)(0) == *(T*)(0)),
            TestEqualityOperator>::value
      };
#if !defined(_MSC_VER)
#pragma GCC diagnostic pop
#endif
    };
  }

  template<typename DataType>
  auto CompareData = [](const DataType &_a, const DataType &_b) -> bool
  {
    if constexpr (std::is_same<DataType, double>::value)
      return math::equal(_a, _b);
    else if constexpr (traits::HasEqualityOperator<DataType>::value)
      return _a == _b;
    return false;
  };

  //----------------------------------------------------------
  // Simple forwarders — identical to the legacy detail header. These
  // go through the type-erased *Implementation() helpers whose bodies
  // live in src/EntityComponentManagerArchetype.cc.

  template<typename ComponentTypeT>
  ComponentTypeT *EntityComponentManager::CreateComponent(
      const Entity _entity, const ComponentTypeT &_data)
  {
    auto updateData = this->CreateComponentImplementation(_entity,
        ComponentTypeT::typeId, &_data);
    auto comp = this->Component<ComponentTypeT>(_entity);
    if (updateData)
    {
      if (!comp)
      {
        gzerr << "Internal error creating component of type "
              << ComponentTypeT::typeId << " for entity "
              << _entity << "\n";
        return comp;
      }
      *comp = _data;
    }
    return comp;
  }

  template<typename ComponentTypeT>
  const ComponentTypeT *EntityComponentManager::Component(
      const Entity _entity) const
  {
    return static_cast<const ComponentTypeT *>(
        this->ComponentImplementation(_entity, ComponentTypeT::typeId));
  }

  template<typename ComponentTypeT>
  ComponentTypeT *EntityComponentManager::Component(const Entity _entity)
  {
    return static_cast<ComponentTypeT *>(
        this->ComponentImplementation(_entity, ComponentTypeT::typeId));
  }

  template<typename ComponentTypeT>
  ComponentTypeT *EntityComponentManager::ComponentDefault(
      Entity _entity, const typename ComponentTypeT::Type &_default)
  {
    auto comp = this->Component<ComponentTypeT>(_entity);
    if (!comp)
    {
      this->CreateComponent(_entity, ComponentTypeT(_default));
      comp = this->Component<ComponentTypeT>(_entity);
    }
    return comp;
  }

  template<typename ComponentTypeT>
  std::optional<typename ComponentTypeT::Type>
      EntityComponentManager::ComponentData(const Entity _entity) const
  {
    auto comp = this->Component<ComponentTypeT>(_entity);
    if (!comp) return std::nullopt;
    return std::make_optional(comp->Data());
  }

  template<typename ComponentTypeT>
  bool EntityComponentManager::SetComponentData(
      const Entity _entity,
      const typename ComponentTypeT::Type &_data)
  {
    auto comp = this->Component<ComponentTypeT>(_entity);
    if (nullptr == comp)
    {
      // NOTE(0b-immediate-mode): immediate semantics per design §6.1.
      // Phase 0c migrates this to deferred-always. Do not remove this
      // comment without updating the 0c checklist.
      this->CreateComponent(_entity, ComponentTypeT(_data));
      return true;
    }
    return comp->SetData(_data, CompareData<typename ComponentTypeT::Type>);
  }

  template<typename ComponentTypeT>
  bool EntityComponentManager::RemoveComponent(Entity _entity)
  {
    return this->RemoveComponent(_entity, ComponentTypeT::typeId);
  }

  //----------------------------------------------------------
  // Identity helper (same as legacy).
  template <typename T>
  struct EntityComponentManager::identity  // NOLINT
  {
    using type = T;
  };

  //----------------------------------------------------------
  // View-driven templates — rewritten against ecs::World::Each.
  //
  // Implementation note: the archetype backend has no detail::View
  // class. `FindView<T...>()` is *declared* in the public header
  // (legacy calls rely on it) but we never define it for the archetype
  // build — any caller that reaches for it will fail to link, which
  // we consider a feature: the legacy `FindView` surface is an
  // implementation detail of the legacy .cc, not public API.
  //
  // We do need to handle the edge case where the archetype World
  // pointer is null (EntityComponentManager constructor before the
  // archetype PIMPL is fully initialized): every Each* becomes a
  // no-op in that case.

  namespace detail_archetype
  {
    /// \brief The archetype core uses ecs::Entity (index+gen). The
    /// legacy public API uses gz::sim::Entity (uint64). The archetype
    /// PIMPL maintains a translation table; this helper is defined in
    /// src/EntityComponentManagerArchetype.cc so it can reach into the
    /// PIMPL without exposing its layout in a header.
    GZ_SIM_VISIBLE gz::sim::Entity FromCore(
        const EntityComponentManager *_ecm,
        gz::sim::ecs::Entity _core);
  }

  //----------------------------------------------------------
  // Each<T...>
  //
  // The archetype backend's Each walks every entity the facade knows
  // about (via AllEntitiesArchetypeFacade) and tests each for the
  // requested component tuple via the non-template
  // ComponentImplementation. That helper already unifies the shadow
  // store + archetype World, so both storage paths are covered in
  // one pass.
  //
  // Performance trade-off: the Phase 0a archetype core's Each<T>
  // gets chunk-contiguous iteration (~3 ns/entity). This per-entity
  // loop is closer to the legacy ECM's per-entity-map-lookup cost
  // (~30-50 ns/entity). For Phase 0b that's acceptable — we're
  // trading raw speed for correctness across both storage paths.
  // Phase 0b completion (Factory ↔ ComponentTypeRegistry bridge)
  // eliminates the shadow store and lets Each<T> go back to the
  // archetype-native fast path.

  template<typename ...ComponentTypeTs>
  void EntityComponentManager::Each(typename identity<std::function<
      bool(const Entity &_entity,
           const ComponentTypeTs *...)>>::type _f) const
  {
    for (Entity e : this->AllEntitiesArchetypeFacade())
    {
      auto comps = std::make_tuple(
          this->Component<std::remove_const_t<ComponentTypeTs>>(e)...);
      bool hasAll = true;
      std::apply([&](auto*... ps) { hasAll = ((ps != nullptr) && ...); },
                 comps);
      if (!hasAll) continue;
      bool keep = std::apply([&](auto*... ps) {
        return _f(e, ps...);
      }, comps);
      if (!keep) break;
    }
  }

  template<typename ...ComponentTypeTs>
  void EntityComponentManager::Each(typename identity<std::function<
      bool(const Entity &_entity, ComponentTypeTs *...)>>::type _f)
  {
    for (Entity e : this->AllEntitiesArchetypeFacade())
    {
      auto comps = std::make_tuple(
          this->Component<std::remove_const_t<ComponentTypeTs>>(e)...);
      bool hasAll = true;
      std::apply([&](auto*... ps) { hasAll = ((ps != nullptr) && ...); },
                 comps);
      if (!hasAll) continue;
      bool keep = std::apply([&](auto*... ps) {
        return _f(e, ps...);
      }, comps);
      if (!keep) break;
    }
  }

  //----------------------------------------------------------
  // EachNoCache — archetype backend doesn't have view caches, so
  // "no cache" just means Each. Identical semantics.

  template<typename ...ComponentTypeTs>
  void EntityComponentManager::EachNoCache(typename identity<std::function<
      bool(const Entity &_entity,
           const ComponentTypeTs *...)>>::type _f) const
  {
    this->Each<ComponentTypeTs...>(_f);
  }

  template<typename ...ComponentTypeTs>
  void EntityComponentManager::EachNoCache(typename identity<std::function<
      bool(const Entity &_entity, ComponentTypeTs *...)>>::type _f)
  {
    this->Each<ComponentTypeTs...>(_f);
  }

  //----------------------------------------------------------
  // EachNew — walks entities in the facade's newlyCreated set,
  // drained by ClearNewlyCreatedEntities at end-of-step. Without
  // this filter Physics sees the same entities every tick and its
  // registration map trips "already on map" warnings.

  template <typename... ComponentTypeTs>
  void EntityComponentManager::EachNew(typename identity<std::function<
      bool(const Entity &_entity, ComponentTypeTs *...)>>::type _f)
  {
    for (Entity e : this->AllEntitiesArchetypeFacade())
    {
      if (!this->IsNewEntity(e)) continue;
      auto comps = std::make_tuple(
          this->Component<std::remove_const_t<ComponentTypeTs>>(e)...);
      bool hasAll = true;
      std::apply([&](auto*... ps) { hasAll = ((ps != nullptr) && ...); },
                 comps);
      if (!hasAll) continue;
      bool keep = std::apply([&](auto*... ps) {
        return _f(e, ps...);
      }, comps);
      if (!keep) break;
    }
  }

  template <typename... ComponentTypeTs>
  void EntityComponentManager::EachNew(typename identity<std::function<
      bool(const Entity &_entity,
           const ComponentTypeTs *...)>>::type _f) const
  {
    for (Entity e : this->AllEntitiesArchetypeFacade())
    {
      if (!this->IsNewEntity(e)) continue;
      auto comps = std::make_tuple(
          this->Component<std::remove_const_t<ComponentTypeTs>>(e)...);
      bool hasAll = true;
      std::apply([&](auto*... ps) { hasAll = ((ps != nullptr) && ...); },
                 comps);
      if (!hasAll) continue;
      bool keep = std::apply([&](auto*... ps) {
        return _f(e, ps...);
      }, comps);
      if (!keep) break;
    }
  }

  //----------------------------------------------------------
  // EachRemoved — walks entities pending removal. Uses the facade's
  // pendingRemovals list. Components are still alive until
  // ProcessRemoveEntityRequests commits the removal, so the
  // component pointers are valid inside the callback.

  template<typename ...ComponentTypeTs>
  void EntityComponentManager::EachRemoved(typename identity<std::function<
      bool(const Entity &_entity,
           const ComponentTypeTs *...)>>::type _f) const
  {
    // Walk entities marked for removal and honor the same
    // "has all components" contract as Each.
    for (Entity e : this->AllEntitiesArchetypeFacade())
    {
      if (!this->IsMarkedForRemoval(e)) continue;
      auto comps = std::make_tuple(
          this->Component<std::remove_const_t<ComponentTypeTs>>(e)...);
      bool hasAll = true;
      std::apply([&](auto*... ps) { hasAll = ((ps != nullptr) && ...); },
                 comps);
      if (!hasAll) continue;
      bool keep = std::apply([&](auto*... ps) {
        return _f(e, ps...);
      }, comps);
      if (!keep) break;
    }
  }

  //----------------------------------------------------------
  // ForEach — the static helper works unchanged; backend-agnostic.

  template <class Function, class... ComponentTypeTs>
  void EntityComponentManager::ForEach(Function _f,
      const ComponentTypeTs &... _components)
  {
    (_f(_components), ...);
  }

  //----------------------------------------------------------
  // EntityByComponents / EntitiesByComponents / ChildrenByComponents
  //
  // These compare desired component values against entity components
  // to find matches. The archetype Each<T...> gives us the candidate
  // set; we filter inside the callback.

  template<typename ...ComponentTypeTs>
  Entity EntityComponentManager::EntityByComponents(
      const ComponentTypeTs &..._desiredComponents) const
  {
    for (Entity e : this->AllEntitiesArchetypeFacade())
    {
      bool different = false;
      ForEach([&](const auto &_want)
      {
        using CT = std::remove_cv_t<std::remove_reference_t<
            decltype(_want)>>;
        auto *have = this->template Component<CT>(e);
        if (!have || !(*have == _want)) different = true;
      }, _desiredComponents...);
      if (!different) return e;
    }
    return kNullEntity;
  }

  template<typename ...ComponentTypeTs>
  std::vector<Entity> EntityComponentManager::EntitiesByComponents(
      const ComponentTypeTs &..._desiredComponents) const
  {
    std::vector<Entity> result;
    for (Entity e : this->AllEntitiesArchetypeFacade())
    {
      bool different = false;
      ForEach([&](const auto &_want)
      {
        using CT = std::remove_cv_t<std::remove_reference_t<
            decltype(_want)>>;
        auto *have = this->template Component<CT>(e);
        if (!have || !(*have == _want)) different = true;
      }, _desiredComponents...);
      if (!different) result.push_back(e);
    }
    return result;
  }

  template<typename ...ComponentTypeTs>
  std::vector<Entity> EntityComponentManager::ChildrenByComponents(
      Entity _parent,
      const ComponentTypeTs &..._desiredComponents) const
  {
    // For this landing the children scan uses the legacy entity graph
    // (which the archetype PIMPL maintains). Filter via component
    // equality afterward.
    std::vector<Entity> result;
    auto children = this->Entities().AdjacentsFrom(_parent);
    for (const auto &child : children)
    {
      bool different = false;
      ForEach([&](const auto &_desired)
      {
        using CT = std::remove_cv_t<std::remove_reference_t<
            decltype(_desired)>>;
        auto comp = this->Component<CT>(child.first);
        if (!comp || !(*comp == _desired)) different = true;
      }, _desiredComponents...);
      if (!different) result.push_back(child.first);
    }
    return result;
  }
}
}
}

#endif  // GZ_SIM_USE_ARCHETYPE_ECM (Phase 0b backend guard)

#endif
