/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 * Licensed under the Apache License, Version 2.0
 */
#ifndef GZ_SIM_ECS_ARCHETYPEBACKEDECM_HH_
#define GZ_SIM_ECS_ARCHETYPEBACKEDECM_HH_

#include <functional>
#include <unordered_map>
#include <vector>

#include "gz/sim/Entity.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/ecs/World.hh"

namespace gz::sim::ecs
{
  /// \brief Thin facade that exposes the subset of the legacy
  /// EntityComponentManager API backed by gz::sim::ecs::World.
  ///
  /// Phase 0b. Purpose: show the archetype core can impersonate the ECM
  /// for the methods most systems actually use, so a later milestone can
  /// swap this in as the `Impl` typedef inside EntityComponentManager's
  /// PIMPL without changing any caller.
  ///
  /// Only covers what's exercised by the parity test in this phase:
  /// CreateEntity, HasEntity, RequestRemoveEntity +
  /// ProcessRemoveEntityRequests, CreateComponent, RemoveComponent,
  /// Component, HasComponentType, Each, EachNew, EachRemoved,
  /// EachChanged, Commit/CommitPhase, ClearNewlyCreatedEntities,
  /// ClearRemovedComponents, SetAllComponentsUnchanged.
  ///
  /// Notably NOT covered in 0b (deferred):
  ///   - State / SetState / ChangedState serialization
  ///   - Views / BaseView (legacy-only)
  ///   - Parent/child graph (ParentEntity still works as a component)
  ///   - Clone, ResetTo, CopyFrom
  ///   - Level pinning — ships as the standard LevelPin component.
  class ArchetypeBackedECM
  {
    public: ArchetypeBackedECM();
    public: ~ArchetypeBackedECM();

    public: ArchetypeBackedECM(const ArchetypeBackedECM &) = delete;
    public: ArchetypeBackedECM &operator=(const ArchetypeBackedECM &) = delete;

    // -------- Entity lifecycle --------

    /// \brief Create a new entity. Matches ECM semantics.
    public: gz::sim::Entity CreateEntity();

    /// \brief Matches ECM::HasEntity.
    public: bool HasEntity(gz::sim::Entity _e) const;

    /// \brief Queue a removal; applied on the next Commit.
    public: void RequestRemoveEntity(gz::sim::Entity _e);

    /// \brief Apply all queued removals.
    public: void ProcessRemoveEntityRequests();

    // -------- Component lifecycle --------

    public: template <class T>
    T *CreateComponent(gz::sim::Entity _e, T _value = T())
    {
      this->world_.Add(this->Handle(_e), std::move(_value));
      return this->world_.Component<T>(this->Handle(_e));
    }

    public: template <class T>
    bool RemoveComponent(gz::sim::Entity _e)
    {
      if (!this->HasEntity(_e)) return false;
      this->world_.Remove<T>(this->Handle(_e));
      return true;
    }

    public: template <class T>
    const T *Component(gz::sim::Entity _e) const
    {
      return this->world_.Component<T>(this->Handle(_e));
    }

    public: template <class T>
    T *Component(gz::sim::Entity _e)
    {
      return this->world_.Component<T>(this->Handle(_e));
    }

    public: template <class T>
    bool EntityHasComponent(gz::sim::Entity _e) const
    {
      return this->HasEntity(_e) &&
             this->world_.Has<T>(this->Handle(_e));
    }

    // -------- Iteration --------

    /// \brief Each<T...>(fn) — fn signature: bool(Entity, T*...).
    /// Callback returning false stops the iteration (ECM parity).
    public: template <class... Ts, class Fn>
    void Each(Fn _fn)
    {
      // We bridge ecs::World's callback (reference-pack) to the ECM's
      // pointer-pack via a wrapping lambda. Early-exit honored via flag.
      bool keep_going = true;
      this->world_.template Each<Ts...>(
          [&](gz::sim::ecs::Entity _he, Ts &... _cs)
      {
        if (!keep_going) return;
        if (!_fn(this->LegacyHandle(_he), &_cs...))
          keep_going = false;
      });
    }

    public: template <class... Ts, class Fn>
    void EachNew(Fn _fn)
    {
      bool keep_going = true;
      this->world_.template EachNew<Ts...>(
          [&](gz::sim::ecs::Entity _he, Ts &... _cs)
      {
        if (!keep_going) return;
        if (!_fn(this->LegacyHandle(_he), &_cs...))
          keep_going = false;
      });
    }

    public: template <class... Ts, class Fn>
    void EachChanged(Fn _fn)
    {
      bool keep_going = true;
      this->world_.template EachChanged<Ts...>(
          [&](gz::sim::ecs::Entity _he, Ts &... _cs)
      {
        if (!keep_going) return;
        if (!_fn(this->LegacyHandle(_he), &_cs...))
          keep_going = false;
      });
    }

    public: template <class... Ts, class Fn>
    void EachRemoved(Fn _fn)
    {
      bool keep_going = true;
      this->world_.template EachRemoved<Ts...>(
          [&](gz::sim::ecs::Entity _he)
      {
        if (!keep_going) return;
        if (!_fn(this->LegacyHandle(_he)))
          keep_going = false;
      });
    }

    // -------- Phase / commit hooks --------

    /// \brief End-of-phase commit. Drains the command buffer, invalidates
    /// pointers obtained during the phase. SimulationRunner calls this
    /// between PreUpdate / Update / PostUpdate.
    public: void CommitPhase();

    /// \brief BeginPhase — subsequent mutations are deferred.
    public: void BeginPhase();

    /// \brief Reset change-tracking arena. Matches
    /// ECM::SetAllComponentsUnchanged.
    public: void SetAllComponentsUnchanged();

    /// \brief Matches ECM::ClearNewlyCreatedEntities (partial — clears
    /// newly-added bookkeeping).
    public: void ClearNewlyCreatedEntities();

    /// \brief Matches ECM::ClearRemovedComponents.
    public: void ClearRemovedComponents();

    // -------- Testing / inspection --------

    public: size_t EntityCount() const;

    public: World &Core() { return this->world_; }
    public: const World &Core() const { return this->world_; }

    // -------- Entity-handle translation --------
    //
    // The legacy API uses a flat uint64 Entity. The archetype core uses a
    // wider handle with index+generation. We maintain a bidirectional
    // map here so public methods can speak legacy handles and forward to
    // the core transparently. In 0e we can drop the translation entirely
    // and make the core's handle the canonical representation.
    private: gz::sim::ecs::Entity Handle(gz::sim::Entity _e) const;
    private: gz::sim::Entity      LegacyHandle(gz::sim::ecs::Entity _h) const;

    private: World world_{1};
    private: std::unordered_map<uint64_t, gz::sim::ecs::Entity> by_legacy_;
    private: std::unordered_map<uint64_t, gz::sim::Entity>      by_core_;
    private: gz::sim::Entity next_legacy_{1};
    private: std::vector<gz::sim::Entity> pending_removals_;
  };
}

#endif
