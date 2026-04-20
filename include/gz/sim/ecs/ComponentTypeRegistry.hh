/*
 * Copyright (C) 2026 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 */
#ifndef GZ_SIM_ECS_COMPONENTTYPEREGISTRY_HH_
#define GZ_SIM_ECS_COMPONENTTYPEREGISTRY_HH_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <type_traits>
#include <typeinfo>
#include <vector>

#include <gz/common/Util.hh>

#include "gz/sim/Types.hh"

namespace gz::sim::ecs
{
  /// \brief Component type identifier. Aliased to the top-level
  /// `gz::sim::ComponentTypeId` so a type registered with
  /// `GZ_SIM_REGISTER_COMPONENT` gets the same id when used in either the
  /// classic ECM or the 0a archetype ECS.
  using ComponentTypeId = gz::sim::ComponentTypeId;

  inline constexpr ComponentTypeId kInvalidComponentTypeId = 0;

  /// \brief Per-type metadata stored in the registry. Mirrors and extends the
  /// current ComponentDescriptor used by gz::sim.
  struct ComponentTypeInfo
  {
    ComponentTypeId id{kInvalidComponentTypeId};
    size_t          size{0};
    size_t          alignment{0};
    void (*construct)(void *dst){nullptr};
    void (*copy)(void *dst, const void *src){nullptr};
    void (*move)(void *dst, void *src){nullptr};
    void (*destruct)(void *obj){nullptr};
    bool            trivially_relocatable{false};
    std::string     name;
  };

  namespace detail
  {
    template <class T>
    void DefaultConstruct(void *dst) { new (dst) T(); }

    template <class T>
    void CopyConstruct(void *dst, const void *src)
    {
      new (dst) T(*static_cast<const T *>(src));
    }

    template <class T>
    void MoveConstruct(void *dst, void *src)
    {
      new (dst) T(std::move(*static_cast<T *>(src)));
    }

    template <class T>
    void Destruct(void *obj) { static_cast<T *>(obj)->~T(); }

    /// \brief SFINAE probe for a `static ComponentTypeId typeId` member, the
    /// convention established by `GZ_SIM_REGISTER_COMPONENT`.
    template <class, class = void>
    struct HasGzSimTypeId : std::false_type {};

    template <class T>
    struct HasGzSimTypeId<T, std::void_t<decltype(T::typeId)>>
        : std::true_type {};
  }

  /// \brief Append-only registry of ComponentTypeInfo. Thread-safe on
  /// registration; lookups are lock-free once the type is warm (registered).
  class ComponentTypeRegistry
  {
    public: static ComponentTypeRegistry &Instance();

    /// \brief Get metadata for a previously registered type, or nullptr.
    public: const ComponentTypeInfo *Get(ComponentTypeId _id) const;

    /// \brief Register a type T. Idempotent: returns the existing entry if
    /// the type is already registered.
    public: template <class T>
    const ComponentTypeInfo *Register(std::string_view _name = "")
    {
      static_assert(std::is_nothrow_destructible_v<T>,
          "Components must be nothrow-destructible.");
      static_assert(alignof(T) <= 64,
          "Component alignment > 64B not supported in 0a.");

      const ComponentTypeId id = TypeIdOf<T>();
      if (auto *existing = this->Get(id))
        return existing;

      ComponentTypeInfo info{};
      info.id = id;
      info.size = sizeof(T);
      info.alignment = alignof(T);
      info.construct = &detail::DefaultConstruct<T>;
      info.copy = &detail::CopyConstruct<T>;
      info.move = &detail::MoveConstruct<T>;
      info.destruct = &detail::Destruct<T>;
      info.trivially_relocatable =
          std::is_trivially_copyable_v<T> &&
          std::is_trivially_destructible_v<T>;
      info.name = _name.empty() ? std::string(typeid(T).name())
                                : std::string(_name);
      return this->Install(std::move(info));
    }

    /// \brief Derive a component type id for T.
    ///
    /// If T is a `gz::sim::components::Component<...>`-style type (i.e.,
    /// it has a `static ComponentTypeId typeId` set by
    /// `GZ_SIM_REGISTER_COMPONENT`), that id is returned. The same type
    /// therefore has the same id whether used via the classic ECM or the
    /// 0a archetype ECS.
    ///
    /// If T is a plain struct (common in tests), the id is
    /// `gz::common::hash64(typeid(T).name())` — the same algorithm used by
    /// `Factory::Register` internally, so the two paths stay consistent.
    public: template <class T>
    static ComponentTypeId TypeIdOf()
    {
      if constexpr (detail::HasGzSimTypeId<T>::value)
      {
        if (T::typeId != 0)
          return T::typeId;
      }
      return gz::common::hash64(typeid(T).name());
    }

    /// \brief Number of registered types (for debug / tests).
    public: size_t Size() const;

    /// \brief Testing hook: wipe the registry. Do not call in production.
    public: void ClearForTesting();

    private: ComponentTypeRegistry() = default;

    private: const ComponentTypeInfo *Install(ComponentTypeInfo &&_info);

    // Entries are append-only; pointers returned from Get() remain valid for
    // the registry's lifetime.
    private: mutable std::mutex               mutex_;
    private: std::vector<ComponentTypeInfo *> entries_;
    // Dense index ComponentTypeId -> pointer. We use a separate linear scan
    // for the lookup — O(N) cold, but N is tiny (dozens of component types).
  };
}

#endif
