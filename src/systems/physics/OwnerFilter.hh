/*
 * Copyright (C) 2026 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_PHYSICS_OWNER_FILTER_HH_
#define GZ_SIM_SYSTEMS_PHYSICS_OWNER_FILTER_HH_

#include <functional>
#include <string>
#include <utility>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/PhysicsOwner.hh"
#include "gz/sim/config.hh"

namespace gz::sim
{
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems::physics_system
{
  /// \brief Selects the entities that one physics provider writes or clears.
  ///
  /// The write-back and command-clearing helpers take a filter, so that a
  /// provider only touches the entities it simulates. A default-constructed
  /// filter accepts every entity, which is the behavior of a provider that
  /// has no need to share the ECM.
  ///
  /// A filter is cheap to copy and holds no ECM state of its own. Build it
  /// for one call or one step, because a filter made by ForProvider() refers
  /// to the ECM it was made from.
  class OwnerFilter
  {
    /// \brief Test that decides whether the provider owns an entity.
    public: using Predicate = std::function<bool(const Entity &_entity)>;

    /// \brief Constructor for a filter that accepts every entity.
    public: OwnerFilter() = default;

    /// \brief Constructor for a filter that asks a predicate. A provider
    /// that keeps its own set of owned entities passes a lookup in the set,
    /// which is cheaper than resolving ownership from the ECM every time.
    /// \param[in] _owns Returns true for the entities the provider owns.
    public: explicit OwnerFilter(Predicate _owns)
        : owns(std::move(_owns))
    {
    }

    /// \brief Make a filter that resolves ownership with gz::sim::isOwnedBy.
    ///
    /// The lookup walks up the entity tree for every entity it is asked
    /// about, so use it for rare checks, not for per-step loops over many
    /// entities. While no entity in the ECM has a PhysicsOwner, the result
    /// is a filter that accepts everything (default provider) or nothing
    /// (any other provider), and the check costs one lookup now and none per
    /// entity.
    ///
    /// \param[in] _ecm Constant reference to the ECM. It must outlive the
    /// filter.
    /// \param[in] _provider Name of the provider, in the form stored in
    /// components::PhysicsOwner.
    /// \param[in] _isDefaultProvider True if the provider also owns every
    /// entity that has no PhysicsOwner.
    /// \return The filter.
    public: static OwnerFilter ForProvider(const EntityComponentManager &_ecm,
        const std::string &_provider, bool _isDefaultProvider)
    {
      if (!_ecm.HasComponentType(components::PhysicsOwner::typeId))
      {
        if (_isDefaultProvider)
          return OwnerFilter();
        return OwnerFilter([](const Entity &) { return false; });
      }

      return OwnerFilter(
          [&_ecm, _provider, _isDefaultProvider](const Entity &_entity)
          {
            return isOwnedBy(_entity, _ecm, _provider, _isDefaultProvider);
          });
    }

    /// \brief Check whether the provider owns an entity.
    /// \param[in] _entity Any entity, such as a model, link, joint or
    /// collision.
    /// \return True if the provider owns the entity.
    public: bool Owns(const Entity &_entity) const
    {
      return !this->owns || this->owns(_entity);
    }

    /// \brief The test, or an empty function if the filter accepts
    /// everything.
    private: Predicate owns;
  };
}
}
}

#endif
