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
#ifndef IGNITION_GAZEBO_DETAIL_VIEW_HH_
#define IGNITION_GAZEBO_DETAIL_VIEW_HH_

#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include "ignition/gazebo/components/Component.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Export.hh"
#include "ignition/gazebo/Types.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace detail
{
/// \brief A key into the map of views
using ComponentTypeKey = std::set<ComponentTypeId>;

/// \brief A view is a cache to entities, and their components, that
/// match a set of component types. A cache is used because systems will
/// frequently, potentially every iteration, query the
/// EntityComponentManager for sets of entities that match a set of
/// component types. Rather than look up the entities every time, we can
/// use a cache to improve performance. The assumption is that entities
/// and the types of components assigned to entities change infrequently
/// compared to the frequency of queries performed by systems.
class IGNITION_GAZEBO_VISIBLE View
{
  /// Get a pointer to a component for an entity based on a component type.
  /// \param[in] _entity The entity.
  /// \param[in] _ecm Pointer to the entity component manager.
  /// \return Pointer to the component.
  public: template<typename ComponentTypeT>
          const ComponentTypeT *Component(const Entity _entity,
              const EntityComponentManager *_ecm) const
  {
    ComponentTypeId typeId = ComponentTypeT::typeId;
    return static_cast<const ComponentTypeT *>(
        this->ComponentImplementation(_entity, typeId, _ecm));
  }

  /// Get a pointer to a component for an entity based on a component type.
  /// \param[in] _entity The entity.
  /// \param[in] _ecm Pointer to the entity component manager.
  /// \return Pointer to the component.
  public: template<typename ComponentTypeT>
          ComponentTypeT *Component(const Entity _entity,
              const EntityComponentManager *_ecm)
  {
    ComponentTypeId typeId = ComponentTypeT::typeId;
    return static_cast<ComponentTypeT *>(
        const_cast<components::BaseComponent *>(
          this->ComponentImplementation(_entity, typeId, _ecm)));
  }

  /// \brief Add an entity to the view.
  /// \param[in] _entity The entity to add.
  /// \param[in] _new Whether to add the entity to the list of new entities.
  /// The new here is to indicate whether the entity is new to the entity
  /// component manager. An existing entity can be added when creating a new
  /// view or when rebuilding the view.
  public: void AddEntity(const Entity _entity, const bool _new = false);

  /// \brief Remove an entity from the view.
  /// \param[in] _entity The entity to remove.
  /// \param[in] _key Components that should also be removed.
  /// \return True if the entity was removed, false if the entity did not
  /// exist in the view.
  public: bool RemoveEntity(const Entity _entity,
                           const ComponentTypeKey &_key);

  /// \brief Add the entity to the list of entities to be removed
  /// \param[in] _entity The entity to add.
  /// \return True if the entity was added to the list, false if the entity
  /// did not exist in the view.
  public: bool AddEntityToRemoved(const Entity _entity);

  /// \brief Add a component to an entity.
  /// \param[in] _entity The entity.
  /// \param[in] _compTypeId Component type id.
  /// \param[in] _compId Component id.
  public: void AddComponent(const Entity _entity,
                            const ComponentTypeId _compTypeId,
                            const ComponentId _compId);

  /// \brief Implementation of the Component accessor.
  /// \param[in] _entity The entity.
  /// \param[in] _typeId Type id of the component.
  /// \param[in] _ecm Pointer to the EntityComponentManager.
  /// \return Pointer to the component, or nullptr if not found.
  private: const components::BaseComponent *ComponentImplementation(
               const Entity _entity,
               ComponentTypeId _typeId,
               const EntityComponentManager *_ecm) const;

  /// \brief Clear the list of new entities
  public: void ClearNewEntities();

  /// \brief All the entities that belong to this view.
  public: std::set<Entity> entities;

  /// \brief List of newly created entities
  public: std::set<Entity> newEntities;

  /// \brief List of entities about to be removed
  public: std::set<Entity> toRemoveEntities;

  /// \brief Hash functor for std::pair<Entity, ComponentTypeId>
  public: struct Hasher
          {
            std::size_t operator()(
                std::pair<Entity, ComponentTypeId> _pair) const
            {
              _pair.first ^= _pair.second + 0x9e3779b9 + (_pair.second << 6)
                 + (_pair.second >> 2);
              return _pair.first;
            }
          };

  /// \brief Equality functor for std::pair<Entity, ComponentTypeId>
  public: struct EqualTo
          {
            bool operator()(const std::pair<Entity, ComponentTypeId> &_lhs,
                const std::pair<Entity, ComponentTypeId> &_rhs) const
            {
              return _lhs.first == _rhs.first &&
                _lhs.second == _rhs.second;
            }
          };

  /// \brief All of the components for each entity.
  public: std::unordered_map<std::pair<Entity, ComponentTypeId>, ComponentId,
            Hasher, EqualTo> components;
};
/// \endcond
}
}
}
}
#endif
