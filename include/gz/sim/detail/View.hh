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
#ifndef GZ_SIM_DETAIL_VIEW_HH_
#define GZ_SIM_DETAIL_VIEW_HH_

#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>

#include "gz/sim/components/Component.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/config.hh"
#include "gz/sim/detail/BaseView.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace detail
{
/// \brief A view that caches a particular set of component type data.
///
/// Note that symbols for this class are visible because methods from this class
/// are used in templated Gz::Sim::EntityComponentManager methods.
/// However, users should not use this class (or anything else in namespace
/// gz::sim::detail) directly.
class GZ_SIM_VISIBLE View : public BaseView
{
  /// \brief Alias for containers that hold and entity and its component data.
  /// The component types held in this container match the component types that
  /// were specified when creating the view.
  private: using ComponentData = std::vector<components::BaseComponent *>;
  private: using ConstComponentData =
               std::vector<const components::BaseComponent *>;

  /// \brief Constructor
  /// \param[in] _compIds a set of IDs of the components cached by this View.
  public: explicit View(const std::set<ComponentTypeId> &_compIds);

  /// \brief Documentation inherited
  public: bool HasCachedComponentData(const Entity _entity) const override;

  /// \brief Documentation inherited
  public: bool RemoveEntity(const Entity _entity) override;

  /// \brief Get an entity and its component data. It is assumed that the entity
  /// being requested exists in the view.
  /// \param[_in] _entity The entity
  /// \return The entity and its component data. Const pointers to the component
  /// data are returned.
  public: const ConstComponentData &EntityComponentConstData(
              const Entity _entity) const;

  /// \brief Get an entity and its component data. It is assumed that the entity
  /// being requested exists in the view.
  /// \param[_in] _entity The entity
  /// \return The entity and its component data. Mutable pointers to the
  /// component data are returned.
  public: const ComponentData &EntityComponentData(const Entity _entity) const;

  /// \brief Add an entity with its component data to the view. It is assumed
  /// that the entity to be added does not already exist in the view.
  /// \tparam ComponentTypeTs The component type(s) that are stored in this
  /// view. These types correspond to each of the types in the _compPtrs
  /// parameter of this function.
  /// \param[in] _entity The entity
  /// \param[in] _new Whether to add the entity to the list of new entities.
  /// The new here is to indicate whether the entity is new to the entity
  /// component manager. An existing entity can be added when creating a new
  /// view or when rebuilding the view.
  /// \param[in] _compPtrs Const pointers to the entity's components
  public: template<typename ...ComponentTypeTs>
          void AddEntityWithConstComps(const Entity &_entity, const bool _new,
              const ComponentTypeTs*... _compPtrs);

  /// \brief Add an entity with its component data to the view. It is assumed
  /// that the entity to be added does not already exist in the view.
  /// \tparam ComponentTypeTs The component type(s) that are stored in this
  /// view. These types correspond to each of the types in the _compPtrs
  /// parameter of this function.
  /// \param[in] _entity The entity
  /// \param[in] _new Whether to add the entity to the list of new entities.
  /// The new here is to indicate whether the entity is new to the entity
  /// component manager. An existing entity can be added when creating a new
  /// view or when rebuilding the view.
  /// \param[in] _compPtrs Pointers to the entity's components
  public: template<typename ...ComponentTypeTs>
          void AddEntityWithComps(const Entity &_entity, const bool _new,
              ComponentTypeTs*... _compPtrs);

  /// \brief Documentation inherited
  public: bool NotifyComponentAddition(const Entity _entity, bool _newEntity,
              const ComponentTypeId _typeId) override;

  /// \brief Documentation inherited
  public: bool NotifyComponentRemoval(const Entity _entity,
              const ComponentTypeId _typeId) override;

  /// \brief Documentation inherited
  public: void Reset() override;

  /// \brief A map of entities to their component data. Since tuples are defined
  /// at compile time, we need separate containers that have tuples for both
  /// non-const and const component pointers (calls to ECM::Each can have a
  /// method signature that uses either non-const or const pointers)
  private: std::unordered_map<Entity, ComponentData> validData;
  private: std::unordered_map<Entity, ConstComponentData> validConstData;

  /// \brief A map of invalid entities to their component data. The difference
  /// between invalidData and validData is that the entities in invalidData were
  /// once in validData, but they had a component removed, so the entity no
  /// longer meets the component requirements of the view. If the missing
  /// component data is ever added back to an entity in invalidData, then this
  /// entity will be moved back to validData. The usage of invalidData is an
  /// implementation detail that should be ignored by those using the View API;
  /// from a user's point of view, entities that belong to invalidData don't
  /// appear to be a part of the view at all.
  ///
  /// The reason for moving entities with missing components to invalidData
  /// instead of completely deleting them from the view is because if components
  /// are added back later and the entity needs to be re-added to the view,
  /// tuple creation can be costly. So, this approach is used instead to
  /// maintain runtime performance (the tradeoff of mainting performance is
  /// increased complexity and memory usage).
  ///
  /// \sa missingCompTracker
  private: std::unordered_map<Entity, ComponentData> invalidData;
  private: std::unordered_map<Entity, ConstComponentData> invalidConstData;

  /// \brief A map that keeps track of which component types for entities in
  /// invalidData need to be added back to the entity in order to move the
  /// entity back to validData. If the set of types (value in the map) becomes
  /// empty, then this means that the entity (key in the map) has all of the
  /// component types defined by the view, so the entity can be moved back to
  /// validData.
  ///
  /// \sa invalidData
  private: std::unordered_map<Entity, std::unordered_set<ComponentTypeId>>
             missingCompTracker;
};

//////////////////////////////////////////////////
template <typename... ComponentTypeTs>
void View::AddEntityWithConstComps(const Entity &_entity, const bool _new,
                                   const ComponentTypeTs *... _compPtrs)
{
  this->validConstData[_entity] =
      std::vector<const components::BaseComponent *>{_compPtrs...};
  this->entities.insert(_entity);
  if (_new)
    this->newEntities.insert(_entity);
}

//////////////////////////////////////////////////
template <typename... ComponentTypeTs>
void View::AddEntityWithComps(const Entity &_entity, const bool _new,
                              ComponentTypeTs *... _compPtrs)
{
  this->validData[_entity] =
      std::vector<components::BaseComponent *>{_compPtrs...};
  this->entities.insert(_entity);
  if (_new)
    this->newEntities.insert(_entity);
}
}  // namespace detail
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
#endif
