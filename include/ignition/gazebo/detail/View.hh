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

#include <memory>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <ignition/common/Console.hh>

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/detail/BaseView.hh"

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace detail
{
/// \brief A view that caches a particular set of component type data.
/// \tparam ComponentTypeTs The component type(s) that are stored in this view.
template<typename ...ComponentTypeTs>
class View : public BaseView
{
  /// \brief Alias for containers that hold and entity and its component data.
  /// The component types held in this container match the component types that
  /// were specified when creating the view.
  private: using ComponentData = std::tuple<Entity, ComponentTypeTs*...>;
  private: using ConstComponentData =
           std::tuple<Entity, const ComponentTypeTs*...>;

  /// \brief Constructor
  public: View();

  /// \brief Documentation inherited
  public: bool HasCachedComponentData(const Entity _entity) const override;

  /// \brief Documentation inherited
  public: bool RemoveEntity(const Entity _entity) override;

  /// \brief Get an entity and its component data. It is assumed that the entity
  /// being requested exists in the view.
  /// \param[_in] _entity The entity
  /// \return The entity and its component data. Const pointers to the component
  /// data are returned.
  public: ConstComponentData EntityComponentConstData(
              const Entity _entity) const;

  /// \brief Get an entity and its component data. It is assumed that the entity
  /// being requested exists in the view.
  /// \param[_in] _entity The entity
  /// \return The entity and its component data. Mutable pointers to the
  /// component data are returned.
  public: ComponentData EntityComponentData(const Entity _entity);

  /// \brief Add an entity with its component data to the view. It is assumed
  /// that the entity to be added does not already exist in the view.
  /// \param[in] _entity The entity
  /// \param[in] _new Whether to add the entity to the list of new entities.
  /// The new here is to indicate whether the entity is new to the entity
  /// component manager. An existing entity can be added when creating a new
  /// view or when rebuilding the view.
  /// \param[in] _compPtrs Const pointers to the entity's components
  public: void AddEntityWithConstComps(const Entity &_entity, const bool _new,
              const ComponentTypeTs*... _compPtrs);

  /// \brief Add an entity with its component data to the view. It is assumed
  /// that the entity to be added does not already exist in the view.
  /// \param[in] _entity The entity
  /// \param[in] _new Whether to add the entity to the list of new entities.
  /// The new here is to indicate whether the entity is new to the entity
  /// component manager. An existing entity can be added when creating a new
  /// view or when rebuilding the view.
  /// \param[in] _compPtrs Pointers to the entity's components
  public: void AddEntityWithComps(const Entity &_entity, const bool _new,
              ComponentTypeTs*... _compPtrs);

  /// \brief Documentation inherited
  public: bool NotifyComponentAddition(const Entity _entity, bool _newEntity,
              const ComponentTypeId _typeId) override;

  /// \brief Documentation inherited
  public: bool NotifyComponentRemoval(const Entity _entity,
              const ComponentTypeId _typeId) override;

  /// \brief Documentation inherited
  public: void Reset() override;

  public: std::unique_ptr<BaseView> Clone() const override;

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
template<typename ...ComponentTypeTs>
View<ComponentTypeTs...>::View()
{
  this->componentTypes = {ComponentTypeTs::typeId...};
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
bool View<ComponentTypeTs...>::HasCachedComponentData(
    const Entity _entity) const
{
  auto cachedComps =
    this->validData.find(_entity) != this->validData.end() ||
    this->invalidData.find(_entity) != this->invalidData.end();
  auto cachedConstComps =
    this->validConstData.find(_entity) != this->validConstData.end() ||
    this->invalidConstData.find(_entity) != this->invalidConstData.end();

  if (cachedComps && !cachedConstComps)
  {
    ignwarn << "Non-const component data is cached for entity " << _entity
      << ", but const component data is not cached." << std::endl;
  }
  else if (cachedConstComps && !cachedComps)
  {
    ignwarn << "Const component data is cached for entity " << _entity
      << ", but non-const component data is not cached." << std::endl;
  }

  return cachedComps && cachedConstComps;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
bool View<ComponentTypeTs...>::RemoveEntity(const Entity _entity)
{
  this->invalidData.erase(_entity);
  this->invalidConstData.erase(_entity);
  this->missingCompTracker.erase(_entity);

  if (!this->HasEntity(_entity) && !this->IsEntityMarkedForAddition(_entity))
    return false;

  this->entities.erase(_entity);
  this->newEntities.erase(_entity);
  this->toRemoveEntities.erase(_entity);
  this->toAddEntities.erase(_entity);
  this->validData.erase(_entity);
  this->validConstData.erase(_entity);

  return true;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
typename View<ComponentTypeTs...>::ConstComponentData
  View<ComponentTypeTs...>::EntityComponentConstData(const Entity _entity) const
{
  return this->validConstData.at(_entity);
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
typename View<ComponentTypeTs...>::ComponentData
  View<ComponentTypeTs...>::EntityComponentData(const Entity _entity)
{
  return this->validData.at(_entity);
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void View<ComponentTypeTs...>::AddEntityWithConstComps(const Entity &_entity,
    const bool _new, const ComponentTypeTs*... _compPtrs)
{
  this->validConstData[_entity] = std::make_tuple(_entity, _compPtrs...);
  this->entities.insert(_entity);
  if (_new)
    this->newEntities.insert(_entity);
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void View<ComponentTypeTs...>::AddEntityWithComps(const Entity &_entity,
    const bool _new, ComponentTypeTs*... _compPtrs)
{
  this->validData[_entity] = std::make_tuple(_entity, _compPtrs...);
  this->entities.insert(_entity);
  if (_new)
    this->newEntities.insert(_entity);
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
bool View<ComponentTypeTs...>::NotifyComponentAddition(const Entity _entity,
    bool _newEntity, const ComponentTypeId _typeId)
{
  // make sure that _typeId is a type required by the view and that _entity is
  // already a part of the view
  if (!this->RequiresComponent(_typeId) ||
      !this->HasCachedComponentData(_entity))
    return false;

  // remove the newly added component type from the missing component types
  // list
  auto missingCompsIter = this->missingCompTracker.find(_entity);
  if (missingCompsIter == this->missingCompTracker.end())
  {
    // the component is already added, so nothing else needs to be done
    return true;
  }
  missingCompsIter->second.erase(_typeId);

  // if the entity now has all components that meet the requirements of the
  // view, then add the entity back to the view
  if (missingCompsIter->second.empty())
  {
    auto nh = this->invalidData.extract(_entity);
    this->validData.insert(std::move(nh));
    auto constCompNh = this->invalidConstData.extract(_entity);
    this->validConstData.insert(std::move(constCompNh));
    this->entities.insert(_entity);
    if (_newEntity)
      this->newEntities.insert(_entity);
    this->missingCompTracker.erase(_entity);
  }

  return true;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
bool View<ComponentTypeTs...>::NotifyComponentRemoval(const Entity _entity,
    const ComponentTypeId _typeId)
{
  // if entity is still marked as to add, remove from the view
  if (this->RequiresComponent(_typeId))
    this->toAddEntities.erase(_entity);

  // make sure that _typeId is a type required by the view and that _entity is
  // already a part of the view
  if (!this->RequiresComponent(_typeId) ||
      !this->HasCachedComponentData(_entity))
    return false;

  // if the component being removed is the first component that causes _entity
  // to be invalid for this view, move _entity from validData to invalidData
  // since _entity should no longer be considered a part of the view
  auto it = this->validData.find(_entity);
  auto constCompIt = this->validConstData.find(_entity);
  if (it != this->validData.end() &&
      constCompIt != this->validConstData.end())
  {
    auto nh = this->validData.extract(it);
    this->invalidData.insert(std::move(nh));
    auto constCompNh = this->validConstData.extract(constCompIt);
    this->invalidConstData.insert(std::move(constCompNh));
    this->entities.erase(_entity);
    this->newEntities.erase(_entity);
  }

  this->missingCompTracker[_entity].insert(_typeId);

  return true;
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
void View<ComponentTypeTs...>::Reset()
{
  // reset all data structures in the BaseView except for componentTypes since
  // the view always requires the types in componentTypes
  this->entities.clear();
  this->newEntities.clear();
  this->toRemoveEntities.clear();
  this->toAddEntities.clear();

  // reset all data structures unique to the templated view
  this->validData.clear();
  this->validConstData.clear();
  this->invalidData.clear();
  this->invalidConstData.clear();
  this->missingCompTracker.clear();
}

//////////////////////////////////////////////////
template<typename ...ComponentTypeTs>
std::unique_ptr<BaseView> View<ComponentTypeTs...>::Clone() const
{
  return std::make_unique<View<ComponentTypeTs...>>(*this);
}
}  // namespace detail
}  // namespace IGNITION_GAZEBO_VERSION_NAMESPACE
}  // namespace gazebo
}  // namespace ignition
#endif
