/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "gz/sim/detail/View.hh"

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace detail
{
//////////////////////////////////////////////////
View::View(const std::set<ComponentTypeId>& _compIds)
{
  this->componentTypes = _compIds;
}

//////////////////////////////////////////////////
const std::vector<const components::BaseComponent *>
    &View::EntityComponentConstData(const Entity _entity) const
{
  return this->validConstData.at(_entity);
}

//////////////////////////////////////////////////
const std::vector<components::BaseComponent *> &View::EntityComponentData(
    const Entity _entity) const
{
  return this->validData.at(_entity);
}

//////////////////////////////////////////////////
bool View::HasCachedComponentData(const Entity _entity) const
{
  auto cachedComps =
    this->validData.find(_entity) != this->validData.end() ||
    this->invalidData.find(_entity) != this->invalidData.end();
  auto cachedConstComps =
    this->validConstData.find(_entity) != this->validConstData.end() ||
    this->invalidConstData.find(_entity) != this->invalidConstData.end();

  if (cachedComps && !cachedConstComps)
  {
    gzwarn << "Non-const component data is cached for entity " << _entity
      << ", but const component data is not cached." << std::endl;
  }
  else if (cachedConstComps && !cachedComps)
  {
    gzwarn << "Const component data is cached for entity " << _entity
      << ", but non-const component data is not cached." << std::endl;
  }

  return cachedComps && cachedConstComps;
}

//////////////////////////////////////////////////
bool View::RemoveEntity(const Entity _entity)
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
bool View::NotifyComponentAddition(const Entity _entity,
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
bool View::NotifyComponentRemoval(const Entity _entity,
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
void View::Reset()
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

}  // namespace detail
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
