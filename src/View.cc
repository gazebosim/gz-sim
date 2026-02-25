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
const std::vector<Entity> &View::ValidEntities() const
{
  return this->validEntities;
}

//////////////////////////////////////////////////
const std::vector<View::ComponentData> &View::ValidComponentData() const
{
  return this->validData;
}

//////////////////////////////////////////////////
const std::vector<View::ConstComponentData>
    &View::ValidConstComponentData() const
{
  return this->validConstData;
}

//////////////////////////////////////////////////
const std::vector<const components::BaseComponent *>
    &View::EntityComponentConstData(const Entity _entity) const
{
  auto it = std::lower_bound(this->validEntities.begin(),
                             this->validEntities.end(), _entity);
  if (it != this->validEntities.end() && *it == _entity)
  {
    return this->validConstData.at(
        std::distance(this->validEntities.begin(), it));
  }
  return this->validConstData.at(0);  // Should not happen based on API contract
}

//////////////////////////////////////////////////
const std::vector<components::BaseComponent *> &View::EntityComponentData(
    const Entity _entity) const
{
  auto it = std::lower_bound(this->validEntities.begin(),
                             this->validEntities.end(), _entity);
  if (it != this->validEntities.end() && *it == _entity)
  {
    return this->validData.at(std::distance(this->validEntities.begin(), it));
  }
  return this->validData.at(0);  // Should not happen based on API contract
}

//////////////////////////////////////////////////
bool View::HasCachedComponentData(const Entity _entity) const
{
  auto it = std::lower_bound(this->validEntities.begin(),
                             this->validEntities.end(), _entity);
  if (it != this->validEntities.end() && *it == _entity)
  {
    auto index = std::distance(this->validEntities.begin(), it);
    return !this->validData[index].empty() &&
           !this->validConstData[index].empty();
  }

  return std::binary_search(this->invalidEntities.begin(),
                            this->invalidEntities.end(), _entity);
}

//////////////////////////////////////////////////
bool View::RemoveEntity(const Entity _entity)
{
  bool removed = false;

  auto invalidIt = std::lower_bound(this->invalidEntities.begin(),
                                    this->invalidEntities.end(), _entity);
  if (invalidIt != this->invalidEntities.end() && *invalidIt == _entity)
  {
    auto index = std::distance(this->invalidEntities.begin(), invalidIt);
    this->invalidEntities.erase(invalidIt);
    this->invalidData.erase(this->invalidData.begin() + index);
    this->invalidConstData.erase(this->invalidConstData.begin() + index);
    this->missingCompTracker.erase(_entity);
    removed = true;
  }

  auto validIt = std::lower_bound(this->validEntities.begin(),
                                  this->validEntities.end(), _entity);
  if (validIt != this->validEntities.end() && *validIt == _entity)
  {
    auto index = std::distance(this->validEntities.begin(), validIt);
    this->validEntities.erase(validIt);
    this->validData.erase(this->validData.begin() + index);
    this->validConstData.erase(this->validConstData.begin() + index);
    removed = true;
  }

  if (this->HasEntity(_entity) || this->IsEntityMarkedForAddition(_entity))
  {
    this->entities.erase(_entity);
    this->newEntities.erase(_entity);
    this->toRemoveEntities.erase(_entity);
    this->toAddEntities.erase(_entity);
    removed = true;
  }

  return removed;
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
    // Move from invalid to valid
    auto invalidIt = std::lower_bound(this->invalidEntities.begin(),
                                      this->invalidEntities.end(), _entity);
    if (invalidIt != this->invalidEntities.end() && *invalidIt == _entity)
    {
      auto index = std::distance(this->invalidEntities.begin(), invalidIt);
      
      // Extract data
      auto data = std::move(this->invalidData[index]);
      auto constData = std::move(this->invalidConstData[index]);

      // Remove from invalid
      this->invalidEntities.erase(invalidIt);
      this->invalidData.erase(this->invalidData.begin() + index);
      this->invalidConstData.erase(this->invalidConstData.begin() + index);

      // Insert into valid (maintaining sort)
      auto validIt = std::lower_bound(this->validEntities.begin(),
                                      this->validEntities.end(), _entity);
      auto validIndex = std::distance(this->validEntities.begin(), validIt);
      
      this->validEntities.insert(validIt, _entity);
      this->validData.insert(this->validData.begin() + validIndex,
                             std::move(data));
      this->validConstData.insert(this->validConstData.begin() + validIndex,
                                  std::move(constData));
    }
    
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
  auto validIt = std::lower_bound(this->validEntities.begin(),
                                  this->validEntities.end(), _entity);

  if (validIt != this->validEntities.end() && *validIt == _entity)
  {
    auto index = std::distance(this->validEntities.begin(), validIt);

    // Extract data
    auto data = std::move(this->validData[index]);
    auto constData = std::move(this->validConstData[index]);

    // Remove from valid
    this->validEntities.erase(validIt);
    this->validData.erase(this->validData.begin() + index);
    this->validConstData.erase(this->validConstData.begin() + index);

    // Insert into invalid (maintaining sort)
    auto invalidIt = std::lower_bound(this->invalidEntities.begin(),
                                      this->invalidEntities.end(), _entity);
    auto invalidIndex = std::distance(this->invalidEntities.begin(), invalidIt);

    this->invalidEntities.insert(invalidIt, _entity);
    this->invalidData.insert(this->invalidData.begin() + invalidIndex,
                             std::move(data));
    this->invalidConstData.insert(this->invalidConstData.begin() + invalidIndex,
                                  std::move(constData));

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
  this->validEntities.clear();
  this->validData.clear();
  this->validConstData.clear();
  this->invalidEntities.clear();
  this->invalidData.clear();
  this->invalidConstData.clear();
  this->missingCompTracker.clear();
}

}  // namespace detail
}  // namespace GZ_SIM_VERSION_NAMESPACE
}  // namespace sim
}  // namespace gz
