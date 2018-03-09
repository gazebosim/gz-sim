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
#include <memory>
#include "ignition/gazebo/Entity.hh"

using namespace ignition::gazebo;

/// \brief Private data class
class ignition::gazebo::EntityPrivate
{
  /// \brief ID of entity
  public: EntityId id = kNullEntity;
};

/////////////////////////////////////////////////
Entity::Entity()
: dataPtr(new EntityPrivate())
{
}

/////////////////////////////////////////////////
Entity::Entity(const Entity &_entity)
: dataPtr(new EntityPrivate())
{
  this->dataPtr->id = _entity.Id();
}

/////////////////////////////////////////////////
Entity::Entity(Entity &&_entity)
: dataPtr(std::move(_entity.dataPtr))
{
  _entity.dataPtr = new EntityPrivate();
}

/////////////////////////////////////////////////
Entity::~Entity()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
Entity &Entity::operator=(Entity &&_entity)
{
  // Delete the existing dataPtr, if it exists
  if (this->dataPtr)
    delete this->dataPtr;

  this->dataPtr = std::move(_entity.dataPtr);
  _entity.dataPtr = new EntityPrivate();

  return *this;
}

/////////////////////////////////////////////////
bool Entity::operator==(const Entity &_entity) const
{
  return this->dataPtr->id == _entity.dataPtr->id;
}

/////////////////////////////////////////////////
EntityId Entity::Id() const
{
  return this->dataPtr->id;
}
