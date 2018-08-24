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

#include "ignition/gazebo/components/ParentEntity.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

class ignition::gazebo::components::ParentEntityPrivate
{
  /// \brief Constructor.
  /// \param[in] _id ParentEntity data.
  public: explicit ParentEntityPrivate(const EntityId &_id)
          : id(_id)
  {
  }

  /// \brief The id data.
  public: EntityId id;
};

//////////////////////////////////////////////////
ParentEntity::ParentEntity(const EntityId &_id)
  : dataPtr(std::make_unique<ParentEntityPrivate>(_id))
{
}

//////////////////////////////////////////////////
ParentEntity::~ParentEntity()
{
}

//////////////////////////////////////////////////
ParentEntity::ParentEntity(const ParentEntity &_parentEntity)
  : dataPtr(std::make_unique<ParentEntityPrivate>(_parentEntity.Id()))
{
}

//////////////////////////////////////////////////
ParentEntity::ParentEntity(ParentEntity &&_parentEntity) noexcept
  : dataPtr(std::move(_parentEntity.dataPtr))
{
}

//////////////////////////////////////////////////
ParentEntity &ParentEntity::operator=(ParentEntity &&_parentEntity)
{
  this->dataPtr = std::move(_parentEntity.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
ParentEntity &ParentEntity::operator=(const ParentEntity &_parentEntity)
{
  this->dataPtr->id = _parentEntity.Id();
  return *this;
}

//////////////////////////////////////////////////
const EntityId &ParentEntity::Id() const
{
  return this->dataPtr->id;
}

