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

#include "ignition/gazebo/components/ChildEntity.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

class ignition::gazebo::components::ChildEntityPrivate
{
  /// \brief Constructor.
  /// \param[in] _id ChildEntity data.
  public: explicit ChildEntityPrivate(const EntityId &_id)
          : id(_id)
  {
  }

  /// \brief The id data.
  public: EntityId id;
};

//////////////////////////////////////////////////
ChildEntity::ChildEntity(const EntityId &_id)
  : dataPtr(std::make_unique<ChildEntityPrivate>(_id))
{
}

//////////////////////////////////////////////////
ChildEntity::~ChildEntity()
{
}

//////////////////////////////////////////////////
ChildEntity::ChildEntity(const ChildEntity &_childEntity)
  : dataPtr(std::make_unique<ChildEntityPrivate>(_childEntity.Id()))
{
}

//////////////////////////////////////////////////
ChildEntity::ChildEntity(ChildEntity &&_childEntity) noexcept
  : dataPtr(std::move(_childEntity.dataPtr))
{
}

//////////////////////////////////////////////////
ChildEntity &ChildEntity::operator=(ChildEntity &&_childEntity)
{
  this->dataPtr = std::move(_childEntity.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
ChildEntity &ChildEntity::operator=(const ChildEntity &_childEntity)
{
  this->dataPtr->id = _childEntity.Id();
  return *this;
}

//////////////////////////////////////////////////
const EntityId &ChildEntity::Id() const
{
  return this->dataPtr->id;
}

