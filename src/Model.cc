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

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/Model.hh"

class ignition::gazebo::ModelPrivate
{
  /// \brief Id of model entity.
  public: EntityId id{kNullEntity};
};

using namespace ignition::gazebo;

//////////////////////////////////////////////////
Model::Model(EntityId _id)
  : dataPtr(std::make_unique<ModelPrivate>())
{
  this->dataPtr->id = _id;
}

/////////////////////////////////////////////////
Model::Model(const Model &_model)
  : dataPtr(std::make_unique<ModelPrivate>(*_model.dataPtr))
{
}

/////////////////////////////////////////////////
Model::Model(Model &&_model) noexcept = default;

//////////////////////////////////////////////////
Model::~Model() = default;

/////////////////////////////////////////////////
Model &Model::operator=(const Model &_model)
{
  *this->dataPtr = (*_model.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
Model &Model::operator=(Model &&_model) noexcept = default;

//////////////////////////////////////////////////
EntityId Model::Id() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool Model::Valid(const EntityComponentManager &_ecm) const
{
  return nullptr != _ecm.Component<components::Model>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::string Model::Name(const EntityComponentManager &_ecm) const
{
  auto comp = _ecm.Component<components::Name>(this->dataPtr->id);
  if (comp)
    return comp->Data();

  return "";
}

//////////////////////////////////////////////////
EntityId Model::JointByName(const EntityComponentManager &_ecm,
    const std::string &_name)
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Joint());
}

//////////////////////////////////////////////////
EntityId Model::LinkByName(const EntityComponentManager &_ecm,
    const std::string &_name)
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Link());
}

