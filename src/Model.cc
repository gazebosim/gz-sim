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

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "gz/sim/components/SelfCollide.hh"
#include "gz/sim/components/SourceFilePath.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/WindMode.hh"
#include "gz/sim/Model.hh"

class gz::sim::ModelPrivate
{
  /// \brief Id of model entity.
  public: Entity id{kNullEntity};
};

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
Model::Model(sim::Entity _entity)
  : dataPtr(std::make_unique<ModelPrivate>())
{
  this->dataPtr->id = _entity;
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
Entity Model::Entity() const
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
bool Model::Static(const EntityComponentManager &_ecm) const
{
  auto comp = _ecm.Component<components::Static>(this->dataPtr->id);
  if (comp)
    return comp->Data();

  return false;
}

//////////////////////////////////////////////////
bool Model::SelfCollide(const EntityComponentManager &_ecm) const
{
  auto comp = _ecm.Component<components::SelfCollide>(this->dataPtr->id);
  if (comp)
    return comp->Data();

  return false;
}

//////////////////////////////////////////////////
bool Model::WindMode(const EntityComponentManager &_ecm) const
{
  auto comp = _ecm.Component<components::WindMode>(this->dataPtr->id);
  if (comp)
    return comp->Data();

  return false;
}

//////////////////////////////////////////////////
std::string Model::SourceFilePath(const EntityComponentManager &_ecm) const
{
  auto comp = _ecm.Component<components::SourceFilePath>(this->dataPtr->id);
  if (comp)
    return comp->Data();

  return "";
}

//////////////////////////////////////////////////
Entity Model::JointByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Joint());
}

//////////////////////////////////////////////////
Entity Model::LinkByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Link());
}

//////////////////////////////////////////////////
Entity Model::ModelByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Model());
}

//////////////////////////////////////////////////
std::vector<Entity> Model::Joints(const EntityComponentManager &_ecm) const
{
  return _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Joint());
}

//////////////////////////////////////////////////
std::vector<Entity> Model::Links(const EntityComponentManager &_ecm) const
{
  return _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Link());
}

//////////////////////////////////////////////////
std::vector<Entity> Model::Models(const EntityComponentManager &_ecm) const
{
  return _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Model());
}

//////////////////////////////////////////////////
uint64_t Model::JointCount(const EntityComponentManager &_ecm) const
{
  return this->Joints(_ecm).size();
}

//////////////////////////////////////////////////
uint64_t Model::LinkCount(const EntityComponentManager &_ecm) const
{
  return this->Links(_ecm).size();
}

//////////////////////////////////////////////////
uint64_t Model::ModelCount(const EntityComponentManager &_ecm) const
{
  return this->Models(_ecm).size();
}

//////////////////////////////////////////////////
void Model::SetWorldPoseCmd(EntityComponentManager &_ecm,
    const math::Pose3d &_pose)
{
  auto poseCmdComp = _ecm.Component<components::WorldPoseCmd>(
      this->dataPtr->id);
  if (!poseCmdComp)
  {
    _ecm.CreateComponent(this->dataPtr->id, components::WorldPoseCmd(_pose));
  }
  else
  {
    poseCmdComp->SetData(_pose,
        [](const math::Pose3d &, const math::Pose3d &){return false;});
    _ecm.SetChanged(this->dataPtr->id,
        components::WorldPoseCmd::typeId, ComponentState::OneTimeChange);
  }
}

//////////////////////////////////////////////////
Entity Model::CanonicalLink(const EntityComponentManager &_ecm) const
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::CanonicalLink());
}
