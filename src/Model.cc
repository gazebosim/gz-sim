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
#include "ignition/gazebo/components/PoseCmd.hh"
#include "ignition/gazebo/components/SelfCollide.hh"
#include "ignition/gazebo/components/SourceFilePath.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/WindMode.hh"
#include "ignition/gazebo/Model.hh"

class ignition::gazebo::ModelPrivate
{
  /// \brief Id of model entity.
  public: Entity id{kNullEntity};
};

using namespace ignition::gazebo;

//////////////////////////////////////////////////
Model::Model(gazebo::Entity _entity)
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
    const std::string &_name)
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Joint());
}

//////////////////////////////////////////////////
Entity Model::LinkByName(const EntityComponentManager &_ecm,
    const std::string &_name)
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Link());
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

