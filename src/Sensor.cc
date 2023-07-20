/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"

#include "gz/sim/Sensor.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;


class gz::sim::Sensor::Implementation
{
  /// \brief Id of sensor entity.
  public: sim::Entity id{kNullEntity};
};

//////////////////////////////////////////////////
Sensor::Sensor(sim::Entity _entity)
  : dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->id = _entity;
}

//////////////////////////////////////////////////
sim::Entity Sensor::Entity() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
void Sensor::ResetEntity(sim::Entity _newEntity)
{
  this->dataPtr->id = _newEntity;
}

//////////////////////////////////////////////////
bool Sensor::Valid(const EntityComponentManager &_ecm) const
{
  return nullptr != _ecm.Component<components::Sensor>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::string> Sensor::Name(const EntityComponentManager &_ecm)
    const
{
  return _ecm.ComponentData<components::Name>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Sensor::Pose(
    const EntityComponentManager &_ecm) const
{
  auto pose = _ecm.Component<components::Pose>(this->dataPtr->id);

  if (!pose)
    return std::nullopt;

  return std::optional<math::Pose3d>(pose->Data());
}

//////////////////////////////////////////////////
std::optional<std::string> Sensor::Topic(
    const EntityComponentManager &_ecm) const
{
  auto topic = _ecm.Component<components::SensorTopic>(this->dataPtr->id);

  if (!topic)
    return std::nullopt;

  return std::optional<std::string>(topic->Data());
}

//////////////////////////////////////////////////
std::optional<Entity> Sensor::Parent(const EntityComponentManager &_ecm) const
{
  auto parent = _ecm.Component<components::ParentEntity>(this->dataPtr->id);

  if (!parent)
    return std::nullopt;

  return std::optional<sim::Entity>(parent->Data());
}
