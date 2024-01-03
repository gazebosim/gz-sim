/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>

#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/Atmosphere.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/MagneticField.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/SphericalCoordinates.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/World.hh"

class gz::sim::WorldPrivate
{
  /// \brief Id of world entity.
  public: Entity id{kNullEntity};
};

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
World::World(sim::Entity _entity)
  : dataPtr(std::make_unique<WorldPrivate>())
{
  this->dataPtr->id = _entity;
}

/////////////////////////////////////////////////
World::World(const World &_world)
  : dataPtr(std::make_unique<WorldPrivate>(*_world.dataPtr))
{
}

/////////////////////////////////////////////////
World::World(World &&_world) noexcept = default;

//////////////////////////////////////////////////
World::~World() = default;

/////////////////////////////////////////////////
World &World::operator=(const World &_world)
{
  *this->dataPtr = (*_world.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
World &World::operator=(World &&_world) noexcept = default;

//////////////////////////////////////////////////
Entity World::Entity() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
bool World::Valid(const EntityComponentManager &_ecm) const
{
  return nullptr != _ecm.Component<components::World>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::string> World::Name(const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::Name>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<sdf::Atmosphere> World::Atmosphere(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::Atmosphere>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<math::SphericalCoordinates> World::SphericalCoordinates(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::SphericalCoordinates>(
      this->dataPtr->id);
}

//////////////////////////////////////////////////
void World::SetSphericalCoordinates(EntityComponentManager &_ecm,
    const math::SphericalCoordinates &_sphericalCoordinates)
{
  auto sphericalCoordinatesComp =
      _ecm.Component<components::SphericalCoordinates>(this->dataPtr->id);
  if (!sphericalCoordinatesComp)
  {
    _ecm.CreateComponent(this->dataPtr->id,
        components::SphericalCoordinates(_sphericalCoordinates));
    return;
  }

  sphericalCoordinatesComp->SetData(_sphericalCoordinates,
      [](const math::SphericalCoordinates &,
         const math::SphericalCoordinates &){return false;});
  _ecm.SetChanged(this->dataPtr->id,
      components::SphericalCoordinates::typeId, ComponentState::OneTimeChange);
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> World::Gravity(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::Gravity>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> World::MagneticField(
    const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::MagneticField>(this->dataPtr->id);
}

//////////////////////////////////////////////////
Entity World::LightByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  // Can't use components::Light in EntityByComponents, see
  // https://github.com/gazebosim/gz-sim/issues/376
  auto entities = _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name));

  for (const auto &entity : entities)
  {
    if (_ecm.Component<components::Light>(entity))
      return entity;
  }
  return kNullEntity;
}

//////////////////////////////////////////////////
Entity World::ActorByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  // Can't use components::Actor in EntityByComponents, see
  // https://github.com/gazebosim/gz-sim/issues/376
  auto entities = _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name));

  for (const auto &entity : entities)
  {
    if (_ecm.Component<components::Actor>(entity))
      return entity;
  }
  return kNullEntity;
}

//////////////////////////////////////////////////
Entity World::ModelByName(const EntityComponentManager &_ecm,
    const std::string &_name) const
{
  return _ecm.EntityByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Name(_name),
      components::Model());
}

//////////////////////////////////////////////////
std::vector<Entity> World::Lights(const EntityComponentManager &_ecm) const
{
  // Can't use components::Light in EntityByComponents, see
  // https://github.com/gazebosim/gz-sim/issues/376
  auto entities = _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id));

  std::vector<sim::Entity> result;
  for (const auto &entity : entities)
  {
    if (_ecm.Component<components::Light>(entity))
      result.push_back(entity);
  }
  return result;
}

//////////////////////////////////////////////////
std::vector<Entity> World::Actors(const EntityComponentManager &_ecm) const
{
  // Can't use components::Actor in EntityByComponents, see
  // https://github.com/gazebosim/gz-sim/issues/376
  auto entities = _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id));

  std::vector<sim::Entity> result;
  for (const auto &entity : entities)
  {
    if (_ecm.Component<components::Actor>(entity))
      result.push_back(entity);
  }
  return result;
}

//////////////////////////////////////////////////
std::vector<Entity> World::Models(const EntityComponentManager &_ecm) const
{
  return _ecm.EntitiesByComponents(
      components::ParentEntity(this->dataPtr->id),
      components::Model());
}

//////////////////////////////////////////////////
uint64_t World::LightCount(const EntityComponentManager &_ecm) const
{
  return this->Lights(_ecm).size();
}

//////////////////////////////////////////////////
uint64_t World::ActorCount(const EntityComponentManager &_ecm) const
{
  return this->Actors(_ecm).size();
}

//////////////////////////////////////////////////
uint64_t World::ModelCount(const EntityComponentManager &_ecm) const
{
  return this->Models(_ecm).size();
}
