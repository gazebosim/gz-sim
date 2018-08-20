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
#include <sdf/World.hh>
#include <sdf/Physics.hh>
#include "ignition/gazebo/WorldComponent.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

/// \brief Private data class.
class ignition::gazebo::WorldComponentPrivate
{
  /// \brief Copy constructor
  /// \param[in] _data Data to copy.
  public: explicit WorldComponentPrivate(const WorldComponentPrivate &_data)
          : name(_data.name),
            desiredRealTimeFactor(_data.desiredRealTimeFactor),
            maxStep(_data.maxStep)
  {
  }

  /// \brief Constructor.
  /// \param[in] _name Name of the world
  /// \param[in] _physics SDF Physics data.
  public: WorldComponentPrivate(const std::string &_name,
              const sdf::Physics *_physics)
          : name(_name),
            desiredRealTimeFactor(_physics->RealTimeFactor())
  {
    auto dur = std::chrono::duration<double>(_physics->MaxStepSize());
    this->maxStep =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(dur);
  }

  /// \brief Name of the component.
  public: std::string componentName{"WorldComponent"};

  /// \brief Name of the world.
  public: std::string name{"default"};

  /// \brief The desired real-time factor.
  public: double desiredRealTimeFactor{1.0};

  /// \brief Physics max step duration.
  public: std::chrono::steady_clock::duration maxStep{1ms};
};

//////////////////////////////////////////////////
WorldComponent::WorldComponent(const sdf::World *_world)
  : dataPtr(new WorldComponentPrivate(
        _world->Name(), _world->PhysicsDefault()))
{
}

//////////////////////////////////////////////////
WorldComponent::WorldComponent(const WorldComponent &_world)
  : dataPtr(new WorldComponentPrivate(*_world.dataPtr))
{
}

//////////////////////////////////////////////////
WorldComponent::WorldComponent(WorldComponent &&_world) noexcept
  : dataPtr(std::move(_world.dataPtr))
{
}

//////////////////////////////////////////////////
WorldComponent::~WorldComponent()
{
  // \todo(nkoenig) Add ability to unregister a component type.
}

//////////////////////////////////////////////////
const std::string &WorldComponent::ComponentName() const
{
  return this->dataPtr->componentName;
}

//////////////////////////////////////////////////
const std::string &WorldComponent::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
double WorldComponent::DesiredRealTimeFactor() const
{
  return this->dataPtr->desiredRealTimeFactor;
}

//////////////////////////////////////////////////
void WorldComponent::SetDesiredRealTimeFactor(const double _factor)
{
  this->dataPtr->desiredRealTimeFactor = _factor;
}

//////////////////////////////////////////////////
std::chrono::steady_clock::duration WorldComponent::MaxStep() const
{
  return this->dataPtr->maxStep;
}

//////////////////////////////////////////////////
void WorldComponent::SetMaxStep(const std::chrono::steady_clock::duration _step)
{
  this->dataPtr->maxStep = _step;
}

//////////////////////////////////////////////////
WorldComponent &WorldComponent::operator=(WorldComponent &&_world)
{
  this->dataPtr = std::move(_world.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
WorldComponent &WorldComponent::operator=(const WorldComponent &_world)
{
  this->dataPtr->name = _world.dataPtr->name;
  this->dataPtr->desiredRealTimeFactor = _world.dataPtr->desiredRealTimeFactor;
  this->dataPtr->maxStep = _world.dataPtr->maxStep;
  return *this;
}
