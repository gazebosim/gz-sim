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
#include "ignition/gazebo/components/World.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;
using namespace std::chrono_literals;

/// \brief Private data class.
class ignition::gazebo::components::WorldPrivate
{
  /// \brief Copy constructor
  /// \param[in] _data Data to copy.
  public: explicit WorldPrivate(const WorldPrivate &_data)
          : desiredRealTimeFactor(_data.desiredRealTimeFactor),
            maxStep(_data.maxStep)
  {
  }

  /// \brief Constructor.
  /// \param[in] _physics SDF Physics data.
  public: explicit WorldPrivate(const sdf::Physics *_physics)
          : desiredRealTimeFactor(_physics->RealTimeFactor())
  {
    auto dur = std::chrono::duration<double>(_physics->MaxStepSize());
    this->maxStep =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(dur);
  }

  /// \brief The desired real-time factor.
  public: double desiredRealTimeFactor{1.0};

  /// \brief Physics max step duration.
  public: std::chrono::steady_clock::duration maxStep{1ms};
};

//////////////////////////////////////////////////
World::World(const sdf::World *_world)
  : dataPtr(std::make_unique<WorldPrivate>(_world->PhysicsDefault()))
{
}

//////////////////////////////////////////////////
World::World(const World &_world)
  : dataPtr(std::make_unique<WorldPrivate>(*_world.dataPtr))
{
}

//////////////////////////////////////////////////
World::World(World &&_world) noexcept
  : dataPtr(std::move(_world.dataPtr))
{
}

//////////////////////////////////////////////////
World::~World()
{
  // \todo(nkoenig) Add ability to unregister a component type.
}

//////////////////////////////////////////////////
double World::DesiredRealTimeFactor() const
{
  return this->dataPtr->desiredRealTimeFactor;
}

//////////////////////////////////////////////////
void World::SetDesiredRealTimeFactor(const double _factor)
{
  this->dataPtr->desiredRealTimeFactor = _factor;
}

//////////////////////////////////////////////////
std::chrono::steady_clock::duration World::MaxStep() const
{
  return this->dataPtr->maxStep;
}

//////////////////////////////////////////////////
void World::SetMaxStep(const std::chrono::steady_clock::duration _step)
{
  this->dataPtr->maxStep = _step;
}

//////////////////////////////////////////////////
World &World::operator=(World &&_world)
{
  this->dataPtr = std::move(_world.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
World &World::operator=(const World &_world)
{
  this->dataPtr->desiredRealTimeFactor = _world.dataPtr->desiredRealTimeFactor;
  this->dataPtr->maxStep = _world.dataPtr->maxStep;
  return *this;
}
