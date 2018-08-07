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
#include "WorldStatisticsComponent.hh"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::WorldStatisticsComponentPrivate
{
  /// \brief Constructor.
  /// \param[in] _pose Pose data.
  public: explicit WorldStatisticsComponentPrivate()
  {
  }

  /// \brief Name of the component.
  public: std::string name{"WorldStatisticsComponent"};

  /// \brief The real time
  public: ignition::math::Stopwatch realTime;

  /// \brief The sim time
  public: ignition::math::clock::duration simTime{0};

  /// \brief The number of iterations.
  public: uint64_t iterations{0};
};

//////////////////////////////////////////////////
WorldStatisticsComponent::WorldStatisticsComponent()
  : dataPtr(new WorldStatisticsComponentPrivate())
{
}

//////////////////////////////////////////////////
WorldStatisticsComponent::WorldStatisticsComponent(
    const WorldStatisticsComponent &_stats)
  : dataPtr(new WorldStatisticsComponentPrivate())
{
  this->dataPtr->realTime = _stats.dataPtr->realTime;
}

//////////////////////////////////////////////////
WorldStatisticsComponent::WorldStatisticsComponent(
    WorldStatisticsComponent &&_stats) noexcept
  : dataPtr(std::move(_stats.dataPtr))
{
}

//////////////////////////////////////////////////
WorldStatisticsComponent::~WorldStatisticsComponent()
{
  // \todo(nkoenig) Add ability to unregister a component type.
  // _compMgr.Unregister<ignition::math::Pose3d>(this->Name());
}

//////////////////////////////////////////////////
uint64_t WorldStatisticsComponent::Iterations() const
{
  return this->dataPtr->iterations;
}

//////////////////////////////////////////////////
void WorldStatisticsComponent::AddIterations(const uint64_t _iters)
{
  this->dataPtr->iterations += _iters;
}

//////////////////////////////////////////////////
void WorldStatisticsComponent::SetIterations(const uint64_t _iters)
{
  this->dataPtr->iterations = _iters;
}

//////////////////////////////////////////////////
const ignition::math::clock::duration &WorldStatisticsComponent::SimTime() const
{
  return this->dataPtr->simTime;
}

//////////////////////////////////////////////////
void WorldStatisticsComponent::AddSimTime(
    const ignition::math::clock::duration &_simTime)
{
  this->dataPtr->simTime += _simTime;
}

//////////////////////////////////////////////////
void WorldStatisticsComponent::SetSimTime(
    const ignition::math::clock::duration &_simTime)
{
  this->dataPtr->simTime = _simTime;
}

//////////////////////////////////////////////////
const ignition::math::Stopwatch &WorldStatisticsComponent::RealTime() const
{
  return this->dataPtr->realTime;
}

//////////////////////////////////////////////////
ignition::math::Stopwatch &WorldStatisticsComponent::RealTime()
{
  return this->dataPtr->realTime;
}

//////////////////////////////////////////////////
const std::string &WorldStatisticsComponent::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
WorldStatisticsComponent &WorldStatisticsComponent::operator=(
  WorldStatisticsComponent &&_stats)
{
  this->dataPtr = std::move(_stats.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
WorldStatisticsComponent &WorldStatisticsComponent::operator=(
  const WorldStatisticsComponent &_stats)
{
  this->dataPtr->realTime = _stats.dataPtr->realTime;
  return *this;
}
