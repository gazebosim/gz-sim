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
#include "ignition/gazebo/components/WorldStatistics.hh"

using namespace ignition;
using namespace gazebo;
using namespace components;

class ignition::gazebo::components::WorldStatisticsPrivate
{
  /// \brief Constructor.
  public: explicit WorldStatisticsPrivate()
  {
  }

  /// \brief The real time
  public: ignition::math::Stopwatch realTime;

  /// \brief The sim time
  public: ignition::math::clock::duration simTime{0};

  /// \brief The number of iterations.
  public: uint64_t iterations{0};
};

//////////////////////////////////////////////////
WorldStatistics::WorldStatistics()
  : dataPtr(std::make_unique<WorldStatisticsPrivate>())
{
}

//////////////////////////////////////////////////
WorldStatistics::WorldStatistics(
    const WorldStatistics &_stats)
  : dataPtr(std::make_unique<WorldStatisticsPrivate>())
{
  this->dataPtr->realTime = _stats.dataPtr->realTime;
}

//////////////////////////////////////////////////
WorldStatistics::WorldStatistics(
    WorldStatistics &&_stats) noexcept
  : dataPtr(std::move(_stats.dataPtr))
{
}

//////////////////////////////////////////////////
WorldStatistics::~WorldStatistics()
{
  // \todo(nkoenig) Add ability to unregister a component type.
}

//////////////////////////////////////////////////
uint64_t WorldStatistics::Iterations() const
{
  return this->dataPtr->iterations;
}

//////////////////////////////////////////////////
void WorldStatistics::AddIterations(const uint64_t _iters)
{
  this->dataPtr->iterations += _iters;
}

//////////////////////////////////////////////////
void WorldStatistics::SetIterations(const uint64_t _iters)
{
  this->dataPtr->iterations = _iters;
}

//////////////////////////////////////////////////
const ignition::math::clock::duration &WorldStatistics::SimTime() const
{
  return this->dataPtr->simTime;
}

//////////////////////////////////////////////////
void WorldStatistics::AddSimTime(
    const ignition::math::clock::duration &_simTime)
{
  this->dataPtr->simTime += _simTime;
}

//////////////////////////////////////////////////
void WorldStatistics::SetSimTime(
    const ignition::math::clock::duration &_simTime)
{
  this->dataPtr->simTime = _simTime;
}

//////////////////////////////////////////////////
const ignition::math::Stopwatch &WorldStatistics::RealTime() const
{
  return this->dataPtr->realTime;
}

//////////////////////////////////////////////////
ignition::math::Stopwatch &WorldStatistics::RealTime()
{
  return this->dataPtr->realTime;
}

//////////////////////////////////////////////////
WorldStatistics &WorldStatistics::operator=(
  WorldStatistics &&_stats)
{
  this->dataPtr = std::move(_stats.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
WorldStatistics &WorldStatistics::operator=(
  const WorldStatistics &_stats)
{
  this->dataPtr->realTime = _stats.dataPtr->realTime;
  return *this;
}
