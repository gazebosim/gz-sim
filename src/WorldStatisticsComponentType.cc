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
#include <ignition/gazebo/EntityComponentManager.hh>

#include "WorldStatisticsComponentType.hh"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::WorldStatisticsComponentTypePrivate
{
  /// \brief Constructor.
  /// \param[in] _pose Pose data.
  public: explicit WorldStatisticsComponentTypePrivate()
  {
  }

  /// \brief Name of the component.
  public: std::string name{"WorldStatisticsComponent"};

  /// \brief Real time stop watch
  public: ignition::math::Stopwatch realTime;
};

//////////////////////////////////////////////////
WorldStatisticsComponentType::WorldStatisticsComponentType()
: ComponentType(
    EntityComponentManager::ComponentType<WorldStatisticsComponentType>()),
  dataPtr(new WorldStatisticsComponentTypePrivate())
{
}

//////////////////////////////////////////////////
WorldStatisticsComponentType::WorldStatisticsComponentType(
    const WorldStatisticsComponentType &_stats)
  : ComponentType(_stats.TypeId()),
    dataPtr(new WorldStatisticsComponentTypePrivate())
{
  this->dataPtr->realTime = _stats.dataPtr->realTime;
}

//////////////////////////////////////////////////
WorldStatisticsComponentType::WorldStatisticsComponentType(
    WorldStatisticsComponentType &&_stats) noexcept
  : ComponentType(_stats.TypeId()),
    dataPtr(std::move(_stats.dataPtr))
{
}

//////////////////////////////////////////////////
WorldStatisticsComponentType::~WorldStatisticsComponentType()
{
  // \todo(nkoenig) Add ability to unregister a component type.
  // _compMgr.Unregister<ignition::math::Pose3d>(this->Name());
}

//////////////////////////////////////////////////
ignition::math::clock::duration WorldStatisticsComponentType::RealTime() const
{
  return this->dataPtr->realTime.ElapsedRunTime();
}

//////////////////////////////////////////////////
const std::string &WorldStatisticsComponentType::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
WorldStatisticsComponentType &WorldStatisticsComponentType::operator=(
  WorldStatisticsComponentType &&_stats)
{
  this->dataPtr = std::move(_stats.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
WorldStatisticsComponentType &WorldStatisticsComponentType::operator=(
  const WorldStatisticsComponentType &_stats)
{
  this->dataPtr->realTime = _stats.dataPtr->realTime;
  return *this;
}
