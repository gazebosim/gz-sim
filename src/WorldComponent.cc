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
#include "ignition/gazebo/WorldComponent.hh"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::WorldComponentPrivate
{
  /// \brief Constructor.
  /// \param[in] _pose Pose data.
  public: explicit WorldComponentPrivate(const std::string &_name)
          : name(_name)
  {
  }

  /// \brief Name of the component.
  public: std::string componentName{"WorldComponent"};

  /// \brief Name of the world.
  public: std::string name{"default"};
};

//////////////////////////////////////////////////
WorldComponent::WorldComponent(const std::string &_name)
  : dataPtr(new WorldComponentPrivate(_name))
{
}

//////////////////////////////////////////////////
WorldComponent::WorldComponent(const WorldComponent &_world)
  : dataPtr(new WorldComponentPrivate(_world.Name()))
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
WorldComponent &WorldComponent::operator=(WorldComponent &&_world)
{
  this->dataPtr = std::move(_world.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
WorldComponent &WorldComponent::operator=(const WorldComponent &_world)
{
  this->dataPtr->name = _world.dataPtr->name;
  return *this;
}
