/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "ignition/gazebo/gui/GuiEvents.hh"

class ignition::gazebo::gui::events::GuiNewRemovedEntities::Implementation
{
  /// \brief Set of newly created entities
  public: std::set<Entity> newEntities;

  /// \brief Set of recently removed entities
  public: std::set<Entity> removedEntities;
};

class ignition::gazebo::gui::events::NewRemovedEntities::Implementation
{
  /// \brief Set of newly created entities
  public: std::set<Entity> newEntities;

  /// \brief Set of recently removed entities
  public: std::set<Entity> removedEntities;
};

using namespace ignition;
using namespace gazebo;
using namespace gui;
using namespace events;

/////////////////////////////////////////////////
GuiNewRemovedEntities::GuiNewRemovedEntities(
    const std::set<Entity> &_newEntities,
    const std::set<Entity> &_removedEntities)
    : QEvent(kType), dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->newEntities = _newEntities;
  this->dataPtr->removedEntities = _removedEntities;
}

/////////////////////////////////////////////////
const std::set<Entity> &GuiNewRemovedEntities::NewEntities() const
{
  return this->dataPtr->newEntities;
}

/////////////////////////////////////////////////
const std::set<Entity> &GuiNewRemovedEntities::RemovedEntities() const
{
  return this->dataPtr->removedEntities;
}

/////////////////////////////////////////////////
NewRemovedEntities::NewRemovedEntities(
    const std::set<Entity> &_newEntities,
    const std::set<Entity> &_removedEntities)
    : QEvent(kType), dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->newEntities = _newEntities;
  this->dataPtr->removedEntities = _removedEntities;
}

/////////////////////////////////////////////////
const std::set<Entity> &NewRemovedEntities::NewEntities() const
{
  return this->dataPtr->newEntities;
}

/////////////////////////////////////////////////
const std::set<Entity> &NewRemovedEntities::RemovedEntities() const
{
  return this->dataPtr->removedEntities;
}
