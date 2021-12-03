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

#include <QMap>

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

class ignition::gazebo::gui::events::ModelEditorAddEntity::Implementation
{
  /// \brief Name of entity to add
  public: QString entity;

  /// \brief Type of entity to add
  public: QString type;

  /// \brief parent of entity to add
  public: ignition::gazebo::Entity parent;

  /// \brief Additional data needed to create specific entities
  public: QMap<QString, QString> data;
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

/////////////////////////////////////////////////
ModelEditorAddEntity::ModelEditorAddEntity(
    QString _entity, QString _type, ignition::gazebo::Entity _parent)
    : QEvent(kType), dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->entity = _entity;
  this->dataPtr->type = _type;
  this->dataPtr->parent = _parent;
}

/////////////////////////////////////////////////
QString ModelEditorAddEntity::Entity() const
{
  return this->dataPtr->entity;
}

/////////////////////////////////////////////////
QString ModelEditorAddEntity::EntityType() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
ignition::gazebo::Entity ModelEditorAddEntity::ParentEntity() const
{
  return this->dataPtr->parent;
}

/////////////////////////////////////////////////
bool ModelEditorAddEntity::HasData(const QString &_key) const
{
  return this->dataPtr->data.contains(_key);
}

/////////////////////////////////////////////////
std::unordered_map<std::string, std::string>
ModelEditorAddEntity::Data() const
{
    std::unordered_map<std::string, std::string> data;
    for (auto key : this->dataPtr->data.toStdMap())
    {
      data[key.first.toStdString()] = key.second.toStdString();
    }
    return data;
}

/////////////////////////////////////////////////
QString ModelEditorAddEntity::Data(const QString &_key) const
{
  return this->dataPtr->data[_key];
}

/////////////////////////////////////////////////
void ModelEditorAddEntity::SetData(const QString &_key, const QString &_value)
{
  this->dataPtr->data[_key] = _value;
}
