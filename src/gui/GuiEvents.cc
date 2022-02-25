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

class ignition::gazebo::gui::events::ModelEditorAddEntity::Implementation
{
  /// \brief Custom data map
  public: QMap<QString, QString> data;

  /// \breif Entity added.
  public: QString entity;

  /// \breif Entity type.
  public: QString type;

  /// \breif Parent entity.
  public: ignition::gazebo::Entity parent;
};

class ignition::gazebo::gui::events::VisualPlugin::Implementation
{
  /// \brief Entity to load the visual plugin for
  public: ignition::gazebo::Entity entity;

  /// \brief Sdf element of the visual plugin
  public: sdf::ElementPtr element;
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
ModelEditorAddEntity::ModelEditorAddEntity(QString _entity, QString _type,
    ignition::gazebo::Entity _parent) :
  QEvent(kType), dataPtr(utils::MakeImpl<Implementation>())
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
QMap<QString, QString> &ModelEditorAddEntity::Data()
{
  return this->dataPtr->data;
}

/////////////////////////////////////////////////
VisualPlugin::VisualPlugin(ignition::gazebo::Entity _entity,
    const sdf::ElementPtr &_elem) :
    QEvent(kType), dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->entity = _entity;
  this->dataPtr->element = _elem;
}

/////////////////////////////////////////////////
ignition::gazebo::Entity VisualPlugin::Entity() const
{
  return this->dataPtr->entity;
}

/////////////////////////////////////////////////
sdf::ElementPtr VisualPlugin::Element() const
{
  return this->dataPtr->element;
}
