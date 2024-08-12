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

#include "gz/sim/gui/GuiEvents.hh"

class gz::sim::gui::events::GuiNewRemovedEntities::Implementation
{
  /// \brief Set of newly created entities
  public: std::set<Entity> newEntities;

  /// \brief Set of recently removed entities
  public: std::set<Entity> removedEntities;
};

class gz::sim::gui::events::NewRemovedEntities::Implementation
{
  /// \brief Set of newly created entities
  public: std::set<Entity> newEntities;

  /// \brief Set of recently removed entities
  public: std::set<Entity> removedEntities;
};

class gz::sim::gui::events::ModelEditorAddEntity::Implementation
{
  /// \brief Custom data map
  public: QMap<QString, QString> data;

  /// \breif Entity added.
  public: QString entity;

  /// \breif Entity type.
  public: QString type;

  /// \breif Parent entity.
  public: gz::sim::Entity parent;
};

class gz::sim::gui::events::VisualPlugins::Implementation
{
  /// \brief Entity to load the visual plugin for
  public: gz::sim::Entity entity;

  /// \brief Sdf plugins for the visual plugin
  public: sdf::Plugins plugins;
};

using namespace gz;
using namespace sim;
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
    gz::sim::Entity _parent) :
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
gz::sim::Entity ModelEditorAddEntity::ParentEntity() const
{
  return this->dataPtr->parent;
}

/////////////////////////////////////////////////
QMap<QString, QString> &ModelEditorAddEntity::Data()
{
  return this->dataPtr->data;
}

/////////////////////////////////////////////////
VisualPlugins::VisualPlugins(gz::sim::Entity _entity,
    const sdf::Plugins &_plugins) :
    QEvent(kType), dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->entity = _entity;
  this->dataPtr->plugins = _plugins;
}

/////////////////////////////////////////////////
gz::sim::Entity VisualPlugins::Entity() const
{
  return this->dataPtr->entity;
}

/////////////////////////////////////////////////
const sdf::Plugins &VisualPlugins::Plugins() const
{
  return this->dataPtr->plugins;
}
