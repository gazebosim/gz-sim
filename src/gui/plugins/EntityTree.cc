/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/gui/Application.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/gui/plugins/EntityTree.hh"

namespace ignition
{
namespace gazebo::gui
{
  class EntityTreePrivate
  {
    public: TreeModel treeModel;
  };
}
}

using namespace ignition::gazebo::gui;
using namespace ignition::gui;

/////////////////////////////////////////////////
TreeModel::TreeModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
void TreeModel::AddEntity(Entity _entity, const QString &_entityName,
    Entity _parentEntity)
{
  QStandardItem *parentItem{nullptr};

  // Root
  if (_parentEntity == kNullEntity)
  {
    parentItem = this->invisibleRootItem();
  }

  // Nested
  // TODO(louise) There should be a way to easily access these from
  // QStandardItemModel instead of keeping our own map
  auto item = this->entityItems.find(_parentEntity);
  if (item != this->entityItems.end())
  {
    parentItem = item->second;
  }

  if (nullptr == parentItem)
  {
    ignerr << "Failed to find parent entity [" << _parentEntity << "]"
           << std::endl;
    return;
  }

  // New entity item
  auto entityItem = new QStandardItem(_entityName);
  entityItem->setData(QString::number(_entity), Qt::ToolTipRole);

  parentItem->appendRow(entityItem);

  this->entityItems[_entity] = entityItem;
}

/////////////////////////////////////////////////
QHash<int, QByteArray> TreeModel::roleNames() const
{
  return {std::pair(Qt::DisplayRole, "entityName"),
          std::pair(Qt::ToolTipRole, "entity")};
}

/////////////////////////////////////////////////
EntityTree::EntityTree()
  : Plugin(), dataPtr(std::make_unique<EntityTreePrivate>())
{
  // Connect model
  App()->Engine()->rootContext()->setContextProperty("EntityTreeModel",
      &this->dataPtr->treeModel);
}

/////////////////////////////////////////////////
EntityTree::~EntityTree()
{
}

/////////////////////////////////////////////////
void EntityTree::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "EntityTree";

  this->dataPtr->treeModel.AddEntity(1u, "ground_plane");
  this->dataPtr->treeModel.AddEntity(2u, "sun");
  this->dataPtr->treeModel.AddEntity(3u, "link", 1u);
  this->dataPtr->treeModel.AddEntity(4u, "visual", 3u);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::gui::EntityTree,
                    ignition::gui::Plugin)
