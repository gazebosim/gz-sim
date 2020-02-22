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
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "EntityTree.hh"

namespace ignition::gazebo
{
  class EntityTreePrivate
  {
    /// \brief Model holding all the current entities in the world.
    public: TreeModel treeModel;

    /// \brief True if initialized
    public: bool initialized{false};

    /// \brief World entity
    public: Entity worldEntity{kNullEntity};
  };
}

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
QString entityType(Entity _entity,
    const EntityComponentManager &_ecm)
{
  if (nullptr != _ecm.Component<components::Model>(_entity))
    return QString("model");

  if (nullptr != _ecm.Component<components::Link>(_entity))
    return QString("link");

  if (nullptr != _ecm.Component<components::Joint>(_entity))
    return QString("joint");

  if (nullptr != _ecm.Component<components::Collision>(_entity))
    return QString("collision");

  if (nullptr != _ecm.Component<components::Visual>(_entity))
    return QString("visual");

  if (nullptr != _ecm.Component<components::Light>(_entity))
    return QString("light");

  if (nullptr != _ecm.Component<components::Level>(_entity))
    return QString("level");

  if (nullptr != _ecm.Component<components::Performer>(_entity))
    return QString("performer");

  if (nullptr != _ecm.Component<components::Sensor>(_entity))
    return QString("sensor");

  return QString();
}

/////////////////////////////////////////////////
TreeModel::TreeModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
void TreeModel::AddEntity(unsigned int _entity, const QString &_entityName,
    unsigned int _parentEntity, const QString &_type)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("TreeModel::AddEntity");
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
  entityItem->setData(_entityName, this->roleNames().key("entityName"));
  entityItem->setData(QString::number(_entity),
      this->roleNames().key("entity"));
  entityItem->setData(_type, this->roleNames().key("type"));

  // TODO(louise) Update icons when available
  auto icon = _type;
  if (_type.isEmpty() ||
      _type == "light" ||
      _type == "level" ||
      _type == "sensor" ||
      _type == "performer")
  {
    icon = "visual";
  }
  entityItem->setData(QUrl("qrc:/Gazebo/images/" + icon + ".png"),
      this->roleNames().key("icon"));

  parentItem->appendRow(entityItem);

  this->entityItems[_entity] = entityItem;
}

/////////////////////////////////////////////////
void TreeModel::RemoveEntity(unsigned int _entity)
{
  IGN_PROFILE("TreeModel::RemoveEntity");
  QStandardItem *item{nullptr};
  auto itemIt = this->entityItems.find(_entity);
  if (itemIt != this->entityItems.end())
  {
    item = itemIt->second;
  }

  if (nullptr == item)
  {
    return;
  }

  // Remove all children from our custom map
  std::function<void(const QStandardItem *)> removeChildren =
      [&](const QStandardItem *_item)
  {
    for (int i = 0; i < _item->rowCount(); ++i)
    {
      auto childItem = _item->child(i);
      removeChildren(childItem);
      this->entityItems.erase(childItem->data(
          this->roleNames().key("entity")).toUInt());
    }
  };
  this->entityItems.erase(_entity);
  removeChildren(item);

  // Remove from the view
  if (nullptr == item->parent())
    this->removeRow(item->row());
  else
    item->parent()->removeRow(item->row());
}

/////////////////////////////////////////////////
QString TreeModel::EntityType(const QModelIndex &_index) const
{
  QString type;
  QStandardItem *item = this->itemFromIndex(_index);
  if (!item)
    return type;

  QVariant typeVar  = item->data(this->roleNames().key("type"));
  if (!typeVar.isValid())
    return type;

  return typeVar.toString();
}

/////////////////////////////////////////////////
QString TreeModel::ScopedName(const QModelIndex &_index) const
{
  QString scopedName;
  QModelIndex idx = _index;
  while (idx.isValid())
  {
    QVariant v = idx.data();
    if (v.isValid())
    {
      QString str = v.toString();
      if (!str.isEmpty())
      {
        scopedName = scopedName.isEmpty() ? str : str + "::" + scopedName;
      }
    }
    idx = idx.parent();
  }
  return scopedName;
}

/////////////////////////////////////////////////
unsigned int TreeModel::EntityId(const QModelIndex &_index) const
{
  Entity entity{kNullEntity};
  QStandardItem *item = this->itemFromIndex(_index);
  if (!item)
    return entity;

  QVariant entityVar  = item->data(this->roleNames().key("entity"));
  if (!entityVar.isValid())
    return entity;

  return entityVar.toUInt();
}

/////////////////////////////////////////////////
QHash<int, QByteArray> TreeModel::roleNames() const
{
  return {std::pair(100, "entityName"),
          std::pair(101, "entity"),
          std::pair(102, "icon"),
          std::pair(103, "type")};
}

/////////////////////////////////////////////////
EntityTree::EntityTree()
  : GuiSystem(), dataPtr(std::make_unique<EntityTreePrivate>())
{
  // Connect model
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
     "EntityTreeModel", &this->dataPtr->treeModel);
}

/////////////////////////////////////////////////
EntityTree::~EntityTree() = default;

/////////////////////////////////////////////////
void EntityTree::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Entity tree";

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void EntityTree::Update(const UpdateInfo &, EntityComponentManager &_ecm)
{
  IGN_PROFILE("EntityTree::Update");
  // Treat all pre-existent entities as new at startup
  if (!this->dataPtr->initialized)
  {
    _ecm.Each<components::Name>(
      [&](const Entity &_entity,
          const components::Name *_name)->bool
    {
      auto worldComp = _ecm.Component<components::World>(_entity);
      if (worldComp)
      {
        this->dataPtr->worldEntity = _entity;

        // Skipping the world for now to keep the tree shallow
        return true;
      }

      // Parent
      Entity parentEntity{kNullEntity};

      auto parentComp = _ecm.Component<components::ParentEntity>(_entity);
      if (parentComp)
      {
        parentEntity = parentComp->Data();
      }

      // World children are top-level
      if (this->dataPtr->worldEntity != kNullEntity &&
          parentEntity == this->dataPtr->worldEntity)
      {
        parentEntity = kNullEntity;
      }

      QMetaObject::invokeMethod(&this->dataPtr->treeModel, "AddEntity",
          Qt::QueuedConnection,
          Q_ARG(unsigned int, _entity),
          Q_ARG(QString, QString::fromStdString(_name->Data())),
          Q_ARG(unsigned int, parentEntity),
          Q_ARG(QString, entityType(_entity, _ecm)));
      return true;
    });
    this->dataPtr->initialized = true;
  }
  else
  {
    // Requiring a parent entity because we're not adding the world, which is
    // parentless, to the tree
    _ecm.EachNew<components::Name, components::ParentEntity>(
      [&](const Entity &_entity,
          const components::Name *_name,
          const components::ParentEntity *_parentEntity)->bool
    {
      auto parentEntity = _parentEntity->Data();

      // World children are top-level
      if (this->dataPtr->worldEntity != kNullEntity &&
          parentEntity == this->dataPtr->worldEntity)
      {
        parentEntity = kNullEntity;
      }

      QMetaObject::invokeMethod(&this->dataPtr->treeModel, "AddEntity",
          Qt::QueuedConnection,
          Q_ARG(unsigned int, _entity),
          Q_ARG(QString, QString::fromStdString(_name->Data())),
          Q_ARG(unsigned int, parentEntity),
          Q_ARG(QString, entityType(_entity, _ecm)));
      return true;
    });
  }

  _ecm.EachRemoved<components::Name>(
    [&](const Entity &_entity,
        const components::Name *)->bool
  {
    QMetaObject::invokeMethod(&this->dataPtr->treeModel, "RemoveEntity",
        Qt::QueuedConnection,
        Q_ARG(unsigned int, _entity));
    return true;
  });
}

/////////////////////////////////////////////////
void EntityTree::OnEntitySelectedFromQml(unsigned int _entity)
{
  std::vector<Entity> entitySet {_entity};
  auto event = new gui::events::EntitiesSelected(entitySet, true);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      event);
}

/////////////////////////////////////////////////
void EntityTree::DeselectAllEntities()
{
  auto event = new gui::events::DeselectAllEntities(true);
  ignition::gui::App()->sendEvent(
      ignition::gui::App()->findChild<ignition::gui::MainWindow *>(),
      event);
}

/////////////////////////////////////////////////
bool EntityTree::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::EntitiesSelected::kType)
  {
    auto selectedEvent =
        reinterpret_cast<gui::events::EntitiesSelected *>(_event);
    if (selectedEvent && !selectedEvent->Data().empty())
    {
      for (const auto &entity : selectedEvent->Data())
      {
        if (entity == kNullEntity)
          continue;

        QMetaObject::invokeMethod(this->PluginItem(), "onEntitySelectedFromCpp",
            Qt::QueuedConnection, Q_ARG(QVariant,
            QVariant(static_cast<unsigned int>(entity))));
      }
    }
  }
  else if (_event->type() ==
           ignition::gazebo::gui::events::DeselectAllEntities::kType)
  {
    auto deselectAllEvent =
        reinterpret_cast<gui::events::DeselectAllEntities *>(_event);
    if (deselectAllEvent)
    {
      QMetaObject::invokeMethod(this->PluginItem(), "deselectAllEntities",
          Qt::QueuedConnection);
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::EntityTree,
                    ignition::gui::Plugin)
