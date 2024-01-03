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

#include "EntityTree.hh"

#include <algorithm>
#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/Profiler.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/Level.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Performer.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/gui/GuiEvents.hh"

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Primitives.hh"

namespace gz::sim
{
  class EntityTreePrivate
  {
    /// \brief Model holding all the current entities in the world.
    public: TreeModel treeModel;

    /// \brief True if initialized
    public: bool initialized{false};

    /// \brief World entity
    public: Entity worldEntity{kNullEntity};

    /// \brief List of new entities from a gui event
    public: std::set<Entity> newEntities;

    /// \brief List of removed entities from a gui event
    public: std::set<Entity> removedEntities;

    /// \brief Mutex to protect gui event and system upate call race conditions
    /// for newEntities and removedEntities
    public: std::mutex newRemovedEntityMutex;
  };
}

using namespace gz;
using namespace sim;

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

  if (nullptr != _ecm.Component<components::Actor>(_entity))
    return QString("actor");

  return QString();
}

/////////////////////////////////////////////////
TreeModel::TreeModel() : QStandardItemModel()
{
  qRegisterMetaType<Entity>("Entity");
}

/////////////////////////////////////////////////
void TreeModel::AddEntity(Entity _entity, const QString &_entityName,
    Entity _parentEntity, const QString &_type)
{
  GZ_PROFILE_THREAD_NAME("Qt thread");
  GZ_PROFILE("TreeModel::AddEntity");
  QStandardItem *parentItem{nullptr};

  // check if entity has already been added or not.
  // This could happen because we get new and removed entity updates from both
  // the ECM and GUI events.
  if (this->entityItems.find(_entity) != this->entityItems.end())
    return;

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
    this->pendingEntities.push_back(
      {_entity, _entityName, _parentEntity, _type});
    return;
  }

  // New entity item
  auto entityItem = new QStandardItem(_entityName);
  entityItem->setData(_entityName, this->roleNames().key("entityName"));
  entityItem->setData(QString::number(_entity),
      this->roleNames().key("entity"));
  entityItem->setData(_type, this->roleNames().key("type"));

  parentItem->appendRow(entityItem);

  this->entityItems[_entity] = entityItem;

  // Check if there are pending children
  auto sep = std::partition(this->pendingEntities.begin(),
      this->pendingEntities.end(), [&_entity](const EntityInfo &_entityInfo)
      {
        return _entityInfo.parentEntity != _entity;
      });

  for (auto it = sep; it != this->pendingEntities.end(); ++it)
  {
    this->AddEntity(it->entity, it->name, it->parentEntity, it->type);
  }
  this->pendingEntities.erase(sep, this->pendingEntities.end());
}

/////////////////////////////////////////////////
void TreeModel::RemoveEntity(Entity _entity)
{
  GZ_PROFILE("TreeModel::RemoveEntity");
  QStandardItem *item{nullptr};
  auto itemIt = this->entityItems.find(_entity);
  if (itemIt != this->entityItems.end())
  {
    item = itemIt->second;
  }

  if (nullptr == item)
  {
    // See if it's pending
    auto toRemove = std::remove_if(this->pendingEntities.begin(),
        this->pendingEntities.end(), [&_entity](const EntityInfo &_entityInfo)
        {
          return _entityInfo.entity == _entity;
        });
    this->pendingEntities.erase(toRemove, this->pendingEntities.end());

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
Entity TreeModel::EntityId(const QModelIndex &_index) const
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
          std::pair(102, "type")};
}

/////////////////////////////////////////////////
EntityTree::EntityTree()
  : GuiSystem(), dataPtr(std::make_unique<EntityTreePrivate>())
{
  // Connect model
  gz::gui::App()->Engine()->rootContext()->setContextProperty(
     "EntityTreeModel", &this->dataPtr->treeModel);
}

/////////////////////////////////////////////////
EntityTree::~EntityTree() = default;

/////////////////////////////////////////////////
void EntityTree::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Entity tree";

  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void EntityTree::Update(const UpdateInfo &, EntityComponentManager &_ecm)
{
  GZ_PROFILE("EntityTree::Update");
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
          Q_ARG(Entity, _entity),
          Q_ARG(QString, QString::fromStdString(_name->Data())),
          Q_ARG(Entity, parentEntity),
          Q_ARG(QString, entityType(_entity, _ecm)));
      return true;
    });

    if (this->dataPtr->worldEntity != kNullEntity)
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
          Q_ARG(Entity, _entity),
          Q_ARG(QString, QString::fromStdString(_name->Data())),
          Q_ARG(Entity, parentEntity),
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
        Q_ARG(Entity, _entity));
    return true;
  });

  {
    // update the entity tree with new/removed entities from gui events
    std::lock_guard<std::mutex> lock(this->dataPtr->newRemovedEntityMutex);

    for (auto entity : this->dataPtr->newEntities)
    {
      // make sure the entity to be added has a name and parent
      auto nameComp = _ecm.Component<components::Name>(entity);
      if (!nameComp)
      {
        gzerr << "Could not add entity [" << entity << "] to the entity tree "
               << "because it does not have a name component.\n";
        continue;
      }
      auto parentComp = _ecm.Component<components::ParentEntity>(entity);
      if (!parentComp)
      {
        gzerr << "Could not add entity [" << entity << "] to the entity tree "
               << "because it does not have a parent entity component.\n";
        continue;
      }

      // World children are top-level
      auto parentEntity = parentComp->Data();
      if (this->dataPtr->worldEntity != kNullEntity &&
          parentEntity == this->dataPtr->worldEntity)
      {
        parentEntity = kNullEntity;
      }

      QMetaObject::invokeMethod(&this->dataPtr->treeModel, "AddEntity",
          Qt::QueuedConnection,
          Q_ARG(Entity, entity),
          Q_ARG(QString, QString::fromStdString(nameComp->Data())),
          Q_ARG(Entity, parentEntity),
          Q_ARG(QString, entityType(entity, _ecm)));
    }

    for (auto entity : this->dataPtr->removedEntities)
    {
      QMetaObject::invokeMethod(&this->dataPtr->treeModel, "RemoveEntity",
          Qt::QueuedConnection,
          Q_ARG(Entity, entity));
    }

    this->dataPtr->newEntities.clear();
    this->dataPtr->removedEntities.clear();
  }
}

/////////////////////////////////////////////////
void EntityTree::OnEntitySelectedFromQml(Entity _entity)
{
  std::vector<Entity> entitySet {_entity};
  gui::events::EntitiesSelected event(entitySet, true);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &event);
}

/////////////////////////////////////////////////
void EntityTree::DeselectAllEntities()
{
  gui::events::DeselectAllEntities event(true);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &event);
}

/////////////////////////////////////////////////
void EntityTree::OnInsertEntity(const QString &_type)
{
  std::string modelSdfString = getPrimitive(_type.toStdString());
  gz::gui::events::SpawnFromDescription event(modelSdfString);
  gz::gui::App()->sendEvent(
      gz::gui::App()->findChild<gz::gui::MainWindow *>(),
      &event);
}

/////////////////////////////////////////////////
void EntityTree::OnLoadMesh(const QString &_mesh)
{
  std::string meshStr = _mesh.toStdString();
  if (QUrl(_mesh).isLocalFile())
  {
    // mesh to sdf model
    common::rtrim(meshStr);

    if (!common::MeshManager::Instance()->IsValidFilename(meshStr))
    {
      QString errTxt = QString::fromStdString("Invalid URI: " + meshStr +
        "\nOnly mesh file types DAE, OBJ, and STL are supported.");
      return;
    }

    std::string filename = common::basename(meshStr);
    std::vector<std::string> splitName = common::split(filename, ".");

    std::string sdf = "<?xml version='1.0'?>"
      "<sdf version='" + std::string(SDF_PROTOCOL_VERSION) + "'>"
        "<model name='" + splitName[0] + "'>"
          "<link name='link'>"
            "<visual name='visual'>"
              "<geometry>"
                "<mesh>"
                  "<uri>" + meshStr + "</uri>"
                "</mesh>"
              "</geometry>"
            "</visual>"
            "<collision name='collision'>"
              "<geometry>"
                "<mesh>"
                  "<uri>" + meshStr + "</uri>"
                "</mesh>"
              "</geometry>"
            "</collision>"
          "</link>"
        "</model>"
      "</sdf>";

    gz::gui::events::SpawnFromDescription event(sdf);
    gz::gui::App()->sendEvent(
        gz::gui::App()->findChild<gz::gui::MainWindow *>(),
        &event);

  }
}

/////////////////////////////////////////////////
bool EntityTree::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == gui::events::EntitiesSelected::kType)
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
            QVariant(static_cast<qulonglong>(entity))));
      }
    }
  }
  else if (_event->type() ==
           gui::events::DeselectAllEntities::kType)
  {
    auto deselectAllEvent =
        reinterpret_cast<gui::events::DeselectAllEntities *>(_event);
    if (deselectAllEvent)
    {
      QMetaObject::invokeMethod(this->PluginItem(), "deselectAllEntities",
          Qt::QueuedConnection);
    }
  }
  else if (_event->type() ==
           gz::sim::gui::events::GuiNewRemovedEntities::kType)
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->newRemovedEntityMutex);
    auto addedRemovedEvent =
        reinterpret_cast<gui::events::GuiNewRemovedEntities *>(_event);
    if (addedRemovedEvent)
    {
      // TODO(chapulina) Make these entities visually different from entities
      // created from the server.
      for (auto entity : addedRemovedEvent->NewEntities())
        this->dataPtr->newEntities.insert(entity);

      for (auto entity : addedRemovedEvent->RemovedEntities())
        this->dataPtr->removedEntities.insert(entity);
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

// Register this plugin
GZ_ADD_PLUGIN(EntityTree,
              gz::gui::Plugin)
