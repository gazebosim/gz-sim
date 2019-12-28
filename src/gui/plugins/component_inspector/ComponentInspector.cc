/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <regex>
#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "ComponentInspector.hh"

namespace ignition::gazebo
{
  class ComponentInspectorPrivate
  {
    /// \brief Model holding all the current components
    public: TreeModel treeModel;

    public: Entity entity;
  };
}

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
std::string shortName(const std::string &_typeName)
{
  // Remove namespaces
  auto name = _typeName.substr(_typeName.rfind('.')+1);

  // Split CamelCase
  std::regex reg("(\\B[A-Z])");
  name = std::regex_replace(name, reg, " $1");

  return name;
}

/////////////////////////////////////////////////
TreeModel::TreeModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
void TreeModel::AddComponent(long _typeId, const QString &_typeName)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("TreeModel::AddComponent");

  if (this->items.find(_typeName) != this->items.end())
    return;

  // New component item
  auto item = new QStandardItem(_typeName);
  item->setData(QString::fromStdString(shortName(
      _typeName.toStdString())), this->roleNames().key("shortName"));
  item->setData(_typeName, this->roleNames().key("typeName"));
  item->setData(QString::number(_typeId),
      this->roleNames().key("typeId"));
  item->setData("true", this->roleNames().key("isType"));

  this->invisibleRootItem()->appendRow(item);
  this->items[_typeName] = item;
}

/////////////////////////////////////////////////
void TreeModel::AddName(const QString &_name)
{
  IGN_PROFILE("TreeModel::AddName");

  auto typeName =
      components::Factory::Instance()->Name(components::Name::typeId);

  auto nameItem = this->items.find(QString::fromStdString(typeName));
  if (nameItem == this->items.end())
  {
    ignerr << "Internal error, missing component ["
           << typeName << "]" << std::endl;
    return;
  }

  auto itemName = QString::fromStdString(typeName + "_component");

  auto itemIt = this->items.find(itemName);

  // New component item
  QStandardItem *item{nullptr};
  if (itemIt != this->items.end())
  {
    item = itemIt->second;
  }
  else
  {
    item = new QStandardItem(itemName);
    nameItem->second->appendRow(item);
  }

  item->setData(QString::fromStdString(typeName),
      this->roleNames().key("typeName"));
  item->setData(_name, this->roleNames().key("name"));

  this->items[itemName] = item;
}

/////////////////////////////////////////////////
void TreeModel::AddPose(
    double _x,
    double _y,
    double _z,
    double _roll,
    double _pitch,
    double _yaw)
{
  IGN_PROFILE("TreeModel::AddPose");

  auto typeName =
      components::Factory::Instance()->Name(components::Pose::typeId);

  auto poseItem =
      this->items.find(QString::fromStdString(typeName));
  if (poseItem == this->items.end())
  {
    ignerr << "Internal error, missing component ["
           << typeName << "]" << std::endl;
    return;
  }

  auto itemName = QString::fromStdString(typeName + "_component");

  auto itemIt = this->items.find(itemName);

  // New component item
  QStandardItem *item{nullptr};
  if (itemIt != this->items.end())
  {
    item = itemIt->second;
  }
  else
  {
    item = new QStandardItem(itemName);
    poseItem->second->appendRow(item);
  }

  item->setData(QString::fromStdString(typeName),
      this->roleNames().key("typeName"));
  item->setData(QString::number(_x), this->roleNames().key("x"));
  item->setData(QString::number(_y), this->roleNames().key("y"));
  item->setData(QString::number(_z), this->roleNames().key("z"));
  item->setData(QString::number(_roll), this->roleNames().key("roll"));
  item->setData(QString::number(_pitch), this->roleNames().key("pitch"));
  item->setData(QString::number(_yaw), this->roleNames().key("yaw"));

  this->items[itemName] = item;
}

/////////////////////////////////////////////////
QHash<int, QByteArray> TreeModel::roleNames() const
{
  return {std::pair(100, "typeName"),
          std::pair(101, "typeId"),
          std::pair(102, "x"),
          std::pair(103, "y"),
          std::pair(104, "z"),
          std::pair(105, "roll"),
          std::pair(106, "pitch"),
          std::pair(107, "yaw"),
          std::pair(108, "name"),
          std::pair(109, "isType"),
          std::pair(110, "shortName")};
}

/////////////////////////////////////////////////
ComponentInspector::ComponentInspector()
  : GuiSystem(), dataPtr(std::make_unique<ComponentInspectorPrivate>())
{
  // Connect model
  ignition::gui::App()->Engine()->rootContext()->setContextProperty(
      "ComponentInspectorModel", &this->dataPtr->treeModel);
}

/////////////////////////////////////////////////
ComponentInspector::~ComponentInspector() = default;

/////////////////////////////////////////////////
void ComponentInspector::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Component inspector";

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);
}

//////////////////////////////////////////////////
void ComponentInspector::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("ComponentInspector::Update");

  auto componentTypes = _ecm.ComponentTypes(this->dataPtr->entity);

  // List all components
  // TODO(louise) Remove components that are no longer present
  for (const auto &typeId : componentTypes)
  {
    auto name = components::Factory::Instance()->Name(typeId);

    QMetaObject::invokeMethod(&this->dataPtr->treeModel, "AddComponent",
        Qt::QueuedConnection,
        Q_ARG(long, typeId),
        Q_ARG(QString, QString::fromStdString(name)));
  }

  // Handle known components
  auto nameComp = _ecm.Component<components::Name>(this->dataPtr->entity);
  if (nameComp)
  {
    QMetaObject::invokeMethod(&this->dataPtr->treeModel, "AddName",
        Qt::QueuedConnection,
        Q_ARG(QString, QString::fromStdString(nameComp->Data())));
  }

  auto poseComp = _ecm.Component<components::Pose>(this->dataPtr->entity);
  if (poseComp)
  {
    QMetaObject::invokeMethod(&this->dataPtr->treeModel, "AddPose",
        Qt::QueuedConnection,
        Q_ARG(double, poseComp->Data().Pos().X()),
        Q_ARG(double, poseComp->Data().Pos().Y()),
        Q_ARG(double, poseComp->Data().Pos().Z()),
        Q_ARG(double, poseComp->Data().Rot().Roll()),
        Q_ARG(double, poseComp->Data().Rot().Pitch()),
        Q_ARG(double, poseComp->Data().Rot().Yaw()));
  }
}

/////////////////////////////////////////////////
bool ComponentInspector::eventFilter(QObject *_obj, QEvent *_event)
{
  if (_event->type() == ignition::gazebo::gui::events::EntitiesSelected::Type)
  {
    auto selectedEvent =
        reinterpret_cast<gui::events::EntitiesSelected *>(_event);
    if (selectedEvent && !selectedEvent->Data().empty())
    {
      this->SetEntity(*selectedEvent->Data().begin());
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
int ComponentInspector::Entity() const
{
  return this->dataPtr->entity;
}

/////////////////////////////////////////////////
void ComponentInspector::SetEntity(const int &_entity)
{
  this->dataPtr->entity = _entity;
  this->EntityChanged();
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ComponentInspector,
                    ignition::gui::Plugin)
