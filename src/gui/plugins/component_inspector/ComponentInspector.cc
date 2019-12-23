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
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

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
  item->setData(_typeName, this->roleNames().key("typeName"));
  item->setData(QString::number(_typeId),
      this->roleNames().key("typeId"));

  this->invisibleRootItem()->appendRow(item);
  this->items[_typeName] = item;
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
          std::pair(107, "yaw")};
}

/////////////////////////////////////////////////
ComponentInspector::ComponentInspector()
  : GuiSystem(), dataPtr(std::make_unique<ComponentInspectorPrivate>())
{
  // Connect model
  gui::App()->Engine()->rootContext()->setContextProperty(
      "ComponentInspectorModel", &this->dataPtr->treeModel);
}

/////////////////////////////////////////////////
ComponentInspector::~ComponentInspector() = default;

/////////////////////////////////////////////////
void ComponentInspector::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Component inspector";

  // Get entity from XML
  this->dataPtr->entity = 12;
}

//////////////////////////////////////////////////
void ComponentInspector::Update(const UpdateInfo &, EntityComponentManager &_ecm)
{
  IGN_PROFILE("ComponentInspector::Update");

  auto components = _ecm.ComponentTypes(this->dataPtr->entity);

  // List all components
  for (const auto &typeId : components)
  {
    auto name = components::Factory::Instance()->Name(typeId);

    QMetaObject::invokeMethod(&this->dataPtr->treeModel, "AddComponent",
        Qt::QueuedConnection,
        Q_ARG(long, typeId),
        Q_ARG(QString, QString::fromStdString(name)));
  }

  // Handle known components
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

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ComponentInspector,
                    ignition::gui::Plugin)
