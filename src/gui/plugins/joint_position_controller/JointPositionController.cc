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

#include <algorithm>
#include <iostream>
#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/math/Helpers.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <ignition/transport/TopicUtils.hh>

#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/JointAxis.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointType.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "JointPositionController.hh"

namespace ignition::gazebo::gui
{
  class JointPositionControllerPrivate
  {
    /// \brief Model holding all the joints.
    public: JointsModel jointsModel;

    /// \brief Model entity being controller.
    public: Entity modelEntity{kNullEntity};

    /// \brief Name of the model
    public: QString modelName;

    /// \brief Whether currently locked on a given entity
    public: bool locked{false};

    /// \brief Transport node for making command requests
    public: transport::Node node;

    /// \brief Whether the initial model set from XML has been setup.
    public: bool sdfModelInitialized{false};
  };
}

using namespace ignition;
using namespace ignition::gazebo;
using namespace ignition::gazebo::gui;

/////////////////////////////////////////////////
JointsModel::JointsModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
QStandardItem *JointsModel::AddJoint(Entity _entity)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("JointsModel::AddJoint");

  auto itemIt = this->items.find(_entity);

  // Existing item
  if (itemIt != this->items.end())
  {
    return itemIt->second;
  }

  // New joint item
  auto item = new QStandardItem(QString::number(_entity));

  this->invisibleRootItem()->appendRow(item);
  this->items[_entity] = item;
  return item;
}

/////////////////////////////////////////////////
void JointsModel::RemoveJoint(Entity _entity)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("JointsModel::RemoveJoint");

  auto itemIt = this->items.find(_entity);

  // Existing item
  if (itemIt != this->items.end())
  {
    this->invisibleRootItem()->removeRow(itemIt->second->row());
    this->items.erase(_entity);
  }
}

/////////////////////////////////////////////////
void JointsModel::Clear()
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("JointsModel::Clear");

  this->invisibleRootItem()->removeRows(0, this->rowCount());
  this->items.clear();
}

/////////////////////////////////////////////////
QHash<int, QByteArray> JointsModel::roleNames() const
{
  return JointsModel::RoleNames();
}

/////////////////////////////////////////////////
QHash<int, QByteArray> JointsModel::RoleNames()
{
  return {std::pair(100, "entity"),
          std::pair(101, "name"),
          std::pair(102, "min"),
          std::pair(103, "max"),
          std::pair(104, "value")};
}

/////////////////////////////////////////////////
JointPositionController::JointPositionController()
  : GuiSystem(), dataPtr(std::make_unique<JointPositionControllerPrivate>())
{
  qRegisterMetaType<Entity>("Entity");
}

/////////////////////////////////////////////////
JointPositionController::~JointPositionController() = default;

/////////////////////////////////////////////////
void JointPositionController::LoadConfig(
    const tinyxml2::XMLElement *_pluginElem)
{
  if (this->title.empty())
    this->title = "Joint position controller";

  if (_pluginElem)
  {
    if (auto elem = _pluginElem->FirstChildElement("model_name"))
    {
      this->dataPtr->modelName = QString::fromStdString(elem->GetText());
    }
  }

  // If model name isn't set, initialization is complete.
  this->dataPtr->sdfModelInitialized = this->dataPtr->modelName.isEmpty();

  ignition::gui::App()->findChild<
      ignition::gui::MainWindow *>()->installEventFilter(this);

  // Connect model
  this->Context()->setContextProperty(
      "JointsModel", &this->dataPtr->jointsModel);
  this->dataPtr->jointsModel.setParent(this);
}

//////////////////////////////////////////////////
void JointPositionController::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("JointPositionController::Update");

  if (!this->dataPtr->sdfModelInitialized)
  {
    this->dataPtr->modelEntity = _ecm.EntityByComponents(
        components::Name(this->dataPtr->modelName.toStdString()));
    this->dataPtr->sdfModelInitialized;
    this->SetLocked(true);
  }

  if (this->dataPtr->modelEntity == kNullEntity ||
      nullptr == _ecm.Component<components::Model>(
      this->dataPtr->modelEntity))
  {
    QMetaObject::invokeMethod(&this->dataPtr->jointsModel,
        "Clear",
        Qt::QueuedConnection);
    this->SetModelName("No model selected");
    this->SetLocked(false);
    return;
  }

  this->SetModelName(QString::fromStdString(
      _ecm.ComponentData<components::Name>(
      this->dataPtr->modelEntity).value()));

  auto jointEntities = _ecm.EntitiesByComponents(components::Joint(),
      components::ParentEntity(this->dataPtr->modelEntity));

  // List all joints
  for (const auto &jointEntity : jointEntities)
  {
    auto typeComp = _ecm.Component<components::JointType>(jointEntity);
    if (nullptr == typeComp)
    {
      ignerr << "Joint [" << jointEntity << "] missing type" << std::endl;
      continue;
    }

    if (typeComp->Data() == sdf::JointType::INVALID ||
        typeComp->Data() == sdf::JointType::BALL ||
        typeComp->Data() == sdf::JointType::FIXED)
    {
      continue;
    }

    // Get joint item
    QStandardItem *item;
    auto itemIt = this->dataPtr->jointsModel.items.find(jointEntity);
    if (itemIt != this->dataPtr->jointsModel.items.end())
    {
      item = itemIt->second;
    }
    // Add joint to list
    else
    {
      // TODO(louise) Blocking here is not the best idea
      QMetaObject::invokeMethod(&this->dataPtr->jointsModel,
          "AddJoint",
          Qt::BlockingQueuedConnection,
          Q_RETURN_ARG(QStandardItem *, item),
          Q_ARG(Entity, jointEntity));
    }

    if (nullptr == item)
    {
      ignerr << "Failed to get item for joint [" << jointEntity << "]"
             << std::endl;
      continue;
    }

    // Name
    auto name = _ecm.ComponentData<components::Name>(jointEntity).value();
    item->setData(QString::fromStdString(name),
        JointsModel::RoleNames().key("name"));

    // Limits
    double min = -IGN_PI;
    double max = IGN_PI;
    auto axisComp = _ecm.Component<components::JointAxis>(jointEntity);
    if (axisComp)
    {
      min = axisComp->Data().Lower();
      max = axisComp->Data().Upper();
    }
    item->setData(min, JointsModel::RoleNames().key("min"));
    item->setData(max, JointsModel::RoleNames().key("max"));

    // Value
    double value = 0.0;
    auto posComp = _ecm.Component<components::JointPosition>(jointEntity);
    if (posComp)
    {
      value = posComp->Data()[0];
    }
    item->setData(value, JointsModel::RoleNames().key("value"));
  }

  // Remove joints no longer present
  for (auto itemIt : this->dataPtr->jointsModel.items)
  {
    auto jointEntity = itemIt.first;
    if (std::find(jointEntities.begin(), jointEntities.end(), jointEntity) ==
        jointEntities.end())
    {
      QMetaObject::invokeMethod(&this->dataPtr->jointsModel,
          "RemoveJoint",
          Qt::QueuedConnection,
          Q_ARG(Entity, jointEntity));
    }
  }
}

/////////////////////////////////////////////////
bool JointPositionController::eventFilter(QObject *_obj, QEvent *_event)
{
  if (!this->dataPtr->locked)
  {
    if (_event->type() == gazebo::gui::events::EntitiesSelected::kType)
    {
      auto event = reinterpret_cast<gui::events::EntitiesSelected *>(_event);
      if (event && !event->Data().empty())
      {
        this->SetModelEntity(*event->Data().begin());
      }
    }

    if (_event->type() == gazebo::gui::events::DeselectAllEntities::kType)
    {
      auto event = reinterpret_cast<gui::events::DeselectAllEntities *>(
          _event);
      if (event)
      {
        this->SetModelEntity(kNullEntity);
      }
    }
  }

  // Standard event processing
  return QObject::eventFilter(_obj, _event);
}

/////////////////////////////////////////////////
Entity JointPositionController::ModelEntity() const
{
  return this->dataPtr->modelEntity;
}

/////////////////////////////////////////////////
void JointPositionController::SetModelEntity(Entity _entity)
{
  this->dataPtr->modelEntity = _entity;
  this->ModelEntityChanged();

  if (this->dataPtr->modelEntity == kNullEntity)
  {
    this->dataPtr->modelName.clear();
  }
}

/////////////////////////////////////////////////
QString JointPositionController::ModelName() const
{
  return this->dataPtr->modelName;
}

/////////////////////////////////////////////////
void JointPositionController::SetModelName(const QString &_modelName)
{
  this->dataPtr->modelName = _modelName;
  this->ModelNameChanged();
}

/////////////////////////////////////////////////
bool JointPositionController::Locked() const
{
  return this->dataPtr->locked;
}

/////////////////////////////////////////////////
void JointPositionController::SetLocked(bool _locked)
{
  this->dataPtr->locked = _locked;
  this->LockedChanged();
}

/////////////////////////////////////////////////
void JointPositionController::OnCommand(const QString &_jointName, double _pos)
{
  std::string jointName = _jointName.toStdString();

  ignition::msgs::Double msg;
  msg.set_data(_pos);
  auto topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->modelName.toStdString() + "/joint/" + jointName +
      "/0/cmd_pos");

  if (topic.empty())
  {
    ignerr << "Failed to create valid topic for joint [" << jointName << "]"
           << std::endl;
    return;
  }

  auto pub = this->dataPtr->node.Advertise<ignition::msgs::Double>(topic);
  pub.Publish(msg);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::gui::JointPositionController,
                    ignition::gui::Plugin)
