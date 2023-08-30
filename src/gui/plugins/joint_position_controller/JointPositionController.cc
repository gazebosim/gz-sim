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
#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/math/Helpers.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/gui/GuiEvents.hh"

#include "JointPositionController.hh"

namespace gz::sim::gui
{
  class JointPositionControllerPrivate
  {
    /// \brief Model holding all the joints.
    public: JointsModel jointsModel;

    /// \brief Model entity being controlled.
    public: Entity modelEntity{kNullEntity};

    /// \brief Previous model entity being controlled.
    public: Entity prevModelEntity{kNullEntity};

    /// \brief Name of the model
    public: QString modelName{"No model selected"};

    /// \brief Whether currently locked on a given entity
    public: bool locked{false};

    /// \brief Transport node for making command requests
    public: transport::Node node;

    /// \brief Whether the initial model set from XML has been setup.
    public: bool xmlModelInitialized{true};
  };
}

using namespace gz;
using namespace gz::sim;
using namespace gz::sim::gui;

/////////////////////////////////////////////////
JointsModel::JointsModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
QStandardItem *JointsModel::AddJoint(Entity _entity)
{
  GZ_PROFILE_THREAD_NAME("Qt thread");
  GZ_PROFILE("JointsModel::AddJoint");

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
  GZ_PROFILE_THREAD_NAME("Qt thread");
  GZ_PROFILE("JointsModel::RemoveJoint");

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
  GZ_PROFILE_THREAD_NAME("Qt thread");
  GZ_PROFILE("JointsModel::Clear");

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
      // If model name isn't set, initialization is not complete yet.
      this->dataPtr->xmlModelInitialized = false;
    }
  }

  gz::gui::App()->findChild<
      gz::gui::MainWindow *>()->installEventFilter(this);

  // Connect model
  this->Context()->setContextProperty(
      "JointsModel", &this->dataPtr->jointsModel);
  this->dataPtr->jointsModel.setParent(this);
}

//////////////////////////////////////////////////
void JointPositionController::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("JointPositionController::Update");

  if (!this->dataPtr->xmlModelInitialized)
  {
    auto entity = _ecm.EntityByComponents(
        components::Name(this->dataPtr->modelName.toStdString()));

    // Don't initialize until we get the entity
    if (entity == kNullEntity)
      return;

    this->SetModelEntity(entity);
    this->SetLocked(true);
    this->dataPtr->xmlModelInitialized = true;
    gzmsg << "Controller locked on [" << this->dataPtr->modelName.toStdString()
           << "]" << std::endl;
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

  if (this->dataPtr->prevModelEntity != this->dataPtr->modelEntity)
  {
    this->dataPtr->prevModelEntity = this->dataPtr->modelEntity;
    this->dataPtr->jointsModel.Clear();
  }

  // List all joints
  for (const auto &jointEntity : jointEntities)
  {
    auto typeComp = _ecm.Component<components::JointType>(jointEntity);
    if (nullptr == typeComp)
    {
      gzerr << "Joint [" << jointEntity << "] missing type" << std::endl;
      continue;
    }

    if (typeComp->Data() == sdf::JointType::INVALID ||
        typeComp->Data() == sdf::JointType::BALL ||
        typeComp->Data() == sdf::JointType::FIXED)
    {
      continue;
    }

    // Get joint item
    bool newItem{false};
    QStandardItem *item;
    auto itemIt = this->dataPtr->jointsModel.items.find(jointEntity);
    if (itemIt != this->dataPtr->jointsModel.items.end())
    {
      item = itemIt->second;
    }
    // Add joint to list
    else
    {
      item = this->dataPtr->jointsModel.AddJoint(jointEntity);
      newItem = true;
    }

    if (nullptr == item)
    {
      gzerr << "Failed to get item for joint [" << jointEntity << "]"
             << std::endl;
      continue;
    }

    if (newItem)
    {
      // Name
      auto name = _ecm.ComponentData<components::Name>(jointEntity).value();
      item->setData(QString::fromStdString(name),
          JointsModel::RoleNames().key("name"));

      // Limits
      double min = -GZ_PI;
      double max = GZ_PI;
      auto axisComp = _ecm.Component<components::JointAxis>(jointEntity);
      if (axisComp)
      {
        min = axisComp->Data().Lower();
        max = axisComp->Data().Upper();
      }
      item->setData(min, JointsModel::RoleNames().key("min"));
      item->setData(max, JointsModel::RoleNames().key("max"));
    }

    // Value
    double value = 0.0;
    auto posComp = _ecm.Component<components::JointPosition>(jointEntity);
    if (posComp && !posComp->Data().empty())
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
          Q_ARG(sim::Entity, jointEntity));
    }
  }
}

/////////////////////////////////////////////////
bool JointPositionController::eventFilter(QObject *_obj, QEvent *_event)
{
  if (!this->dataPtr->locked)
  {
    if (_event->type() == sim::gui::events::EntitiesSelected::kType)
    {
      auto event = reinterpret_cast<gui::events::EntitiesSelected *>(_event);
      if (event && !event->Data().empty())
      {
        this->SetModelEntity(*event->Data().begin());
      }
    }

    if (_event->type() == sim::gui::events::DeselectAllEntities::kType)
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

  msgs::Double msg;
  msg.set_data(_pos);
  auto topic = transport::TopicUtils::AsValidTopic("/model/" +
      this->dataPtr->modelName.toStdString() + "/joint/" + jointName +
      "/0/cmd_pos");

  if (topic.empty())
  {
    gzerr << "Failed to create valid topic for joint [" << jointName << "]"
           << std::endl;
    return;
  }

  auto pub = this->dataPtr->node.Advertise<msgs::Double>(topic);
  pub.Publish(msg);
}

/////////////////////////////////////////////////
void JointPositionController::OnReset()
{
  for (auto itemIt : this->dataPtr->jointsModel.items)
  {
    auto jointName = itemIt.second->data(JointsModel::RoleNames().key("name"))
        .toString().toStdString();
    if (jointName.empty())
    {
      gzerr << "Internal error: failed to get joint name." << std::endl;
      continue;
    }

    msgs::Double msg;
    msg.set_data(0);
    auto topic = transport::TopicUtils::AsValidTopic("/model/" +
        this->dataPtr->modelName.toStdString() + "/joint/" + jointName +
        "/0/cmd_pos");

    if (topic.empty())
    {
      gzerr << "Failed to create valid topic for joint [" << jointName << "]"
             << std::endl;
      return;
    }

    auto pub = this->dataPtr->node.Advertise<msgs::Double>(topic);
    pub.Publish(msg);
  }
}

// Register this plugin
GZ_ADD_PLUGIN(JointPositionController,
                    gz::gui::Plugin)
