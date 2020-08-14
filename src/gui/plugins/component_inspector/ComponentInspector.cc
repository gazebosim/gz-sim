/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Actor.hh"
#include "ignition/gazebo/components/AngularAcceleration.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/LinearVelocitySeed.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/PoseCmd.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/WindMode.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "ComponentInspector.hh"

namespace ignition::gazebo
{
  class ComponentInspectorPrivate
  {
    /// \brief Model holding all the current components.
    public: ComponentsModel componentsModel;

    /// \brief Entity being inspected. Default to world.
    public: Entity entity{1};

    /// \brief World entity
    public: Entity worldEntity{kNullEntity};

    /// \brief Name of the world
    public: std::string worldName;

    /// \brief Entity type, such as 'world' or 'model'.
    public: QString type;

    /// \brief Whether currently locked on a given entity
    public: bool locked{false};

    /// \brief Whether updates are currently paused.
    public: bool paused{false};

    /// \brief Transport node for making command requests
    public: transport::Node node;
  };
}

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
template<>
void ignition::gazebo::setData(QStandardItem *_item, const math::Pose3d &_data)
{
  _item->setData(QString("Pose3d"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QList({
    QVariant(_data.Pos().X()),
    QVariant(_data.Pos().Y()),
    QVariant(_data.Pos().Z()),
    QVariant(_data.Rot().Roll()),
    QVariant(_data.Rot().Pitch()),
    QVariant(_data.Rot().Yaw())
  }), ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void ignition::gazebo::setData(QStandardItem *_item,
    const math::Vector3d &_data)
{
  _item->setData(QString("Vector3d"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QList({
    QVariant(_data.X()),
    QVariant(_data.Y()),
    QVariant(_data.Z())
  }), ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void ignition::gazebo::setData(QStandardItem *_item, const std::string &_data)
{
  _item->setData(QString("String"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QString::fromStdString(_data),
      ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void ignition::gazebo::setData(QStandardItem *_item,
    const std::ostringstream &_data)
{
  _item->setData(QString("Raw"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QString::fromStdString(_data.str()),
      ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void ignition::gazebo::setData(QStandardItem *_item, const bool &_data)
{
  _item->setData(QString("Boolean"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(_data, ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void ignition::gazebo::setData(QStandardItem *_item, const int &_data)
{
  _item->setData(QString("Integer"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(_data, ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void ignition::gazebo::setData(QStandardItem *_item, const double &_data)
{
  _item->setData(QString("Float"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(_data, ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
void ignition::gazebo::setUnit(QStandardItem *_item, const std::string &_unit)
{
  _item->setData(QString::fromStdString(_unit),
      ComponentsModel::RoleNames().key("unit"));
}

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
ComponentsModel::ComponentsModel() : QStandardItemModel()
{
}

/////////////////////////////////////////////////
QStandardItem *ComponentsModel::AddComponentType(
    ignition::gazebo::ComponentTypeId _typeId)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("ComponentsModel::AddComponentType");

  auto typeName = QString::fromStdString(
      components::Factory::Instance()->Name(_typeId));

  auto itemIt = this->items.find(_typeId);

  // Existing component item
  if (itemIt != this->items.end())
  {
    return itemIt->second;
  }

  // New component item
  auto item = new QStandardItem(typeName);
  item->setData(QString::fromStdString(shortName(
      typeName.toStdString())), this->roleNames().key("shortName"));
  item->setData(typeName, this->roleNames().key("typeName"));
  item->setData(QString::number(_typeId),
      this->roleNames().key("typeId"));

  this->invisibleRootItem()->appendRow(item);
  this->items[_typeId] = item;
  return item;
}

/////////////////////////////////////////////////
void ComponentsModel::RemoveComponentType(
      ignition::gazebo::ComponentTypeId _typeId)
{
  IGN_PROFILE_THREAD_NAME("Qt thread");
  IGN_PROFILE("ComponentsModel::RemoveComponentType");

  auto itemIt = this->items.find(_typeId);

  // Existing component item
  if (itemIt != this->items.end())
  {
    this->invisibleRootItem()->removeRow(itemIt->second->row());
    this->items.erase(_typeId);
  }
}

/////////////////////////////////////////////////
QHash<int, QByteArray> ComponentsModel::roleNames() const
{
  return ComponentsModel::RoleNames();
}

/////////////////////////////////////////////////
QHash<int, QByteArray> ComponentsModel::RoleNames()
{
  return {std::pair(100, "typeName"),
          std::pair(101, "typeId"),
          std::pair(102, "shortName"),
          std::pair(103, "dataType"),
          std::pair(104, "unit"),
          std::pair(105, "data")};
}

/////////////////////////////////////////////////
ComponentInspector::ComponentInspector()
  : GuiSystem(), dataPtr(std::make_unique<ComponentInspectorPrivate>())
{
  qRegisterMetaType<ignition::gazebo::ComponentTypeId>();
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

  // Connect model
  this->Context()->setContextProperty(
      "ComponentsModel", &this->dataPtr->componentsModel);
}

//////////////////////////////////////////////////
void ComponentInspector::Update(const UpdateInfo &,
    EntityComponentManager &_ecm)
{
  IGN_PROFILE("ComponentInspector::Update");

  if (this->dataPtr->paused)
    return;

  auto componentTypes = _ecm.ComponentTypes(this->dataPtr->entity);

  // List all components
  for (const auto &typeId : componentTypes)
  {
    // Type components
    if (typeId == components::World::typeId)
    {
      this->dataPtr->worldEntity = this->dataPtr->entity;
      this->SetType("world");
      continue;
    }

    if (typeId == components::Model::typeId)
    {
      this->SetType("model");
      continue;
    }

    if (typeId == components::Link::typeId)
    {
      this->SetType("link");
      continue;
    }

    if (typeId == components::Collision::typeId)
    {
      this->SetType("collision");
      continue;
    }

    if (typeId == components::Visual::typeId)
    {
      this->SetType("visual");
      continue;
    }

    if (typeId == components::Sensor::typeId)
    {
      this->SetType("sensor");
      continue;
    }

    if (typeId == components::Joint::typeId)
    {
      this->SetType("joint");
      continue;
    }

    if (typeId == components::Performer::typeId)
    {
      this->SetType("performer");
      continue;
    }

    if (typeId == components::Level::typeId)
    {
      this->SetType("level");
      continue;
    }

    if (typeId == components::Light::typeId)
    {
      this->SetType("light");
      continue;
    }

    if (typeId == components::Actor::typeId)
    {
      this->SetType("actor");
      continue;
    }

    // Get component item
    QStandardItem *item;
    auto itemIt = this->dataPtr->componentsModel.items.find(typeId);
    if (itemIt != this->dataPtr->componentsModel.items.end())
    {
      item = itemIt->second;
    }
    // Add component to list
    else
    {
      // TODO(louise) Blocking here is not the best idea
      QMetaObject::invokeMethod(&this->dataPtr->componentsModel,
          "AddComponentType",
          Qt::BlockingQueuedConnection,
          Q_RETURN_ARG(QStandardItem *, item),
          Q_ARG(ignition::gazebo::ComponentTypeId, typeId));
    }

    if (nullptr == item)
    {
      ignerr << "Failed to get item for component type [" << typeId << "]"
             << std::endl;
      continue;
    }

    // Populate component-specific data
    if (typeId == components::AngularAcceleration::typeId)
    {
      auto comp = _ecm.Component<components::AngularAcceleration>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "rad/s\u00B2");
      }
    }
    else if (typeId == components::AngularVelocity::typeId)
    {
      auto comp = _ecm.Component<components::AngularVelocity>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "rad/s");
      }
    }
    else if (typeId == components::CastShadows::typeId)
    {
      auto comp = _ecm.Component<components::CastShadows>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::ChildLinkName::typeId)
    {
      auto comp = _ecm.Component<components::ChildLinkName>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::Gravity::typeId)
    {
      auto comp = _ecm.Component<components::Gravity>(this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "m/s\u00B2");
      }
    }
    else if (typeId == components::LinearAcceleration::typeId)
    {
      auto comp = _ecm.Component<components::LinearAcceleration>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "m/s\u00B2");
      }
    }
    else if (typeId == components::LinearVelocity::typeId)
    {
      auto comp = _ecm.Component<components::LinearVelocity>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "m/s");
      }
    }
    else if (typeId == components::MagneticField::typeId)
    {
      auto comp = _ecm.Component<components::MagneticField>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "T");
      }
    }
    else if (typeId == components::Name::typeId)
    {
      auto comp = _ecm.Component<components::Name>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());

      if (this->dataPtr->entity == this->dataPtr->worldEntity)
        this->dataPtr->worldName = comp->Data();
    }
    else if (typeId == components::ParentEntity::typeId)
    {
      auto comp = _ecm.Component<components::ParentEntity>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::ParentLinkName::typeId)
    {
      auto comp = _ecm.Component<components::ParentLinkName>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::PerformerAffinity::typeId)
    {
      auto comp = _ecm.Component<components::PerformerAffinity>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::Pose::typeId)
    {
      auto comp = _ecm.Component<components::Pose>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::Static::typeId)
    {
      auto comp = _ecm.Component<components::Static>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::SensorTopic::typeId)
    {
      auto comp =
          _ecm.Component<components::SensorTopic>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::WindMode::typeId)
    {
      auto comp = _ecm.Component<components::WindMode>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::WorldAngularAcceleration::typeId)
    {
      auto comp = _ecm.Component<components::WorldAngularAcceleration>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "rad/s\u00B2");
      }
    }
    else if (typeId == components::WorldLinearVelocity::typeId)
    {
      auto comp = _ecm.Component<components::WorldLinearVelocity>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "m/s");
      }
    }
    else if (typeId == components::WorldLinearVelocitySeed::typeId)
    {
      auto comp = _ecm.Component<components::WorldLinearVelocitySeed>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "m/s");
      }
    }
    else if (typeId == components::WorldPose::typeId)
    {
      auto comp = _ecm.Component<components::WorldPose>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::WorldPoseCmd::typeId)
    {
      auto comp = _ecm.Component<components::WorldPoseCmd>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
  }

  // Remove components no longer present
  for (auto itemIt : this->dataPtr->componentsModel.items)
  {
    auto typeId = itemIt.first;
    if (componentTypes.find(typeId) == componentTypes.end())
    {
      QMetaObject::invokeMethod(&this->dataPtr->componentsModel,
          "RemoveComponentType",
          Qt::QueuedConnection,
          Q_ARG(ignition::gazebo::ComponentTypeId, typeId));
    }
  }
}

/////////////////////////////////////////////////
bool ComponentInspector::eventFilter(QObject *_obj, QEvent *_event)
{
  if (!this->dataPtr->locked)
  {
    if (_event->type() == gazebo::gui::events::EntitiesSelected::kType)
    {
      auto event = reinterpret_cast<gui::events::EntitiesSelected *>(_event);
      if (event && !event->Data().empty())
      {
        this->SetEntity(*event->Data().begin());
      }
    }

    if (_event->type() == gazebo::gui::events::DeselectAllEntities::kType)
    {
      auto event = reinterpret_cast<gui::events::DeselectAllEntities *>(
          _event);
      if (event)
      {
        this->SetEntity(kNullEntity);
      }
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
  // If nothing is selected, display world properties
  if (_entity == kNullEntity)
  {
    this->dataPtr->entity = this->dataPtr->worldEntity;
  }
  else
  {
    this->dataPtr->entity = _entity;
  }
  this->EntityChanged();
}

/////////////////////////////////////////////////
QString ComponentInspector::Type() const
{
  return this->dataPtr->type;
}

/////////////////////////////////////////////////
void ComponentInspector::SetType(const QString &_type)
{
  this->dataPtr->type = _type;
  this->TypeChanged();
}

/////////////////////////////////////////////////
bool ComponentInspector::Locked() const
{
  return this->dataPtr->locked;
}

/////////////////////////////////////////////////
void ComponentInspector::SetLocked(bool _locked)
{
  this->dataPtr->locked = _locked;
  this->LockedChanged();
}

/////////////////////////////////////////////////
bool ComponentInspector::Paused() const
{
  return this->dataPtr->paused;
}

/////////////////////////////////////////////////
void ComponentInspector::SetPaused(bool _paused)
{
  this->dataPtr->paused = _paused;
  this->PausedChanged();
}

/////////////////////////////////////////////////
void ComponentInspector::OnPose(double _x, double _y, double _z, double _roll,
    double _pitch, double _yaw)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
        ignerr << "Error setting pose" << std::endl;
  };

  ignition::msgs::Pose req;
  req.set_id(this->dataPtr->entity);
  msgs::Set(req.mutable_position(), math::Vector3d(_x, _y, _z));
  msgs::Set(req.mutable_orientation(), math::Quaterniond(_roll, _pitch, _yaw));
  auto poseCmdService = "/world/" + this->dataPtr->worldName
      + "/set_pose";
  this->dataPtr->node.Request(poseCmdService, req, cb);
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::ComponentInspector,
                    ignition::gui::Plugin)
