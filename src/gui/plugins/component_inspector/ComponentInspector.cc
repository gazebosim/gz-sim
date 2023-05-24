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
#include <list>
#include <regex>
#include <unordered_map>
#include <ignition/common/Console.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/gui/Application.hh>
#include <ignition/gui/MainWindow.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/components/Actor.hh"
#include "ignition/gazebo/components/AngularAcceleration.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/BatterySoC.hh"
#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/CenterOfVolume.hh"
#include "ignition/gazebo/components/ChildLinkName.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Joint.hh"
#include "ignition/gazebo/components/LaserRetro.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LightCmd.hh"
#include "ignition/gazebo/components/LightType.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/LinearVelocitySeed.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/Material.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/Physics.hh"
#include "ignition/gazebo/components/PhysicsEnginePlugin.hh"
#include "ignition/gazebo/components/RenderEngineGuiPlugin.hh"
#include "ignition/gazebo/components/RenderEngineServerPlugin.hh"
#include "ignition/gazebo/components/SelfCollide.hh"
#include "ignition/gazebo/components/Sensor.hh"
#include "ignition/gazebo/components/SourceFilePath.hh"
#include "ignition/gazebo/components/SphericalCoordinates.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/SystemPluginInfo.hh"
#include "ignition/gazebo/components/ThreadPitch.hh"
#include "ignition/gazebo/components/Transparency.hh"
#include "ignition/gazebo/components/Visual.hh"
#include "ignition/gazebo/components/Volume.hh"
#include "ignition/gazebo/components/WindMode.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/gui/GuiEvents.hh"

#include "ComponentInspector.hh"
#include "Pose3d.hh"
#include "SystemPluginInfo.hh"

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

    /// \brief Entity name
    public: std::string entityName;

    /// \brief Entity type, such as 'world' or 'model'.
    public: QString type;

    /// \brief Nested model or not
    public: bool nestedModel = false;

    /// \brief Whether currently locked on a given entity
    public: bool locked{false};

    /// \brief Whether updates are currently paused.
    public: bool paused{false};

    /// \brief Transport node for making command requests
    public: transport::Node node;

    /// \brief A map of component types to the function used to update it.
    public: std::map<ComponentTypeId, inspector::UpdateViewCb>
        updateViewCbs;

    /// \brief Handles all components displayed as a 3D pose.
    public: std::unique_ptr<inspector::Pose3d> pose3d;

    /// \brief Handles all system info components.
    public: std::unique_ptr<inspector::SystemPluginInfo> systemInfo;

    /// \brief A list of system plugin human readable names.
    public: QStringList systemNameList;

    /// \brief Maps plugin display names to their filenames.
    public: std::unordered_map<std::string, std::string> systemMap;
  };
}

// Helper to remove a prefix from a string if present
void removePrefix(const std::string &_prefix, std::string &_s)
{
  auto id = _s.find(_prefix);
  if (id != std::string::npos)
  {
    _s = _s.substr(_prefix.length());
  }
}

// Helper to remove a suffix from a string if present
void removeSuffix(const std::string &_suffix, std::string &_s)
{
  auto id = _s.find(_suffix);
  if (id != std::string::npos && id + _suffix.length() == _s.length())
  {
    _s.erase(id, _suffix.length());
  }
}

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item, const msgs::Light &_data)
{
  if (nullptr == _item)
    return;

  int lightType = -1;
  if (_data.type() == msgs::Light::POINT)
  {
    lightType = 0;
  }
  else if (_data.type() == msgs::Light::SPOT)
  {
    lightType = 1;
  }
  else if (_data.type() == msgs::Light::DIRECTIONAL)
  {
    lightType = 2;
  }

  bool visualizeVisual = true;
  for (int i = 0; i < _data.header().data_size(); ++i)
  {
    for (int j = 0;
        j < _data.header().data(i).value_size(); ++j)
    {
      if (_data.header().data(i).key() ==
          "visualizeVisual")
      {
        visualizeVisual = ignition::math::parseInt(
          _data.header().data(i).value(0));
      }
    }
  }

  bool isLightOn = true;
  for (int i = 0; i < _data.header().data_size(); ++i)
  {
    for (int j = 0;
        j < _data.header().data(i).value_size(); ++j)
    {
      if (_data.header().data(i).key() ==
          "isLightOn")
      {
        isLightOn = ignition::math::parseInt(
          _data.header().data(i).value(0));
      }
    }
  }

  _item->setData(QString("Light"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QList({
    QVariant(_data.specular().r()),
    QVariant(_data.specular().g()),
    QVariant(_data.specular().b()),
    QVariant(_data.specular().a()),
    QVariant(_data.diffuse().r()),
    QVariant(_data.diffuse().g()),
    QVariant(_data.diffuse().b()),
    QVariant(_data.diffuse().a()),
    QVariant(_data.range()),
    QVariant(_data.attenuation_linear()),
    QVariant(_data.attenuation_constant()),
    QVariant(_data.attenuation_quadratic()),
    QVariant(_data.cast_shadows()),
    QVariant(_data.direction().x()),
    QVariant(_data.direction().y()),
    QVariant(_data.direction().z()),
    QVariant(_data.spot_inner_angle()),
    QVariant(_data.spot_outer_angle()),
    QVariant(_data.spot_falloff()),
    QVariant(_data.intensity()),
    QVariant(lightType),
    QVariant(isLightOn),
    QVariant(visualizeVisual)
  }), ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item,
    const math::Vector3d &_data)
{
  if (nullptr == _item)
    return;

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
void gazebo::setData(QStandardItem *_item, const std::string &_data)
{
  if (nullptr == _item)
    return;

  _item->setData(QString("String"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QString::fromStdString(_data),
      ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item,
    const std::ostringstream &_data)
{
  if (nullptr == _item)
    return;

  _item->setData(QString("Raw"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QString::fromStdString(_data.str()),
      ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item, const bool &_data)
{
  if (nullptr == _item)
    return;

  _item->setData(QString("Boolean"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(_data, ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item, const int &_data)
{
  if (nullptr == _item)
    return;

  _item->setData(QString("Integer"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(_data, ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item, const Entity &_data)
{
  setData(_item, static_cast<int>(_data));
}

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item, const double &_data)
{
  if (nullptr == _item)
    return;

  _item->setData(QString("Float"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(_data, ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item, const sdf::Physics &_data)
{
  if (nullptr == _item)
    return;

  _item->setData(QString("Physics"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QList({
    QVariant(_data.MaxStepSize()),
    QVariant(_data.RealTimeFactor())
  }), ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item,
    const sdf::Material &_data)
{
  if (nullptr == _item)
    return;

  _item->setData(QString("Material"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QList({
    QVariant(_data.Ambient().R()),
    QVariant(_data.Ambient().G()),
    QVariant(_data.Ambient().B()),
    QVariant(_data.Ambient().A()),
    QVariant(_data.Diffuse().R()),
    QVariant(_data.Diffuse().G()),
    QVariant(_data.Diffuse().B()),
    QVariant(_data.Diffuse().A()),
    QVariant(_data.Specular().R()),
    QVariant(_data.Specular().G()),
    QVariant(_data.Specular().B()),
    QVariant(_data.Specular().A()),
    QVariant(_data.Emissive().R()),
    QVariant(_data.Emissive().G()),
    QVariant(_data.Emissive().B()),
    QVariant(_data.Emissive().A())
  }), ComponentsModel::RoleNames().key("data"));

  // TODO(anyone) Only shows colors of material,
  // need to add others (e.g., pbr)
}

//////////////////////////////////////////////////
template<>
void gazebo::setData(QStandardItem *_item,
    const math::SphericalCoordinates &_data)
{
  if (nullptr == _item)
    return;

  _item->setData(QString("SphericalCoordinates"),
      ComponentsModel::RoleNames().key("dataType"));
  _item->setData(QList({
    QVariant(QString::fromStdString(math::SphericalCoordinates::Convert(
        _data.Surface()))),
    QVariant(_data.LatitudeReference().Degree()),
    QVariant(_data.LongitudeReference().Degree()),
    QVariant(_data.ElevationReference()),
    QVariant(_data.HeadingOffset().Degree()),
  }), ComponentsModel::RoleNames().key("data"));
}

//////////////////////////////////////////////////
void gazebo::setUnit(QStandardItem *_item, const std::string &_unit)
{
  if (nullptr == _item)
    return;

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
    ComponentTypeId _typeId)
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
      ComponentTypeId _typeId)
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
          std::pair(105, "data"),
          std::pair(106, "entity")};
}

/////////////////////////////////////////////////
ComponentInspector::ComponentInspector()
  : GuiSystem(), dataPtr(std::make_unique<ComponentInspectorPrivate>())
{
  qRegisterMetaType<ComponentTypeId>();
  qRegisterMetaType<Entity>("Entity");
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

  // Type-specific handlers
  this->dataPtr->pose3d = std::make_unique<inspector::Pose3d>(this);
  this->dataPtr->systemInfo =
      std::make_unique<inspector::SystemPluginInfo>(this);
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

      // check if entity is nested model
      auto parentComp = _ecm.Component<components::ParentEntity>(
           this->dataPtr->entity);
      if (parentComp)
      {
        auto modelComp = _ecm.Component<components::Model>(parentComp->Data());
        this->dataPtr->nestedModel = (modelComp);
      }
      this->NestedModelChanged();

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
      item = this->dataPtr->componentsModel.AddComponentType(typeId);
    }

    item->setData(QString::number(this->dataPtr->entity),
                  ComponentsModel::RoleNames().key("entity"));

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
    else if (typeId == components::AnimationName::typeId)
    {
      auto comp = _ecm.Component<components::AnimationName>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::BatterySoC::typeId)
    {
      auto comp = _ecm.Component<components::BatterySoC>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::CastShadows::typeId)
    {
      auto comp = _ecm.Component<components::CastShadows>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::CenterOfVolume::typeId)
    {
      auto comp = _ecm.Component<components::CenterOfVolume>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "m");
      }
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
    else if (typeId == components::LaserRetro::typeId)
    {
      auto comp = _ecm.Component<components::LaserRetro>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
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
      this->dataPtr->entityName = comp->Data();
    }
    else if (typeId == components::LightType::typeId)
    {
      auto comp = _ecm.Component<components::LightType>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
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
    else if (typeId == components::Light::typeId)
    {
      this->SetType("light");
      auto comp = _ecm.Component<components::Light>(this->dataPtr->entity);
      if (comp)
      {
        msgs::Light lightMsgs = convert<msgs::Light>(comp->Data());
        setData(item, lightMsgs);
      }
    }
    else if (typeId == components::Physics::typeId)
    {
      auto comp = _ecm.Component<components::Physics>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::PhysicsCollisionDetector::typeId)
    {
      auto comp = _ecm.Component<components::PhysicsCollisionDetector>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::PhysicsSolver::typeId)
    {
      auto comp = _ecm.Component<components::PhysicsSolver>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::RenderEngineGuiPlugin::typeId)
    {
      auto comp = _ecm.Component<components::RenderEngineGuiPlugin>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::RenderEngineServerPlugin::typeId)
    {
      auto comp = _ecm.Component<components::RenderEngineServerPlugin>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::Static::typeId)
    {
      auto comp = _ecm.Component<components::Static>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::SelfCollide::typeId)
    {
      auto comp =
          _ecm.Component<components::SelfCollide>(this->dataPtr->entity);
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
    else if (typeId == components::SourceFilePath::typeId)
    {
      auto comp =
          _ecm.Component<components::SourceFilePath>(this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::SphericalCoordinates::typeId)
    {
      auto comp = _ecm.Component<components::SphericalCoordinates>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::ThreadPitch::typeId)
    {
      auto comp = _ecm.Component<components::ThreadPitch>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "m");
      }
    }
    else if (typeId == components::Transparency::typeId)
    {
      auto comp = _ecm.Component<components::Transparency>(
          this->dataPtr->entity);
      if (comp)
        setData(item, comp->Data());
    }
    else if (typeId == components::Volume::typeId)
    {
      auto comp = _ecm.Component<components::Volume>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "m\u00B3");
      }
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
    else if (typeId == components::WorldAngularVelocity::typeId)
    {
      auto comp = _ecm.Component<components::WorldAngularVelocity>(
          this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
        setUnit(item, "rad/s");
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
    else if (this->dataPtr->updateViewCbs.find(typeId) !=
          this->dataPtr->updateViewCbs.end())
    {
      this->dataPtr->updateViewCbs[typeId](_ecm, item);
    }
    else if (typeId == components::Material::typeId)
    {
      auto comp = _ecm.Component<components::Material>(this->dataPtr->entity);
      if (comp)
      {
        setData(item, comp->Data());
      }
    }
  }

  // Remove components no longer present - list items to remove
  std::list<ignition::gazebo::ComponentTypeId> itemsToRemove;
  for (auto itemIt : this->dataPtr->componentsModel.items)
  {
    auto typeId = itemIt.first;
    if (componentTypes.find(typeId) == componentTypes.end())
    {
      itemsToRemove.push_back(typeId);
    }
  }

  // Remove components in list
  for (auto typeId : itemsToRemove)
  {
    QMetaObject::invokeMethod(&this->dataPtr->componentsModel,
        "RemoveComponentType",
        Qt::QueuedConnection,
        Q_ARG(ignition::gazebo::ComponentTypeId, typeId));
  }
}

/////////////////////////////////////////////////
void ComponentInspector::AddUpdateViewCb(ComponentTypeId _id,
    inspector::UpdateViewCb _cb)
{
  this->dataPtr->updateViewCbs[_id] = _cb;
}

/////////////////////////////////////////////////
bool ComponentInspector::eventFilter(QObject *_obj, QEvent *_event)
{
  if (!this->dataPtr->locked)
  {
    if (_event->type() == gui::events::EntitiesSelected::kType)
    {
      auto event = reinterpret_cast<gui::events::EntitiesSelected *>(_event);
      if (event && !event->Data().empty())
      {
        this->SetEntity(*event->Data().begin());
      }
    }

    if (_event->type() == gui::events::DeselectAllEntities::kType)
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
Entity ComponentInspector::GetEntity() const
{
  return this->dataPtr->entity;
}

/////////////////////////////////////////////////
void ComponentInspector::SetEntity(const Entity &_entity)
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
void ComponentInspector::OnLight(
  double _rSpecular, double _gSpecular, double _bSpecular, double _aSpecular,
  double _rDiffuse, double _gDiffuse, double _bDiffuse, double _aDiffuse,
  double _attRange, double _attLinear, double _attConstant,
  double _attQuadratic, bool _castShadows, double _directionX,
  double _directionY, double _directionZ, double _innerAngle,
  double _outerAngle, double _falloff, double _intensity, int _type,
  bool _isLightOn, bool _visualizeVisual)
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting light configuration" << std::endl;
  };

  ignition::msgs::Light req;
  {
    // todo(ahcorde) Use the field is_light_off in light.proto from
    // Garden on.
    auto header = req.mutable_header()->add_data();
    header->set_key("isLightOn");
    std::string *value = header->add_value();
    *value = std::to_string(_isLightOn);
  }
  {
    // todo(ahcorde) Use the field visualize_visual in light.proto from
    // Garden on.
    auto header = req.mutable_header()->add_data();
    header->set_key("visualizeVisual");
    std::string *value = header->add_value();
    *value = std::to_string(_visualizeVisual);
  }

  req.set_name(this->dataPtr->entityName);
  req.set_id(this->dataPtr->entity);
  ignition::msgs::Set(req.mutable_diffuse(),
    ignition::math::Color(_rDiffuse, _gDiffuse, _bDiffuse, _aDiffuse));
  ignition::msgs::Set(req.mutable_specular(),
    ignition::math::Color(_rSpecular, _gSpecular, _bSpecular, _aSpecular));
  req.set_range(_attRange);
  req.set_attenuation_linear(_attLinear);
  req.set_attenuation_constant(_attConstant);
  req.set_attenuation_quadratic(_attQuadratic);
  req.set_cast_shadows(_castShadows);
  req.set_intensity(_intensity);
  if (_type == 0)
    req.set_type(ignition::msgs::Light::POINT);
  else if (_type == 1)
    req.set_type(ignition::msgs::Light::SPOT);
  else
    req.set_type(ignition::msgs::Light::DIRECTIONAL);

  if (_type == 1)  // sdf::LightType::SPOT
  {
    req.set_spot_inner_angle(_innerAngle);
    req.set_spot_outer_angle(_outerAngle);
    req.set_spot_falloff(_falloff);
  }

  // if sdf::LightType::SPOT || sdf::LightType::DIRECTIONAL
  if (_type == 1 || _type == 2)
  {
    ignition::msgs::Set(req.mutable_direction(),
      ignition::math::Vector3d(_directionX, _directionY, _directionZ));
  }

  auto lightConfigService = "/world/" + this->dataPtr->worldName +
    "/light_config";
  lightConfigService = transport::TopicUtils::AsValidTopic(lightConfigService);
  if (lightConfigService.empty())
  {
    ignerr << "Invalid light command service topic provided" << std::endl;
    return;
  }
  this->dataPtr->node.Request(lightConfigService, req, cb);
}

/////////////////////////////////////////////////
void ComponentInspector::OnPhysics(double _stepSize, double _realTimeFactor)
{
  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
        ignerr << "Error setting physics parameters" << std::endl;
  };

  msgs::Physics req;
  req.set_max_step_size(_stepSize);
  req.set_real_time_factor(_realTimeFactor);
  auto physicsCmdService = "/world/" + this->dataPtr->worldName
      + "/set_physics";
  physicsCmdService = transport::TopicUtils::AsValidTopic(physicsCmdService);
  if (physicsCmdService.empty())
  {
    ignerr << "Invalid physics command service topic provided" << std::endl;
    return;
  }
  this->dataPtr->node.Request(physicsCmdService, req, cb);
}

/////////////////////////////////////////////////
void ComponentInspector::OnMaterialColor(
  double _rAmbient, double _gAmbient, double _bAmbient, double _aAmbient,
  double _rDiffuse, double _gDiffuse, double _bDiffuse, double _aDiffuse,
  double _rSpecular, double _gSpecular, double _bSpecular, double _aSpecular,
  double _rEmissive, double _gEmissive, double _bEmissive, double _aEmissive,
  QString /*_type*/, QColor /*_currColor*/)
{
  std::function<void(const ignition::msgs::Boolean &, const bool)> cb =
      [](const ignition::msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting material color configuration"
             << " on visual" << std::endl;
  };

  msgs::Visual req;
  req.set_id(this->dataPtr->entity);

  msgs::Set(req.mutable_material()->mutable_ambient(),
    math::Color(_rAmbient, _gAmbient, _bAmbient, _aAmbient));
  msgs::Set(req.mutable_material()->mutable_diffuse(),
    math::Color(_rDiffuse, _gDiffuse, _bDiffuse, _aDiffuse));
  msgs::Set(req.mutable_material()->mutable_specular(),
    math::Color(_rSpecular, _gSpecular, _bSpecular, _aSpecular));
  msgs::Set(req.mutable_material()->mutable_emissive(),
    math::Color(_rEmissive, _gEmissive, _bEmissive, _aEmissive));

  auto materialCmdService = "/world/" + this->dataPtr->worldName
      + "/visual_config";
  materialCmdService = transport::TopicUtils::AsValidTopic(materialCmdService);
  if (materialCmdService.empty())
  {
    ignerr << "Invalid material command service topic provided" << std::endl;
    return;
  }
  this->dataPtr->node.Request(materialCmdService, req, cb);
}

/////////////////////////////////////////////////
void ComponentInspector::OnSphericalCoordinates(QString _surface,
    double _latitude, double _longitude, double _elevation,
    double _heading)
{
  if (_surface != QString("EARTH_WGS84"))
  {
    ignerr << "Surface [" << _surface.toStdString() << "] not supported."
           << std::endl;
    return;
  }

  std::function<void(const msgs::Boolean &, const bool)> cb =
      [](const msgs::Boolean &/*_rep*/, const bool _result)
  {
    if (!_result)
      ignerr << "Error setting spherical coordinates." << std::endl;
  };

  msgs::SphericalCoordinates req;
  req.set_surface_model(msgs::SphericalCoordinates::EARTH_WGS84);
  req.set_latitude_deg(_latitude);
  req.set_longitude_deg(_longitude);
  req.set_elevation(_elevation);
  req.set_heading_deg(_heading);

  auto sphericalCoordsCmdService = "/world/" + this->dataPtr->worldName
      + "/set_spherical_coordinates";
  sphericalCoordsCmdService =
      transport::TopicUtils::AsValidTopic(sphericalCoordsCmdService);
  if (sphericalCoordsCmdService.empty())
  {
    ignerr << "Invalid spherical coordinates service" << std::endl;
    return;
  }
  this->dataPtr->node.Request(sphericalCoordsCmdService, req, cb);
}

/////////////////////////////////////////////////
bool ComponentInspector::NestedModel() const
{
  return this->dataPtr->nestedModel;
}

/////////////////////////////////////////////////
const std::string &ComponentInspector::WorldName() const
{
  return this->dataPtr->worldName;
}

/////////////////////////////////////////////////
transport::Node &ComponentInspector::TransportNode()
{
  return this->dataPtr->node;
}

/////////////////////////////////////////////////
void ComponentInspector::QuerySystems()
{
  msgs::Empty req;
  msgs::EntityPlugin_V res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/" + this->dataPtr->worldName +
      "/system/info"};
  if (!this->dataPtr->node.Request(service, req, timeout, res, result))
  {
    ignerr << "Unable to query available systems." << std::endl;
    return;
  }

  this->dataPtr->systemNameList.clear();
  this->dataPtr->systemMap.clear();
  for (const auto &plugin : res.plugins())
  {
    if (plugin.filename().empty())
    {
      ignerr << "Received empty plugin name. This shouldn't happen."
             << std::endl;
      continue;
    }

    // Remove common prefixes and suffixes
    auto humanReadable = plugin.filename();
    removePrefix("ignition-gazebo-", humanReadable);
    removePrefix("ignition-gazebo" +
        std::string(IGNITION_GAZEBO_MAJOR_VERSION_STR) + "-", humanReadable);
    removeSuffix("-system", humanReadable);
    removeSuffix("system", humanReadable);
    removeSuffix("-plugin", humanReadable);
    removeSuffix("plugin", humanReadable);

    // Replace - with space, capitalize
    std::replace(humanReadable.begin(), humanReadable.end(), '-', ' ');
    humanReadable[0] = std::toupper(humanReadable[0]);

    this->dataPtr->systemMap[humanReadable] = plugin.filename();
    this->dataPtr->systemNameList.push_back(
        QString::fromStdString(humanReadable));
  }
  this->dataPtr->systemNameList.sort();
  this->dataPtr->systemNameList.removeDuplicates();
  this->SystemNameListChanged();
}

/////////////////////////////////////////////////
QStringList ComponentInspector::SystemNameList() const
{
  return this->dataPtr->systemNameList;
}

/////////////////////////////////////////////////
void ComponentInspector::SetSystemNameList(const QStringList &_list)
{
  this->dataPtr->systemNameList = _list;
}

/////////////////////////////////////////////////
void ComponentInspector::OnAddSystem(const QString &_name,
    const QString &_filename, const QString &_innerxml)
{
  auto filenameStr = _filename.toStdString();
  auto it = this->dataPtr->systemMap.find(filenameStr);
  if (it == this->dataPtr->systemMap.end())
  {
    ignerr << "Internal error: failed to find [" << filenameStr
           << "] in system map." << std::endl;
    return;
  }

  msgs::EntityPlugin_V req;
  auto ent = req.mutable_entity();
  ent->set_id(this->dataPtr->entity);
  auto plugin = req.add_plugins();
  std::string name = _name.toStdString();
  std::string filename = this->dataPtr->systemMap[filenameStr];
  std::string innerxml = _innerxml.toStdString();
  plugin->set_name(name);
  plugin->set_filename(filename);
  plugin->set_innerxml(innerxml);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/" + this->dataPtr->worldName +
      "/entity/system/add"};
  if (!this->dataPtr->node.Request(service, req, timeout, res, result))
  {
    ignerr << "Error adding new system to entity: "
           << this->dataPtr->entity << "\n"
           << "Name: " << name << "\n"
           << "Filename: " << filename << "\n"
           << "Inner XML: " << innerxml << std::endl;
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ComponentInspector,
                    ignition::gui::Plugin)
