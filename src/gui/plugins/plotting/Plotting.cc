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

#include "Plotting.hh"

#include <ignition/plugin/Register.hh>
#include "ignition/gazebo/components/AngularAcceleration.hh"
#include "ignition/gazebo/components/AngularVelocity.hh"
#include "ignition/gazebo/components/CastShadows.hh"
#include "ignition/gazebo/components/Factory.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/LinearAcceleration.hh"
#include "ignition/gazebo/components/LinearVelocity.hh"
#include "ignition/gazebo/components/LinearVelocitySeed.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/PoseCmd.hh"
#include "ignition/gazebo/components/Static.hh"
#include "ignition/gazebo/components/WindMode.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

namespace ignition::gazebo
{
  class PlottingPrivate
  {
    /// \brief Interface to communicate with Qml
    public: std::unique_ptr<gui::PlottingInterface> plottingIface{nullptr};

    /// \brief registered components for plotting
    /// map key: string contains EntityID + "," + ComponentID
    public: std::map<std::string,
      std::shared_ptr<PlotComponent>> components;
  };

  class PlotComponentPrivate
  {
    /// \brief entity id in the simulation
    public: Entity entity;

    /// \brief type identifier unique to each component type
    public: ComponentTypeId typeId;

    /// \brief component data type (Pose3d, Vector3d, double)
    public: std::string type;

    /// \brief attributes of the components,
    /// ex: x,y,z attributes in Vector3d type component
    public: std::map<std::string,
      std::shared_ptr<ignition::gui::PlotData>> data;
  };
}

using namespace ignition::gazebo;
using namespace ignition::gui;

//////////////////////////////////////////////////
PlotComponent::PlotComponent(const std::string &_type,
                             ignition::gazebo::Entity _entity,
                             ComponentTypeId _typeId) :
    dataPtr(std::make_unique<PlotComponentPrivate>())
{
  this->dataPtr->entity = _entity;
  this->dataPtr->typeId = _typeId;
  this->dataPtr->type = _type;

  if (_type == "Vector3d")
  {
    this->dataPtr->data["x"] = std::make_shared<PlotData>();
    this->dataPtr->data["y"] = std::make_shared<PlotData>();
    this->dataPtr->data["z"] = std::make_shared<PlotData>();
  }
  else if (_type == "Pose3d")
  {
    this->dataPtr->data["x"] = std::make_shared<PlotData>();
    this->dataPtr->data["y"] = std::make_shared<PlotData>();
    this->dataPtr->data["z"] = std::make_shared<PlotData>();
    this->dataPtr->data["roll"] = std::make_shared<PlotData>();
    this->dataPtr->data["pitch"] = std::make_shared<PlotData>();
    this->dataPtr->data["yaw"] = std::make_shared<PlotData>();
  }
  else if (_type == "Light")
  {
    this->dataPtr->data["diffuseR"] = std::make_shared<PlotData>();
    this->dataPtr->data["diffuseG"] = std::make_shared<PlotData>();
    this->dataPtr->data["diffuseB"] = std::make_shared<PlotData>();
    this->dataPtr->data["diffuseA"] = std::make_shared<PlotData>();
    this->dataPtr->data["specularR"] = std::make_shared<PlotData>();
    this->dataPtr->data["specularG"] = std::make_shared<PlotData>();
    this->dataPtr->data["specularB"] = std::make_shared<PlotData>();
    this->dataPtr->data["specularA"] = std::make_shared<PlotData>();
    this->dataPtr->data["attRange"] = std::make_shared<PlotData>();
    this->dataPtr->data["attConstant"] = std::make_shared<PlotData>();
    this->dataPtr->data["attLinear"] = std::make_shared<PlotData>();
    this->dataPtr->data["attQuadratic"] = std::make_shared<PlotData>();
    this->dataPtr->data["castshadows"] = std::make_shared<PlotData>();
    this->dataPtr->data["directionX"] = std::make_shared<PlotData>();
    this->dataPtr->data["directionY"] = std::make_shared<PlotData>();
    this->dataPtr->data["directionZ"] = std::make_shared<PlotData>();
    this->dataPtr->data["innerAngle"] = std::make_shared<PlotData>();
    this->dataPtr->data["outerAngle"] = std::make_shared<PlotData>();
    this->dataPtr->data["falloff"] = std::make_shared<PlotData>();
  }
  else if (_type == "double")
    this->dataPtr->data["value"] = std::make_shared<PlotData>();
  else
    ignwarn << "Invalid Plot Component Type:" << _type << std::endl;
}

//////////////////////////////////////////////////
PlotComponent::~PlotComponent()
{
}

//////////////////////////////////////////////////
void PlotComponent::RegisterChart(std::string _attribute, int _chart)
{
  if (this->dataPtr->data.count(_attribute) == 0)
  {
    ignwarn << "Invalid Plot Component Attribute" << std::endl;
    return;
  }
  this->dataPtr->data[_attribute]->AddChart(_chart);
}

//////////////////////////////////////////////////
void PlotComponent::UnRegisterChart(std::string _attribute, int _chart)
{
  if (this->dataPtr->data.count(_attribute) == 0)
  {
    ignwarn << "Invalid Plot Component Attribute" << std::endl;
    return;
  }
  this->dataPtr->data[_attribute]->RemoveChart(_chart);
}

//////////////////////////////////////////////////
bool PlotComponent::HasCharts()
{
  for (auto field : this->dataPtr->data)
    if (field.second->ChartCount() > 0)
      return true;
  return false;
}

//////////////////////////////////////////////////
void PlotComponent::SetAttributeValue(std::string _attribute,
                                      const double &_value)
{
  if (this->dataPtr->data.count(_attribute) > 0)
    this->dataPtr->data[_attribute]->SetValue(_value);
}

//////////////////////////////////////////////////
std::map<std::string, std::shared_ptr<PlotData>> PlotComponent::Data() const
{
  return this->dataPtr->data;
}

//////////////////////////////////////////////////
Entity PlotComponent::Entity()
{
  return this->dataPtr->entity;
}

//////////////////////////////////////////////////
ComponentTypeId PlotComponent::TypeId()
{
  return this->dataPtr->typeId;
}

//////////////////////////////////////////////////
Plotting ::Plotting() : GuiSystem(),
  dataPtr(std::make_unique<PlottingPrivate>())
{
  this->dataPtr->plottingIface = std::make_unique<gui::PlottingInterface>();

  // PlottingInterface connecting
  this->connect(this->dataPtr->plottingIface.get(), SIGNAL(ComponentSubscribe
              (uint64_t, uint64_t, std::string, std::string, int)),
          this, SLOT(RegisterChartToComponent
              (uint64_t, uint64_t, std::string, std::string, int)));

  this->connect(this->dataPtr->plottingIface.get(), SIGNAL(ComponentUnSubscribe
              (uint64_t, uint64_t, std::string, int)),
          this, SLOT(UnRegisterChartToComponent
              (uint64_t, uint64_t, std::string, int)));

  this->connect(this->dataPtr->plottingIface.get(),
          SIGNAL(ComponentName(uint64_t)), this, SLOT(ComponentName(uint64_t)));
}

//////////////////////////////////////////////////
Plotting ::~Plotting()
{
}

//////////////////////////////////////////
void Plotting::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Plotting";
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const ignition::math::Vector3d &_vector)
{
  this->dataPtr->components[_Id]->SetAttributeValue("x", _vector.X());
  this->dataPtr->components[_Id]->SetAttributeValue("y", _vector.Y());
  this->dataPtr->components[_Id]->SetAttributeValue("z", _vector.Z());
}

void Plotting::SetData(std::string _Id, const sdf::Light &_light)
{
  this->dataPtr->components[_Id]->SetAttributeValue("specularR",
    _light.Specular().R());
  this->dataPtr->components[_Id]->SetAttributeValue("specularG",
    _light.Specular().G());
  this->dataPtr->components[_Id]->SetAttributeValue("specularB",
    _light.Specular().B());
  this->dataPtr->components[_Id]->SetAttributeValue("specularA",
    _light.Specular().A());
  this->dataPtr->components[_Id]->SetAttributeValue("diffuseR",
    _light.Diffuse().R());
  this->dataPtr->components[_Id]->SetAttributeValue("diffuseG",
    _light.Diffuse().G());
  this->dataPtr->components[_Id]->SetAttributeValue("diffuseB",
    _light.Diffuse().B());
  this->dataPtr->components[_Id]->SetAttributeValue("diffuseA",
    _light.Diffuse().A());
  this->dataPtr->components[_Id]->SetAttributeValue("attRange",
    _light.AttenuationRange());
  this->dataPtr->components[_Id]->SetAttributeValue("attLinear",
    _light.LinearAttenuationFactor());
  this->dataPtr->components[_Id]->SetAttributeValue("attConstant",
    _light.ConstantAttenuationFactor());
  this->dataPtr->components[_Id]->SetAttributeValue("attQuadratic",
    _light.QuadraticAttenuationFactor());
  this->dataPtr->components[_Id]->SetAttributeValue("castshadows",
    _light.CastShadows());
  this->dataPtr->components[_Id]->SetAttributeValue("directionX",
    _light.Direction().X());
  this->dataPtr->components[_Id]->SetAttributeValue("directionY",
    _light.Direction().Y());
  this->dataPtr->components[_Id]->SetAttributeValue("directionZ",
    _light.Direction().Z());
  this->dataPtr->components[_Id]->SetAttributeValue("innerAngle",
    _light.SpotInnerAngle().Radian());
  this->dataPtr->components[_Id]->SetAttributeValue("outerAngle",
    _light.SpotOuterAngle().Radian());
  this->dataPtr->components[_Id]->SetAttributeValue("falloff",
    _light.SpotFalloff());
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const ignition::math::Pose3d &_pose)
{
  this->dataPtr->components[_Id]->SetAttributeValue("x", _pose.Pos().X());
  this->dataPtr->components[_Id]->SetAttributeValue("y", _pose.Pos().Y());
  this->dataPtr->components[_Id]->SetAttributeValue("z", _pose.Pos().Z());
  this->dataPtr->components[_Id]->SetAttributeValue("roll", _pose.Rot().Roll());
  this->dataPtr->components[_Id]->SetAttributeValue("pitch",
                                                    _pose.Rot().Pitch());
  this->dataPtr->components[_Id]->SetAttributeValue("yaw", _pose.Rot().Yaw());
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const double &_value)
{
  this->dataPtr->components[_Id]->SetAttributeValue("value", _value);
}


//////////////////////////////////////////////////
void Plotting::RegisterChartToComponent(uint64_t _entity, uint64_t _typeId,
                                        std::string _type,
                                        std::string _attribute,
                                        int _chart)
{
  std::string Id = std::to_string(_entity) + "," + std::to_string(_typeId);

  if (this->dataPtr->components.count(Id) == 0)
  {
    this->dataPtr->components[Id] = std::make_shared<PlotComponent>(
          _type, _entity, _typeId);
  }

  this->dataPtr->components[Id]->RegisterChart(_attribute, _chart);
}

//////////////////////////////////////////////////
void Plotting::UnRegisterChartFromComponent(uint64_t _entity, uint64_t _typeId,
                                            std::string _attribute, int _chart)
{
  std::string id = std::to_string(_entity) + "," + std::to_string(_typeId);
  igndbg << "UnRegister [" << id  << "]" << std::endl;

  if (this->dataPtr->components.count(id) == 0)
    return;

  this->dataPtr->components[id]->UnRegisterChart(_attribute, _chart);

  if (!this->dataPtr->components[id]->HasCharts())
      this->dataPtr->components.erase(id);
}

//////////////////////////////////////////////////
std::string Plotting::ComponentName(const uint64_t &_typeId)
{
  std::string name = components::Factory::Instance()->Name(_typeId);

  auto pos = name.find("ign.gazebo.components.");

  if (pos != std::string::npos)
    name.erase(pos, 22);

  return name;
}

//////////////////////////////////////////////////
void Plotting ::Update(const ignition::gazebo::UpdateInfo &_info,
                       ignition::gazebo::EntityComponentManager &_ecm)
{
  for (auto component : this->dataPtr->components)
  {
    auto entity = component.second->Entity();
    auto typeId = component.second->TypeId();

    if (typeId == components::AngularAcceleration::typeId)
    {
      auto comp = _ecm.Component<components::AngularAcceleration>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::AngularVelocity::typeId)
    {
      auto comp = _ecm.Component<components::AngularVelocity>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::CastShadows::typeId)
    {
      auto comp = _ecm.Component<components::CastShadows>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::Gravity::typeId)
    {
      auto comp = _ecm.Component<components::Gravity>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::LinearAcceleration::typeId)
    {
      auto comp = _ecm.Component<components::LinearAcceleration>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::LinearVelocity::typeId)
    {
      auto comp = _ecm.Component<components::LinearVelocity>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::MagneticField::typeId)
    {
      auto comp = _ecm.Component<components::MagneticField>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::ParentEntity::typeId)
    {
      auto comp = _ecm.Component<components::ParentEntity>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::Pose::typeId)
    {
      auto comp = _ecm.Component<components::Pose>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::Static::typeId)
    {
      auto comp = _ecm.Component<components::Static>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::TrajectoryPose::typeId)
    {
      auto comp = _ecm.Component<components::TrajectoryPose>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::WindMode::typeId)
    {
      auto comp = _ecm.Component<components::WindMode>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::WorldAngularAcceleration::typeId)
    {
      auto comp = _ecm.Component<components::WorldAngularAcceleration>(
                  entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::WorldLinearVelocity::typeId)
    {
      auto comp = _ecm.Component<components::WorldLinearVelocity>(
                  entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::WorldLinearVelocitySeed::typeId)
    {
      auto comp = _ecm.Component<components::WorldLinearVelocitySeed>(
                  entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::WorldPose::typeId)
    {
      auto comp = _ecm.Component<components::WorldPose>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::WorldPoseCmd::typeId)
    {
      auto comp = _ecm.Component<components::WorldPoseCmd>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }
    else if (typeId == components::Light::typeId)
    {
      auto comp = _ecm.Component<components::Light>(entity);
      if (comp)
        this->SetData(component.first, comp->Data());
    }

    for (auto attribute : component.second->Data())
    {
      for (auto chart : attribute.second->Charts())
      {
        QString attributeName = QString::fromStdString(
                    component.first + "," + attribute.first);
        double y = attribute.second->Value();

        double x = _info.simTime.count() * std::pow(10, -9);

        emit this->dataPtr->plottingIface->plot(chart, attributeName, x, y);
      }
    }
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Plotting ,
                    ignition::gazebo::GuiSystem,
                    ignition::gui::Plugin)
