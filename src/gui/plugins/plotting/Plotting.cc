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

#include <gz/math/Pose3.hh>
#include <gz/math/SphericalCoordinates.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/components/AngularAcceleration.hh"
#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/CastShadows.hh"
#include "gz/sim/components/Factory.hh"
#include "gz/sim/components/Gravity.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearVelocitySeed.hh"
#include "gz/sim/components/MagneticField.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/PoseCmd.hh"
#include "gz/sim/components/SphericalCoordinates.hh"
#include "gz/sim/components/Static.hh"
#include "gz/sim/components/WindMode.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/EntityComponentManager.hh"

namespace gz::sim
{
  class PlottingPrivate
  {
    /// \brief Interface to communicate with Qml
    public: std::unique_ptr<gui::PlottingInterface> plottingIface{nullptr};

    /// \brief registered components for plotting
    /// map key: string contains EntityID + "," + ComponentID
    public: std::map<std::string,
      std::shared_ptr<PlotComponent>> components;

    /// \brief Mutex to protect the components map.
    public: std::recursive_mutex componentsMutex;
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
      std::shared_ptr<gz::gui::PlotData>> data;
  };
}

using namespace gz;
using namespace gz::sim;
using namespace gz::gui;

//////////////////////////////////////////////////
PlotComponent::PlotComponent(const std::string &_type,
                             gz::sim::Entity _entity,
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
    this->dataPtr->data["intensity"] = std::make_shared<PlotData>();
  }
  else if (_type == "double")
    this->dataPtr->data["value"] = std::make_shared<PlotData>();
  else if (_type == "Physics")
  {
    this->dataPtr->data["stepSize"] = std::make_shared<PlotData>();
    this->dataPtr->data["realTimeFactor"] = std::make_shared<PlotData>();
  }
  else if (_type == "SphericalCoordinates")
  {
    this->dataPtr->data["latitude"] = std::make_shared<PlotData>();
    this->dataPtr->data["longitude"] = std::make_shared<PlotData>();
    this->dataPtr->data["elevation"] = std::make_shared<PlotData>();
    this->dataPtr->data["heading"] = std::make_shared<PlotData>();
  }
  else
    gzwarn << "Invalid Plot Component Type:" << _type << std::endl;
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
    gzwarn << "Invalid Plot Component Attribute" << std::endl;
    return;
  }
  this->dataPtr->data[_attribute]->AddChart(_chart);
}

//////////////////////////////////////////////////
void PlotComponent::UnRegisterChart(std::string _attribute, int _chart)
{
  if (this->dataPtr->data.count(_attribute) == 0)
  {
    gzwarn << "Invalid Plot Component Attribute" << std::endl;
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
Plotting::Plotting() : GuiSystem(),
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
          this, SLOT(UnRegisterChartFromComponent
              (uint64_t, uint64_t, std::string, int)));

  this->connect(this->dataPtr->plottingIface.get(),
          SIGNAL(ComponentName(uint64_t)), this, SLOT(ComponentName(uint64_t)));
}

//////////////////////////////////////////////////
Plotting::~Plotting()
{
}

//////////////////////////////////////////
void Plotting::LoadConfig(const tinyxml2::XMLElement *)
{
  if (this->title.empty())
    this->title = "Plotting";
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const gz::math::Vector3d &_vector)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->componentsMutex);
  this->dataPtr->components[_Id]->SetAttributeValue("x", _vector.X());
  this->dataPtr->components[_Id]->SetAttributeValue("y", _vector.Y());
  this->dataPtr->components[_Id]->SetAttributeValue("z", _vector.Z());
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const gz::msgs::Light &_light)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->componentsMutex);
  if (_light.has_specular())
  {
    this->dataPtr->components[_Id]->SetAttributeValue("specularR",
      _light.specular().r());
    this->dataPtr->components[_Id]->SetAttributeValue("specularG",
      _light.specular().g());
    this->dataPtr->components[_Id]->SetAttributeValue("specularB",
      _light.specular().b());
    this->dataPtr->components[_Id]->SetAttributeValue("specularA",
      _light.specular().a());
  }
  if (_light.has_diffuse())
  {
    this->dataPtr->components[_Id]->SetAttributeValue("diffuseR",
      _light.diffuse().r());
    this->dataPtr->components[_Id]->SetAttributeValue("diffuseG",
      _light.diffuse().g());
    this->dataPtr->components[_Id]->SetAttributeValue("diffuseB",
      _light.diffuse().b());
    this->dataPtr->components[_Id]->SetAttributeValue("diffuseA",
      _light.diffuse().a());
  }
  this->dataPtr->components[_Id]->SetAttributeValue("attRange",
    _light.range());
  this->dataPtr->components[_Id]->SetAttributeValue("attLinear",
    _light.attenuation_linear());
  this->dataPtr->components[_Id]->SetAttributeValue("attConstant",
    _light.attenuation_constant());
  this->dataPtr->components[_Id]->SetAttributeValue("attQuadratic",
    _light.attenuation_quadratic());
  this->dataPtr->components[_Id]->SetAttributeValue("castshadows",
    _light.cast_shadows());
  this->dataPtr->components[_Id]->SetAttributeValue("intensity",
    _light.intensity());
  if (_light.has_direction())
  {
    this->dataPtr->components[_Id]->SetAttributeValue("directionX",
      _light.direction().x());
    this->dataPtr->components[_Id]->SetAttributeValue("directionY",
      _light.direction().y());
    this->dataPtr->components[_Id]->SetAttributeValue("directionZ",
      _light.direction().z());
  }
  this->dataPtr->components[_Id]->SetAttributeValue("innerAngle",
    _light.spot_inner_angle());
  this->dataPtr->components[_Id]->SetAttributeValue("outerAngle",
    _light.spot_outer_angle());
  this->dataPtr->components[_Id]->SetAttributeValue("falloff",
    _light.spot_falloff());
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const gz::math::Pose3d &_pose)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->componentsMutex);
  this->dataPtr->components[_Id]->SetAttributeValue("x", _pose.Pos().X());
  this->dataPtr->components[_Id]->SetAttributeValue("y", _pose.Pos().Y());
  this->dataPtr->components[_Id]->SetAttributeValue("z", _pose.Pos().Z());
  this->dataPtr->components[_Id]->SetAttributeValue("roll", _pose.Rot().Roll());
  this->dataPtr->components[_Id]->SetAttributeValue("pitch",
                                                    _pose.Rot().Pitch());
  this->dataPtr->components[_Id]->SetAttributeValue("yaw", _pose.Rot().Yaw());
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const math::SphericalCoordinates &_sc)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->componentsMutex);
  this->dataPtr->components[_Id]->SetAttributeValue("latitude",
      _sc.LatitudeReference().Degree());
  this->dataPtr->components[_Id]->SetAttributeValue("longitude",
      _sc.LongitudeReference().Degree());
  this->dataPtr->components[_Id]->SetAttributeValue("elevation",
      _sc.ElevationReference());
  this->dataPtr->components[_Id]->SetAttributeValue("heading",
      _sc.HeadingOffset().Degree());
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const sdf::Physics &_physics)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->componentsMutex);
  this->dataPtr->components[_Id]->SetAttributeValue("stepSize",
      _physics.MaxStepSize());
  this->dataPtr->components[_Id]->SetAttributeValue("realTimeFactor",
      _physics.RealTimeFactor());
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const double &_value)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->componentsMutex);
  this->dataPtr->components[_Id]->SetAttributeValue("value", _value);
}


//////////////////////////////////////////////////
void Plotting::RegisterChartToComponent(uint64_t _entity, uint64_t _typeId,
                                        std::string _type,
                                        std::string _attribute,
                                        int _chart)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->componentsMutex);
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
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->componentsMutex);
  std::string id = std::to_string(_entity) + "," + std::to_string(_typeId);
  gzdbg << "UnRegister [" << id  << "]" << std::endl;

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
void Plotting::Update(const gz::sim::UpdateInfo &_info,
                       gz::sim::EntityComponentManager &_ecm)
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->componentsMutex);
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
    else if (typeId == components::Physics::typeId)
    {
      auto comp = _ecm.Component<components::Physics>(entity);
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
    else if (typeId == components::SphericalCoordinates::typeId)
    {
      auto comp = _ecm.Component<components::SphericalCoordinates>(entity);
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
      {
        gz::msgs::Light lightMsgs =
          convert<gz::msgs::Light>(comp->Data());
        this->SetData(component.first, lightMsgs);
      }
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
GZ_ADD_PLUGIN(gz::sim::Plotting,
                    gz::sim::GuiSystem,
                    gz::gui::Plugin)
