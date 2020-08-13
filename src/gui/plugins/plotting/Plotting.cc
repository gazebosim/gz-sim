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
#include <ignition/plugin/Register.hh>

#include "Plotting.hh"

namespace ignition::gazebo
{
  class PlottingPrivate
  {
    /// \brief Interface to communicate with Qml
    public: ignition::gui::PlottingInterface *plottingIface;

    public: std::map<std::string, PlotComponent*> components;

    public: QTimer *timer;
  };
}

using namespace ignition::gazebo;
using namespace ignition::gui;

//////////////////////////////////////////////////
PlotComponent::PlotComponent(std::string _type, uint64_t _entity,
                             uint64_t _typeId)
{
  this->entity = _entity;
  this->typeId = _typeId;
  this->type = _type;

  if (_type == "Vector3d")
  {
    this->data["x"] = new PlotData();
    this->data["y"] = new PlotData();
    this->data["z"] = new PlotData();
  }
  else if (_type == "Pose3d")
  {
    this->data["x"] = new PlotData();
    this->data["y"] = new PlotData();
    this->data["z"] = new PlotData();
    this->data["roll"] = new PlotData();
    this->data["pitch"] = new PlotData();
    this->data["yaw"] = new PlotData();
  }
  else if (_type == "double")
      this->data["value"] = new PlotData();
  else
    ignwarn << "Invalid Plot Component Type:" << _type << std::endl;
}

//////////////////////////////////////////////////
void PlotComponent::RegisterChart(std::string _attribute, int _chart)
{
  if (this->data.count(_attribute) == 0)
  {
    ignwarn << "Invalid Plot Component Attribute" << std::endl;
    return;
  }
  this->data[_attribute]->AddChart(_chart);
}

//////////////////////////////////////////////////
void PlotComponent::UnRegisterChart(std::string _attribute, int _chart)
{
  if (this->data.count(_attribute) == 0)
  {
    ignwarn << "Invalid Plot Component Attribute" << std::endl;
    return;
  }
  this->data[_attribute]->RemoveChart(_chart);
}

//////////////////////////////////////////////////
bool PlotComponent::HasCharts()
{
  for (auto field : data)
    if (field.second->ChartCount() > 0)
        return true;
  return false;
}

//////////////////////////////////////////////////
void PlotComponent::SetAttributeValue(std::string _attribute,
                                      const double &_value)
{
  if (this->data.count(_attribute) > 0)
    this->data[_attribute]->SetValue(_value);
}

//////////////////////////////////////////////////
std::map<std::string, PlotData*> PlotComponent::Data() const
{
  return this->data;
}

//////////////////////////////////////////////////
uint64_t PlotComponent::Entity()
{
  return this->entity;
}

//////////////////////////////////////////////////
uint64_t PlotComponent::TypeId()
{
  return this->typeId;
}

// ======================= Plotting =========================
Plotting ::Plotting ()  : GuiSystem() , dataPtr(new PlottingPrivate)
{
  this->dataPtr->plottingIface = new ignition::gui::PlottingInterface();

  // Plot Timer
  this->dataPtr->timer = new QTimer();
  auto timeout = this->dataPtr->plottingIface->Timeout();
  this->dataPtr->timer->setInterval(timeout);
  connect(this->dataPtr->timer, SIGNAL(timeout()), this, SLOT(UpdateGui()));
  this->dataPtr->timer->start();

  // PlottingInterface connecting
  connect(this->dataPtr->plottingIface, SIGNAL(ComponentSubscribe
              (uint64_t, uint64_t, std::string, std::string, int)),
          this, SLOT(RegisterChartToComponent
              (uint64_t, uint64_t, std::string, std::string, int)));

  connect(this->dataPtr->plottingIface, SIGNAL(ComponentUnSubscribe
              (uint64_t, uint64_t, std::string, int)),
          this, SLOT(UnRegisterChartToComponent
              (uint64_t, uint64_t, std::string, int)));
}

//////////////////////////////////////////////////
Plotting ::~Plotting()
{
  delete this->dataPtr->plottingIface;
}

//////////////////////////////////////////////////
void Plotting::SetData(std::string _Id, const ignition::math::Vector3d &_vector)
{
  this->dataPtr->components[_Id]->SetAttributeValue("x", _vector.X());
  this->dataPtr->components[_Id]->SetAttributeValue("y", _vector.Y());
  this->dataPtr->components[_Id]->SetAttributeValue("z", _vector.Z());
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
      this->dataPtr->components[Id] = new PlotComponent(_type,
                                                        _entity,
                                                        _typeId);

  this->dataPtr->components[Id]->RegisterChart(_attribute, _chart);
}

//////////////////////////////////////////////////
void Plotting::UnRegisterChartToComponent(uint64_t _entity, uint64_t _typeId,
                                          std::string _attribute, int _chart)
{
  std::string Id = std::to_string(_entity) + "," + std::to_string(_typeId);
  std::cout << "UnRegister " << Id << std::endl;

  if (this->dataPtr->components.count(Id) == 0)
      return;

  this->dataPtr->components[Id]->UnRegisterChart(_attribute, _chart);

  if (!this->dataPtr->components[Id]->HasCharts())
      this->dataPtr->components.erase(Id);
}

//////////////////////////////////////////////////
void Plotting ::Update(const ignition::gazebo::UpdateInfo &,
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
  }
}

//////////////////////////////////////////////////
void Plotting ::UpdateGui()
{
  // Complexty O(registered attributes)
  for (auto component : this->dataPtr->components)
  {
    for (auto attribute : component.second->Data())
    {
      for (auto chart : attribute.second->Charts())
      {
        QString attributeName = QString::fromStdString(
                    component.first + "," + attribute.first);
        float x = this->dataPtr->plottingIface->Time();
        double y = attribute.second->Value();

        emit this->dataPtr->plottingIface->plot(chart, attributeName, x, y);
      }
    }
  }
}

// Register this plugin
IGNITION_ADD_PLUGIN(ignition::gazebo::Plotting ,
                    ignition::gazebo::GuiSystem,
                    ignition::gui::Plugin)
