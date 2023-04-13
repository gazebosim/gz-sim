/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <gz/msgs/light.pb.h>

#include <sdf/Light.hh>

#include "gz/sim/components/Light.hh"
#include "gz/sim/components/LightType.hh"
#include "gz/sim/components/LightCmd.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"

#include "gz/sim/Light.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;


class gz::sim::Light::Implementation
{
  /// \brief Id of light entity.
  public: sim::Entity id{kNullEntity};
};

//////////////////////////////////////////////////
Light::Light(sim::Entity _entity)
  : dataPtr(utils::MakeImpl<Implementation>())
{
  this->dataPtr->id = _entity;
}

//////////////////////////////////////////////////
sim::Entity Light::Entity() const
{
  return this->dataPtr->id;
}

//////////////////////////////////////////////////
void Light::ResetEntity(sim::Entity _newEntity)
{
  this->dataPtr->id = _newEntity;
}

//////////////////////////////////////////////////
bool Light::Valid(const EntityComponentManager &_ecm) const
{
  return nullptr != _ecm.Component<components::Light>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::string> Light::Name(const EntityComponentManager &_ecm) const
{
  return _ecm.ComponentData<components::Name>(this->dataPtr->id);
}

//////////////////////////////////////////////////
std::optional<std::string> Light::Type(
    const EntityComponentManager &_ecm) const
{
  auto lightType = _ecm.Component<components::LightType>(this->dataPtr->id);

  if (!lightType)
    return std::nullopt;

  return std::optional<std::string>(lightType->Data());
}

//////////////////////////////////////////////////
std::optional<math::Pose3d> Light::Pose(
    const EntityComponentManager &_ecm) const
{
  auto pose = _ecm.Component<components::Pose>(this->dataPtr->id);

  if (!pose)
    return std::nullopt;

  return std::optional<math::Pose3d>(pose->Data());
}

//////////////////////////////////////////////////
std::optional<bool> Light::CastShadows(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<bool>(light->Data().CastShadows());
}

//////////////////////////////////////////////////
std::optional<double> Light::Intensity(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<double>(light->Data().Intensity());
}

//////////////////////////////////////////////////
std::optional<math::Vector3d> Light::Direction(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<math::Vector3d>(light->Data().Direction());
}

//////////////////////////////////////////////////
std::optional<math::Color> Light::DiffuseColor(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<math::Color>(light->Data().Diffuse());
}

//////////////////////////////////////////////////
std::optional<math::Color> Light::SpecularColor(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<math::Color>(light->Data().Specular());
}

//////////////////////////////////////////////////
std::optional<double> Light::AttenuationRange(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<double>(light->Data().AttenuationRange());
}

//////////////////////////////////////////////////
std::optional<double> Light::AttenuationConstant(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<double>(light->Data().ConstantAttenuationFactor());
}

//////////////////////////////////////////////////
std::optional<double> Light::AttenuationLinear(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<double>(light->Data().LinearAttenuationFactor());
}

//////////////////////////////////////////////////
std::optional<double> Light::AttenuationQuadratic(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<double>(light->Data().QuadraticAttenuationFactor());
}

//////////////////////////////////////////////////
std::optional<math::Angle> Light::SpotInnerAngle(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<math::Angle>(light->Data().SpotInnerAngle());
}

//////////////////////////////////////////////////
std::optional<math::Angle> Light::SpotOuterAngle(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<math::Angle>(light->Data().SpotOuterAngle());
}

//////////////////////////////////////////////////
std::optional<double> Light::SpotFalloff(
    const EntityComponentManager &_ecm) const
{
  auto light = _ecm.Component<components::Light>(this->dataPtr->id);

  if (!light)
    return std::nullopt;

  return std::optional<double>(light->Data().SpotFalloff());
}

//////////////////////////////////////////////////
void Light::SetPose(EntityComponentManager &_ecm,
    const math::Pose3d &_pose)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  msgs::Set(lightMsg.mutable_pose(), _pose);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetCastShadows(EntityComponentManager &_ecm,
   bool _castShadows)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  lightMsg.set_cast_shadows(_castShadows);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetIntensity(EntityComponentManager &_ecm,
   double _intensity)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  lightMsg.set_intensity(_intensity);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetDirection(EntityComponentManager &_ecm,
   const math::Vector3d &_dir)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  msgs::Set(lightMsg.mutable_direction(), _dir);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetDiffuseColor(EntityComponentManager &_ecm,
   const math::Color &_color)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  msgs::Set(lightMsg.mutable_diffuse(), _color);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetSpecularColor(EntityComponentManager &_ecm,
   const math::Color &_color)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  msgs::Set(lightMsg.mutable_specular(), _color);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetAttenuationRange(EntityComponentManager &_ecm,
   double _range)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  lightMsg.set_range(_range);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetAttenuationConstant(EntityComponentManager &_ecm,
   double _value)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  lightMsg.set_attenuation_constant(_value);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}
//////////////////////////////////////////////////
void Light::SetAttenuationLinear(EntityComponentManager &_ecm,
   double _value)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  lightMsg.set_attenuation_linear(_value);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetAttenuationQuadratic(EntityComponentManager &_ecm,
   double _value)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  lightMsg.set_attenuation_quadratic(_value);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetSpotInnerAngle(EntityComponentManager &_ecm,
   const math::Angle &_angle)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  lightMsg.set_spot_inner_angle(_angle.Radian());
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetSpotOuterAngle(EntityComponentManager &_ecm,
   const math::Angle &_angle)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  lightMsg.set_spot_outer_angle(_angle.Radian());
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
void Light::SetSpotFalloff(EntityComponentManager &_ecm,
   double _falloff)
{
  auto lightCmd =
    _ecm.Component<components::LightCmd>(this->dataPtr->id);

  msgs::Light lightMsg;
  lightMsg.set_spot_falloff(_falloff);
  if (!lightCmd)
  {
    _ecm.CreateComponent(
        this->dataPtr->id,
        components::LightCmd(lightMsg));
  }
  else
  {
    lightCmd->Data() = lightMsg;
  }
}

//////////////////////////////////////////////////
std::optional<Entity> Light::Parent(const EntityComponentManager &_ecm) const
{
  auto parent = _ecm.Component<components::ParentEntity>(this->dataPtr->id);

  if (!parent)
    return std::nullopt;

  return std::optional<sim::Entity>(parent->Data());
}
