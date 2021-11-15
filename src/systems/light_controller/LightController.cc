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

#include "LightController.hh"

#include <ignition/msgs/double.pb.h>

#include <string>

#include <ignition/common/Profiler.hh>
#include <ignition/math/PID.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/gazebo/components/Light.hh>
#include <ignition/gazebo/components/LightCmd.hh>

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LightControllerPrivate
{
  /// \brief Light equality comparison function.
  public: std::function<bool(const msgs::Light &, const msgs::Light &)>
    lightEql { [](const msgs::Light &_a, const msgs::Light &_b)
    {
      return
        _a.type() == _b.type() &&
        _a.name() == _b.name() &&
        math::equal(
           _a.diffuse().a(), _b.diffuse().a(), 1e-6f) &&
        math::equal(
          _a.diffuse().r(), _b.diffuse().r(), 1e-6f) &&
        math::equal(
          _a.diffuse().g(), _b.diffuse().g(), 1e-6f) &&
        math::equal(
          _a.diffuse().b(), _b.diffuse().b(), 1e-6f) &&
        math::equal(
          _a.specular().a(), _b.specular().a(), 1e-6f) &&
        math::equal(
          _a.specular().r(), _b.specular().r(), 1e-6f) &&
        math::equal(
          _a.specular().g(), _b.specular().g(), 1e-6f) &&
        math::equal(
          _a.specular().b(), _b.specular().b(), 1e-6f) &&
        math::equal(
          _a.range(), _b.range(), 1e-6f) &&
        math::equal(
          _a.attenuation_linear(),
          _b.attenuation_linear(),
          1e-6f) &&
        math::equal(
          _a.attenuation_constant(),
          _b.attenuation_constant(),
          1e-6f) &&
        math::equal(
          _a.attenuation_quadratic(),
          _b.attenuation_quadratic(),
          1e-6f) &&
        _a.cast_shadows() == _b.cast_shadows() &&
        math::equal(
          _a.intensity(),
          _b.intensity(),
          1e-6f) &&
        math::equal(
          _a.direction().x(), _b.direction().x(), 1e-6) &&
        math::equal(
          _a.direction().y(), _b.direction().y(), 1e-6) &&
        math::equal(
          _a.direction().z(), _b.direction().z(), 1e-6) &&
        math::equal(
          _a.spot_inner_angle(), _b.spot_inner_angle(), 1e-6f) &&
        math::equal(
          _a.spot_outer_angle(), _b.spot_outer_angle(), 1e-6f) &&
        math::equal(_a.spot_falloff(), _b.spot_falloff(), 1e-6f);
    }};

  /// \brief Callback for light subscription
  /// \param[in] _msg A new message describing a light that needs
  /// to be followed
  public: void LightCallback(const ignition::msgs::Light &_msg);

  /// \brief Ignition communication node.
  public: transport::Node node;

  /// \brief Light Entity
  public: Entity lightEntity;

  /// \brief Light data
  public: ignition::msgs::Light lightData;

  /// \brief Update light data
  public: bool isUpdateRequired;
};

//////////////////////////////////////////////////
LightController::LightController()
  : dataPtr(std::make_unique<LightControllerPrivate>())
{
}

//////////////////////////////////////////////////
void LightController::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  ignmsg << "[LightController] Configure light controller\n";

  auto lightName = _sdf->Get<std::string>("light_name");

  // Can't use components::Light in EntityByComponents, see
  // https://github.com/ignitionrobotics/ign-gazebo/issues/376
  auto entity = _ecm.EntityByComponents(components::Name(lightName));
  if (!_ecm.Component<components::Light>(entity))
  {
    ignerr << "Entity named [" << lightName << "] is not a Light" << std::endl;
    return;
  }

  this->dataPtr->lightEntity = entity;

  auto component =
    _ecm.Component<components::Light>(this->dataPtr->lightEntity);

  sdf::Light lightSDF = component->Data();
  this->dataPtr->lightData = convert<msgs::Light>(lightSDF);

  // Subscribe to light commands
  auto lightTopic = _sdf->Get<std::string>("topic");
  if (lightTopic.empty())
  {
    // If not specified, use the default topic based on light name
    lightTopic = "/light/" + lightName + "/light_config";
  }
  // Make sure the topic is valid
  const auto validLightTopic = transport::TopicUtils::AsValidTopic(
      lightTopic);
  if (validLightTopic.empty())
  {
    ignerr << "[LightController] Cannot subscribe to invalid topic ["
           << lightTopic << "].\n";
    return;
  }
  // Subscribe
  ignmsg << "[LightController] Subscribing to light"
            " commands on topic [" << validLightTopic << "].\n";
  this->dataPtr->node.Subscribe(
      validLightTopic,
      &LightControllerPrivate::LightCallback,
      this->dataPtr.get());
}

//////////////////////////////////////////////////
void LightController::PreUpdate(const ignition::gazebo::UpdateInfo &_info,
    ignition::gazebo::EntityComponentManager &_ecm)
{
  if (this->dataPtr->isUpdateRequired)
  {
    auto lightCmdComp =
      _ecm.Component<components::LightCmd>(this->dataPtr->lightEntity);
    if (!lightCmdComp)
    {
      _ecm.CreateComponent(
          this->dataPtr->lightEntity, components::LightCmd(
          this->dataPtr->lightData));
    }
    else
    {
      auto state = lightCmdComp->SetData(this->dataPtr->lightData,
          this->dataPtr->lightEql) ? ComponentState::OneTimeChange :
          ComponentState::NoChange;
      _ecm.SetChanged(this->dataPtr->lightEntity, components::LightCmd::typeId,
          state);
    }

    this->dataPtr->isUpdateRequired = false;
  }
}

//////////////////////////////////////////////////
void LightControllerPrivate::LightCallback(
    const ignition::msgs::Light &_msg)
{
  if (_msg.name() == "pose")
  {
    this->lightData.mutable_pose()->mutable_position()->set_x(
        _msg.pose().position().x());

    this->lightData.mutable_pose()->mutable_position()->set_y(
        _msg.pose().position().y());

    this->lightData.mutable_pose()->mutable_position()->set_z(
        _msg.pose().position().z());

    this->lightData.mutable_pose()->mutable_orientation()->set_x(
        _msg.pose().orientation().x());

    this->lightData.mutable_pose()->mutable_orientation()->set_y(
        _msg.pose().orientation().y());

    this->lightData.mutable_pose()->mutable_orientation()->set_z(
        _msg.pose().orientation().z());

    this->lightData.mutable_pose()->mutable_orientation()->set_w(
        _msg.pose().orientation().w());
  }
  else if (_msg.name() == "diffuse")
  {
    this->lightData.mutable_diffuse()->set_r(_msg.diffuse().r());
    this->lightData.mutable_diffuse()->set_g(_msg.diffuse().g());
    this->lightData.mutable_diffuse()->set_b(_msg.diffuse().b());
    this->lightData.mutable_diffuse()->set_a(_msg.diffuse().a());
  }
  else if (_msg.name() == "specular")
  {
    this->lightData.mutable_specular()->set_r(_msg.specular().r());
    this->lightData.mutable_specular()->set_g(_msg.specular().g());
    this->lightData.mutable_specular()->set_b(_msg.specular().b());
    this->lightData.mutable_specular()->set_a(_msg.specular().a());
  }
  else if (_msg.name() == "attenuation_constant")
  {
    this->lightData.set_attenuation_constant(_msg.attenuation_constant());
  }
  else if (_msg.name() == "attenuation_linear")
  {
    this->lightData.set_attenuation_linear(_msg.attenuation_linear());
  }
  else if (_msg.name() == "attenuation_quadratic")
  {
    this->lightData.set_attenuation_quadratic(_msg.attenuation_quadratic());
  }
  else if (_msg.name() == "direction")
  {
    this->lightData.mutable_direction()->set_x(_msg.direction().x());
    this->lightData.mutable_direction()->set_y(_msg.direction().y());
    this->lightData.mutable_direction()->set_z(_msg.direction().z());
  }
  else if (_msg.name() == "range")
  {
    this->lightData.set_range(_msg.range());
  }
  else if (_msg.name() == "spot_inner_angle")
  {
    this->lightData.set_spot_inner_angle(_msg.spot_inner_angle());
  }
  else if (_msg.name() == "spot_outer_angle")
  {
    this->lightData.set_spot_outer_angle(_msg.spot_outer_angle());
  }
  else if (_msg.name() == "spot_falloff")
  {
    this->lightData.set_spot_falloff(_msg.spot_falloff());
  }
  else if (_msg.name() == "intensity")
  {
    this->lightData.set_intensity(_msg.intensity());
  }
  else
  {
    this->lightData = _msg;
  }

  this->isUpdateRequired = true;
}

IGNITION_ADD_PLUGIN(LightController,
                    ignition::gazebo::System,
                    LightController::ISystemConfigure,
                    LightController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LightController,
                          "ignition::gazebo::systems::LightController")
