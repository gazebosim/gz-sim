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

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::LightControllerPrivate
{
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
}

//////////////////////////////////////////////////
void LightControllerPrivate::LightCallback(
    const ignition::msgs::Light &_msg)
{
  std::cout << "Hello World!" << std::endl;
}

IGNITION_ADD_PLUGIN(LightController,
                    ignition::gazebo::System,
                    LightController::ISystemConfigure,
                    LightController::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(LightController,
                          "ignition::gazebo::systems::LightController")
