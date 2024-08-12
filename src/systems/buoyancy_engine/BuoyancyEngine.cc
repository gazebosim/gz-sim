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
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>

#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "BuoyancyEngine.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::BuoyancyEnginePrivateData
{
  /// \brief Callback for incoming commands
  /// \param[in] _volumeSetPoint - Gazebo message containing the desired
  /// volume (in m^3) to fill/drain bladder to.
  public: void OnCmdBuoyancyEngine(
    const gz::msgs::Double &_volumeSetPoint);

  /// \brief Current volume of bladder in m^3
  public: double bladderVolume = 3e-5;

  /// \brief Maximum inflation rate in m^3*s^-1
  public: double maxInflationRate = 3e-6;

  /// \brief Set-point for volume, in m^3
  public: double volumeSetPoint = 0.000030;

  /// \brief Minimum volume of bladder in m^3
  public: double minVolume = 0.000030;

  /// \brief Maximum volume of bladder in m^3
  public: double maxVolume = 0.000990;

  /// \brief The link which the bladder is attached to
  public: gz::sim::Entity linkEntity{kNullEntity};

  /// \brief The world entity
  public: Entity world{kNullEntity};

  /// \brief The fluid density in kg*m^-3
  public: double fluidDensity = 1000;

  /// \brief The neutral volume in m^3
  public: double neutralVolume = 0.0003;

  /// \brief Surface location
  public: std::optional<double> surface = std::nullopt;

  /// \brief Trasport node for control
  public: gz::transport::Node node;

  /// \brief Publishes bladder status
  public: gz::transport::Node::Publisher statusPub;

  /// \brief mutex for protecting bladder volume and set point.
  public: std::mutex mtx;

  /// \brief  Get fluid density based on the link origin's current position.
  /// \param[in] _ecm - The ecm in question.
  public: double CurrentFluidDensity(
    const EntityComponentManager &_ecm) const;
};

//////////////////////////////////////////////////
double BuoyancyEnginePrivateData::CurrentFluidDensity(
  const EntityComponentManager &_ecm) const
{
  if (!this->surface.has_value())
    return fluidDensity;

  auto pose = sim::worldPose(this->linkEntity, _ecm);

  if (pose.Pos().Z() < this->surface.value())
  {
    return fluidDensity;
  }
  return 0;
}

//////////////////////////////////////////////////
void BuoyancyEnginePrivateData::OnCmdBuoyancyEngine(
  const gz::msgs::Double &_volumeSetpoint)
{
  auto volume = std::max(this->minVolume, _volumeSetpoint.data());
  volume = std::min(volume, this->maxVolume);

  std::lock_guard lock(this->mtx);
  this->volumeSetPoint = volume;
}

//////////////////////////////////////////////////
BuoyancyEnginePlugin::BuoyancyEnginePlugin()
  : dataPtr(std::make_unique<BuoyancyEnginePrivateData>())
{
}

//////////////////////////////////////////////////
void BuoyancyEnginePlugin::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &/*_eventMgr*/)
{
  auto model = gz::sim::Model(_entity);
  if (!_sdf->HasElement("link_name"))
  {
    gzerr << "Buoyancy Engine must be attached to some link."  << std::endl;
    return;
  }

  this->dataPtr->linkEntity =
    model.LinkByName(_ecm, _sdf->Get<std::string>("link_name"));
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    gzerr << "Link [" << _sdf->Get<std::string>("link_name")
      << "] was not found in model [" << model.Name(_ecm) << "]" << std::endl;
    return;
  }

  if (_sdf->HasElement("min_volume"))
  {
    this->dataPtr->minVolume = _sdf->Get<double>("min_volume");
  }

  if (_sdf->HasElement("max_volume"))
  {
    this->dataPtr->maxVolume = _sdf->Get<double>("max_volume");
  }

  if (_sdf->HasElement("fluid_density"))
  {
    this->dataPtr->fluidDensity = _sdf->Get<double>("fluid_density");
  }

  this->dataPtr->bladderVolume = this->dataPtr->minVolume;
  if (_sdf->HasElement("default_volume"))
  {
    this->dataPtr->bladderVolume = _sdf->Get<double>("default_volume");
    this->dataPtr->volumeSetPoint = this->dataPtr->bladderVolume;
  }

  if (_sdf->HasElement("neutral_volume"))
  {
    this->dataPtr->neutralVolume = _sdf->Get<double>("neutral_volume");
  }

  if (_sdf->HasElement("max_inflation_rate"))
  {
    this->dataPtr->maxInflationRate = _sdf->Get<double>("max_inflation_rate");
  }

  if (_sdf->HasElement("surface"))
  {
    this->dataPtr->surface = _sdf->Get<double>("surface");
  }

  this->dataPtr->world = _ecm.EntityByComponents(components::World());
  if (this->dataPtr->world == kNullEntity)
  {
    gzerr << "World entity not found" <<std::endl;
    return;
  }

  std::string cmdTopic = "/buoyancy_engine/";
  std::string statusTopic = "/buoyancy_engine/current_volume";
  if (_sdf->HasElement("namespace"))
  {
    cmdTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + _sdf->Get<std::string>("namespace") + "/buoyancy_engine/");
    statusTopic = gz::transport::TopicUtils::AsValidTopic(
      "/model/" + _sdf->Get<std::string>("namespace")
      + "/buoyancy_engine/current_volume");
  }

  if (!this->dataPtr->node.Subscribe(cmdTopic,
    &BuoyancyEnginePrivateData::OnCmdBuoyancyEngine, this->dataPtr.get()))
  {
    gzerr << "Failed to subscribe to [" << cmdTopic << "]" << std::endl;
  }

  this->dataPtr->statusPub =
    this->dataPtr->node.Advertise<gz::msgs::Double>(statusTopic);

  gzdbg << "Listening to commands on [" << cmdTopic
         << "], publishing status on [" << statusTopic << "]" <<std::endl;
}

//////////////////////////////////////////////////
void BuoyancyEnginePlugin::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  typedef std::chrono::duration<float, std::ratio<1L, 1L>> DurationInSecs;
  auto dt = std::chrono::duration_cast<DurationInSecs>(_info.dt).count();

  gz::msgs::Double msg;

  const components::Gravity *gravity = _ecm.Component<components::Gravity>(
    this->dataPtr->world);
  if (!gravity)
  {
    gzerr << "World has no gravity component" << std::endl;
    return;
  }

  gz::sim::Link link(this->dataPtr->linkEntity);
  math::Vector3d zForce;
  {
    std::lock_guard lock(this->dataPtr->mtx);
    // Adjust the bladder volume using the pump. Assume ability to pump at
    // max flow rate
    if (this->dataPtr->bladderVolume < this->dataPtr->volumeSetPoint)
    {
      this->dataPtr->bladderVolume +=
        std::min(
          dt * this->dataPtr->maxInflationRate,
          this->dataPtr->volumeSetPoint - this->dataPtr->bladderVolume
        );
    }
    else if (this->dataPtr->bladderVolume > this->dataPtr->volumeSetPoint)
    {
      this->dataPtr->bladderVolume -=
        std::min(
          dt * this->dataPtr->maxInflationRate,
          this->dataPtr->bladderVolume - this->dataPtr->volumeSetPoint
        );
    }

    /// Populate status message
    msg.set_data(this->dataPtr->bladderVolume);
    this->dataPtr->statusPub.Publish(msg);

    // Get the fluid density of the current layer
    auto currentFluidDensity = this->dataPtr->CurrentFluidDensity(_ecm);

    // Simply use Archimede's principle to apply a force at the desired link
    // position. We take off the neutral buoyancy element in order to simulate
    // the mass of the oil in the bladder.
    zForce = - gravity->Data() *
      ( currentFluidDensity * this->dataPtr->bladderVolume
      - this->dataPtr->fluidDensity * this->dataPtr->neutralVolume);
  }
  link.AddWorldWrench(_ecm, zForce, {0, 0, 0});
}

GZ_ADD_PLUGIN(
  BuoyancyEnginePlugin,
  gz::sim::System,
  BuoyancyEnginePlugin::ISystemConfigure,
  BuoyancyEnginePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(BuoyancyEnginePlugin,
                          "gz::sim::systems::BuoyancyEngine")
