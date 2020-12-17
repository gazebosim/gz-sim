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

#include <google/protobuf/message.h>
#include <ignition/msgs/double.pb.h>

#include <ignition/gazebo/components/AngularVelocity.hh>
#include <ignition/gazebo/components/Link.hh>
#include <ignition/gazebo/components/LinearVelocity.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Inertial.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>

#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include "KineticEnergyMonitor.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

/// \brief Private data class
class ignition::gazebo::systems::KineticEnergyMonitorPrivate
{
  /// \brief Base link of the model.
  public: Entity baseLinkEntity;

  /// \brief Name of the model this plugin is attached to.
  public: std::string modelName;

  /// \brief Kinetic energy during the previous step.
  public: double prevKineticEnergy {0.0};

  /// \brief Kinetic energy threshold.
  public: double keThreshold {7.0};

  /// \brief Ignition communication publisher.
  public: transport::Node::Publisher pub;

  /// \brief The model this plugin is attached to.
  public: Model model;
};

//////////////////////////////////////////////////
KineticEnergyMonitor::KineticEnergyMonitor() : System(),
    dataPtr(std::make_unique<KineticEnergyMonitorPrivate>())
{
}

//////////////////////////////////////////////////
KineticEnergyMonitor::~KineticEnergyMonitor() = default;

//////////////////////////////////////////////////
void KineticEnergyMonitor::Configure(const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    ignerr << "DamagePlugin should be attached to a model "
      << "entity. Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  auto sdfClone = _sdf->Clone();
  std::string baseLinkName;
  if (sdfClone->HasElement("base_link_name"))
  {
    baseLinkName = sdfClone->Get<std::string>("base_link_name");
  }

  if (baseLinkName.empty())
  {
    ignerr << "found an empty baseLinkName parameter. Failed to initialize."
      << std::endl;
    return;
  }

  // Get the link entity
  this->dataPtr->baseLinkEntity = this->dataPtr->model.LinkByName(_ecm,
      baseLinkName);

  if (this->dataPtr->baseLinkEntity == kNullEntity)
  {
    ignerr << "Link " << baseLinkName
      << " could not be found. Failed to initialize.\n";
    return;
  }

  this->dataPtr->keThreshold = sdfClone->Get<double>(
      "kinetic_energy_threshold", 7.0).first;

  std::string defaultTopic{"/model/" + this->dataPtr->modelName + "/damage"};
  std::string topic = sdfClone->Get<std::string>("topic", defaultTopic).first;

  ignmsg << "KineticEnergyMonitor publishing messages on "
    << "[" << topic << "]" << std::endl;

  transport::Node node;
  this->dataPtr->pub = node.Advertise<msgs::Double>(topic);

  if (!_ecm.Component<components::WorldPose>(this->dataPtr->baseLinkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->baseLinkEntity,
        components::WorldPose());
  }

  if (!_ecm.Component<components::Inertial>(this->dataPtr->baseLinkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->baseLinkEntity, components::Inertial());
  }

  // Create a world linear velocity component if one is not present.
  if (!_ecm.Component<components::WorldLinearVelocity>(
        this->dataPtr->baseLinkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->baseLinkEntity,
        components::WorldLinearVelocity());
  }

  // Create an angular velocity component if one is not present.
  if (!_ecm.Component<components::AngularVelocity>(
        this->dataPtr->baseLinkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->baseLinkEntity,
        components::AngularVelocity());
  }

  // Create an angular velocity component if one is not present.
  if (!_ecm.Component<components::WorldAngularVelocity>(
        this->dataPtr->baseLinkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->baseLinkEntity,
        components::WorldAngularVelocity());
  }
}

//////////////////////////////////////////////////
void KineticEnergyMonitor::PostUpdate(const UpdateInfo &/*_info*/,
    const EntityComponentManager &_ecm)
{
  if (this->dataPtr->baseLinkEntity != kNullEntity)
  {
    Link link(this->dataPtr->baseLinkEntity);
    if (std::nullopt != link.WorldKineticEnergy(_ecm))
    {
      double currKineticEnergy = *link.WorldKineticEnergy(_ecm);

      // We only care about positive values of this (the links looses energy)
      double deltaKE = this->dataPtr->prevKineticEnergy - currKineticEnergy;
      this->dataPtr->prevKineticEnergy = currKineticEnergy;

      if (deltaKE > this->dataPtr->keThreshold)
      {
        ignmsg << this->dataPtr->modelName << " Crashed - deltaKE: " << deltaKE
          << std::endl;
        msgs::Double msg;
        msg.set_data(deltaKE);
        this->dataPtr->pub.Publish(msg);
      }
    }
  }
}

IGNITION_ADD_PLUGIN(KineticEnergyMonitor, System,
  KineticEnergyMonitor::ISystemConfigure,
  KineticEnergyMonitor::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(KineticEnergyMonitor,
  "ignition::gazebo::systems::KineticEnergyMonitor")
