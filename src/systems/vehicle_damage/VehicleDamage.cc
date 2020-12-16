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

#include "VehicleDamage.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::VehicleDamagePrivate
{
  private: Entity baseLinkEntity;
  private: std::string modelName;
  private: double prevKineticEnergy {0.0};
  private: double keThreshold;
  /// \brief Ignition communication publisher.
  private: transport::Node::Publisher pub;

  public: Model model;

  public: void Load(EntityComponentManager &_ecm,
                  const std::shared_ptr<const sdf::Element> &_sdf)
  {
    if (!this->model.Valid(_ecm))
    {
      ignerr << "DamagePlugin should be attached to a model "
            << "entity. Failed to initialize." << std::endl;
      return;
    }

    this->modelName = this->model.Name(_ecm);

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
    this->baseLinkEntity = this->model.LinkByName(_ecm, baseLinkName);

    if (this->baseLinkEntity == kNullEntity)
    {
      ignerr << "Link " << baseLinkName
            << " could not be found. Failed to initialize.\n";
      return;
    }

    this->keThreshold = sdfClone->Get<double>(
                                        "kinetic_energy_threshold", 7.0).first;

    std::string defaultTopic{"/model/" + this->modelName + "/damage"};
    std::string topic = sdfClone->Get<std::string>("topic", defaultTopic).first;

    ignmsg << "PerformerDetector publishing messages on "
           << "[" << topic << "]" << std::endl;

    transport::Node node;
    this->pub = node.Advertise<msgs::Double>(topic);

    if (!_ecm.Component<components::WorldPose>(this->baseLinkEntity))
    {
      _ecm.CreateComponent(this->baseLinkEntity, components::WorldPose());
    }

    if (!_ecm.Component<components::Inertial>(this->baseLinkEntity))
    {
      _ecm.CreateComponent(this->baseLinkEntity, components::Inertial());
    }

    // Create a world linear velocity component if one is not present.
    if (!_ecm.Component<components::WorldLinearVelocity>(this->baseLinkEntity))
    {
      _ecm.CreateComponent(this->baseLinkEntity,
        components::WorldLinearVelocity());
    }

    // Create an angular velocity component if one is not present.
    if (!_ecm.Component<components::AngularVelocity>(this->baseLinkEntity))
    {
      _ecm.CreateComponent(this->baseLinkEntity, components::AngularVelocity());
    }

    // Create an angular velocity component if one is not present.
    if (!_ecm.Component<components::WorldAngularVelocity>(this->baseLinkEntity))
    {
      _ecm.CreateComponent(this->baseLinkEntity,
        components::WorldAngularVelocity());
    }
  }

  public: void Update(const UpdateInfo &/*_info*/,
      const EntityComponentManager &_ecm)
  {
    if (this->baseLinkEntity != kNullEntity)
    {
     Link link(this->baseLinkEntity);
     if (std::nullopt != link.WorldKineticEnergy(_ecm))
     {
      double currKineticEnergy = *link.WorldKineticEnergy(_ecm);

      // We only care about positive values of this (the links looses energy)
      double deltaKE = this->prevKineticEnergy - currKineticEnergy;
      this->prevKineticEnergy = currKineticEnergy;

      if (deltaKE > this->keThreshold )
      {
        ignmsg << this->modelName << " Crashed - deltaKE: " << deltaKE
          << std::endl;
        msgs::Double msg;
        msg.set_data(deltaKE);
        this->pub.Publish(msg);
      }
     }
    }
  }
};

//////////////////////////////////////////////////
VehicleDamage::VehicleDamage() : System(),
    dataPtr(std::make_unique<VehicleDamagePrivate>())
{
}

//////////////////////////////////////////////////
VehicleDamage::~VehicleDamage() = default;

//////////////////////////////////////////////////
void VehicleDamage::Configure(const Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        EntityComponentManager &_ecm,
        EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);
  this->dataPtr->Load(_ecm, _sdf);
}

//////////////////////////////////////////////////
void VehicleDamage::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  this->dataPtr->Update(_info, _ecm);
}

IGNITION_ADD_PLUGIN(VehicleDamage, System,
  VehicleDamage::ISystemConfigure,
  VehicleDamage::ISystemPostUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(VehicleDamage,
  "ignition::gazebo::systems::VehicleDamage")
