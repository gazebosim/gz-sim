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

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable: 4251)
#endif

#include <google/protobuf/message.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <gz/msgs/double.pb.h>

#include <string>

#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>

#include <gz/transport/Node.hh>

#include "KineticEnergyMonitor.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Private data class
class gz::sim::systems::KineticEnergyMonitorPrivate
{
  /// \brief Link of the model.
  public: Entity linkEntity;

  /// \brief Name of the model this plugin is attached to.
  public: std::string modelName;

  /// \brief Kinetic energy during the previous step.
  public: double prevKineticEnergy {0.0};

  /// \brief Kinetic energy threshold.
  public: double keThreshold {7.0};

  /// \brief Gazebo communication publisher.
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
    gzerr << "KineticEnergyMonitor should be attached to a model "
      << "entity. Failed to initialize." << std::endl;
    return;
  }

  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  auto sdfClone = _sdf->Clone();
  std::string linkName;
  if (sdfClone->HasElement("link_name"))
  {
    linkName = sdfClone->Get<std::string>("link_name");
  }

  if (linkName.empty())
  {
    gzerr << "found an empty <link_name> parameter. Failed to initialize."
      << std::endl;
    return;
  }

  // Get the link entity
  this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);

  if (this->dataPtr->linkEntity == kNullEntity)
  {
    gzerr << "Link " << linkName
      << " could not be found. Failed to initialize.\n";
    return;
  }

  this->dataPtr->keThreshold = sdfClone->Get<double>(
      "kinetic_energy_threshold", 7.0).first;

  std::string defaultTopic{"/model/" + this->dataPtr->modelName +
    "/kinetic_energy"};
  std::string topic = sdfClone->Get<std::string>("topic", defaultTopic).first;

  gzmsg << "KineticEnergyMonitor publishing messages on "
    << "[" << topic << "]" << std::endl;

  transport::Node node;
  this->dataPtr->pub = node.Advertise<msgs::Double>(topic);

  Link link(this->dataPtr->linkEntity);
  link.EnableVelocityChecks(_ecm, true);

  // Create a default inertia in case the link doesn't have it
  enableComponent<components::Inertial>(_ecm, this->dataPtr->linkEntity, true);
}

//////////////////////////////////////////////////
void KineticEnergyMonitor::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("KineticEnergyMonitor::PostUpdate");
  // Nothing left to do if paused or the publisher wasn't created.
  if (_info.paused || !this->dataPtr->pub)
    return;

  if (this->dataPtr->linkEntity != kNullEntity)
  {
    Link link(this->dataPtr->linkEntity);
    if (std::nullopt != link.WorldKineticEnergy(_ecm))
    {
      double currKineticEnergy = *link.WorldKineticEnergy(_ecm);

      // We only care about positive values of this (the links looses energy)
      double deltaKE = this->dataPtr->prevKineticEnergy - currKineticEnergy;
      this->dataPtr->prevKineticEnergy = currKineticEnergy;

      if (deltaKE > this->dataPtr->keThreshold)
      {
        gzmsg << this->dataPtr->modelName
          << " Change in kinetic energy above threshold - deltaKE: "
          << deltaKE << std::endl;
        msgs::Double msg;
        msg.set_data(deltaKE);
        this->dataPtr->pub.Publish(msg);
      }
    }
  }
}

GZ_ADD_PLUGIN(KineticEnergyMonitor,
                    gz::sim::System,
                    KineticEnergyMonitor::ISystemConfigure,
                    KineticEnergyMonitor::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(KineticEnergyMonitor,
  "gz::sim::systems::KineticEnergyMonitor")
