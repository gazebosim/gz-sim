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

#include <gz/msgs/serialized_map.pb.h>
#include "gz/sim/private_msgs/peer_control.pb.h"
#include "gz/sim/private_msgs/simulation_step.pb.h"

#include <algorithm>
#include <string>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/common/Profiler.hh>

#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/PerformerAffinity.hh"
#include "gz/sim/Conversions.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Events.hh"

#include "NetworkManagerPrivate.hh"
#include "NetworkManagerSecondary.hh"
#include "PeerTracker.hh"

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
NetworkManagerSecondary::NetworkManagerSecondary(
    const std::function<void(const UpdateInfo &_info)> &_stepFunction,
    EntityComponentManager &_ecm,
    EventManager *_eventMgr,
    const NetworkConfig &_config, const NodeOptions &_options):
  NetworkManager(_stepFunction, _ecm, _eventMgr, _config, _options),
  node(_options)
{
  std::string controlService{this->Namespace() + "/control"};
  if (!this->node.Advertise(controlService, &NetworkManagerSecondary::OnControl,
      this))
  {
    gzerr << "Error advertising PeerControl service [" << controlService
           << "]" << std::endl;
  }
  else
  {
    gzdbg << "Advertised PeerControl service on [" << controlService << "]"
      << std::endl;
  }

  this->node.Subscribe("step", &NetworkManagerSecondary::OnStep, this);

  this->stepAckPub = this->node.Advertise<msgs::SerializedStateMap>("step_ack");
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::Ready() const
{
  // The detected number of peers in the "Primary" role must be 1
  auto primaries = this->dataPtr->tracker->NumPrimary();
  return (primaries == 1);
}

//////////////////////////////////////////////////
void NetworkManagerSecondary::Handshake()
{
  while (!this->enableSim && !this->dataPtr->stopReceived)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

//////////////////////////////////////////////////
std::string NetworkManagerSecondary::Namespace() const
{
  return this->dataPtr->peerInfo.id.substr(0, 8);
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::OnControl(const private_msgs::PeerControl &_req,
                                        private_msgs::PeerControl& _resp)
{
  this->enableSim = _req.enable_sim();
  _resp.set_enable_sim(this->enableSim);

  gzdbg << "Received msg PeerControl: enable_sim: "
        << this->enableSim << std::endl;

  return true;
}

/////////////////////////////////////////////////
void NetworkManagerSecondary::OnStep(
    const private_msgs::SimulationStep &_msg)
{
  GZ_PROFILE("NetworkManagerSecondary::OnStep");

  // Do not print debug message if paused.
  if (!_msg.stats().paused())
  {
    gzdbg << "NetworkManagerSecondary::OnStep [" << this->Namespace() << "]"
          << std::endl;
    if (_msg.affinity_size() > 0)
    {
      gzdbg << "Secondary: SimulationStep:\n"
            << _msg.DebugString() << std::endl;
    }
  }

  // Throttle the number of step messages going to the debug output.
  if (!_msg.stats().paused() && _msg.stats().iterations() % 1000 == 0)
  {
    gzdbg << "Network iterations: " << _msg.stats().iterations()
          << std::endl;
  }

  // Update affinities
  for (int i = 0; i < _msg.affinity_size(); ++i)
  {
    const auto &affinityMsg = _msg.affinity(i);
    const auto &perfEntity = affinityMsg.entity().id();

    if (affinityMsg.secondary_prefix() == this->Namespace())
    {
      this->performers.insert(perfEntity);

      // Set performer affinity to this secondary
      this->dataPtr->ecm->RemoveComponent<components::PerformerAffinity>(
          perfEntity);
      auto newAffinity = components::PerformerAffinity(this->Namespace());
      this->dataPtr->ecm->CreateComponent(perfEntity, newAffinity);

      gzmsg << "Secondary [" << this->Namespace()
             << "] assigned affinity to performer ["
             << perfEntity << "]."
             << std::endl;
    }
    else
    {
      // If performer assigned to this secondary is also assigned to another
      // secondary, remove it from performers and the parent model from the ECM.
      if (this->performers.find(perfEntity) != this->performers.end())
      {
        gzmsg << "Secondary [" << this->Namespace()
               << "] unassigned affinity to performer ["
               << perfEntity << "]."
               << std::endl;
        this->performers.erase(perfEntity);

        // Remove performer affinity from this this secondary
        this->dataPtr->ecm->RemoveComponent<components::PerformerAffinity>(
            perfEntity);

        auto parent =
            this->dataPtr->ecm->Component<components::ParentEntity>(perfEntity);
        if (parent != nullptr)
        {
          this->dataPtr->ecm->RequestRemoveEntity(parent->Data());
          gzmsg << "Secondary [" << this->Namespace()
                << "] request remove model entity ["
                << parent->Data() << "]."
                << std::endl;
        }
        else
        {
          gzerr << "Secondary [" << this->Namespace()
                << "] should remove parent for performer ["
                << perfEntity << "]."
                << std::endl;
        }
      }
    }
  }

  // Update info
  auto info = convert<UpdateInfo>(_msg.stats());

  //! @todo must set the secondaryNamespace here...
  info.secondaryNamespace = this->Namespace();

  // Step runner
  this->dataPtr->stepFunction(info);

  // Update state with all the performer's entities
  std::unordered_set<Entity> entities;
  if (!_msg.stats().paused())
  {
    gzdbg << "Secondary: performer: size: "
          << this->performers.size() << std::endl;
  }
  for (const auto &perfEntity : this->performers)
  {
    // Performer model
    auto parent =
        this->dataPtr->ecm->Component<components::ParentEntity>(perfEntity);
    if (parent == nullptr)
    {
      gzerr << "Secondary: failed to get parent for performer ["
            << perfEntity << "]"
            << std::endl;
      continue;
    }

    //! @todo why was the performers model entity not also included?
    auto modelEntity = parent->Data();
    entities.insert(modelEntity);
    if (!_msg.stats().paused())
    {
      gzdbg << "Secondary: performer [" << perfEntity << "], "
            << "parent: [" << modelEntity << "]"
            <<  std:: endl;
    }
    auto children = this->dataPtr->ecm->Descendants(modelEntity);
    entities.insert(children.begin(), children.end());
  }

  msgs::SerializedStateMap stateMsg;
  if (!entities.empty())
  {
    this->dataPtr->ecm->State(stateMsg, entities);
  }
  stateMsg.set_has_one_time_component_changes(
    this->dataPtr->ecm->HasOneTimeComponentChanges());

  this->stepAckPub.Publish(stateMsg);

  this->dataPtr->ecm->SetAllComponentsUnchanged();
}
