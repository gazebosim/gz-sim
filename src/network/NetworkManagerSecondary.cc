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
  return true;
}

/////////////////////////////////////////////////
void NetworkManagerSecondary::OnStep(
    const private_msgs::SimulationStep &_msg)
{
  GZ_PROFILE("NetworkManagerSecondary::OnStep");

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
    const auto &entityId = affinityMsg.entity().id();

    if (affinityMsg.secondary_prefix() == this->Namespace())
    {
      this->performers.insert(entityId);

      gzmsg << "Secondary [" << this->Namespace()
             << "] assigned affinity to performer [" << entityId << "]."
             << std::endl;
    }
    // If performer has been assigned to another secondary, remove it
    else
    {
      auto parent =
          this->dataPtr->ecm->Component<components::ParentEntity>(entityId);
      this->dataPtr->ecm->RequestRemoveEntity(parent->Data());

      if (this->performers.find(entityId) != this->performers.end())
      {
        gzmsg << "Secondary [" << this->Namespace()
               << "] unassigned affinity to performer [" << entityId << "]."
               << std::endl;
        this->performers.erase(entityId);
      }
    }
  }

  // Update info
  auto info = convert<UpdateInfo>(_msg.stats());

  // Step runner
  this->dataPtr->stepFunction(info);

  // Update state with all the performer's entities
  std::unordered_set<Entity> entities;
  for (const auto &perf : this->performers)
  {
    // Performer model
    auto parent = this->dataPtr->ecm->Component<components::ParentEntity>(perf);
    if (parent == nullptr)
    {
      gzerr << "Failed to get parent for performer [" << perf << "]"
             << std::endl;
      continue;
    }
    auto modelEntity = parent->Data();

    auto children = this->dataPtr->ecm->Descendants(modelEntity);
    entities.insert(children.begin(), children.end());
  }

  msgs::SerializedStateMap stateMsg;
  if (!entities.empty())
    this->dataPtr->ecm->State(stateMsg, entities);
  stateMsg.set_has_one_time_component_changes(
    this->dataPtr->ecm->HasOneTimeComponentChanges());

  this->stepAckPub.Publish(stateMsg);

  this->dataPtr->ecm->SetAllComponentsUnchanged();
}
