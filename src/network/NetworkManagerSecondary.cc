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

#include <algorithm>
#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/Profiler.hh>

#include "msgs/peer_control.pb.h"

#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Events.hh"

#include "NetworkManagerPrivate.hh"
#include "NetworkManagerSecondary.hh"
#include "PeerTracker.hh"

using namespace ignition;
using namespace gazebo;

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
    ignerr << "Error advertising PeerControl service [" << controlService
           << "]" << std::endl;
  }
  else
  {
    igndbg << "Advertised PeerControl service on [" << controlService << "]"
      << std::endl;
  }

  this->node.Subscribe("step", &NetworkManagerSecondary::OnStep, this);

  this->stepAckPub = this->node.Advertise<msgs::Boolean>("step_ack");
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
    const private_msgs::SimulationStateStep &_msg)
{
  IGN_PROFILE("NetworkManagerSecondary::OnStep");

  // Throttle the number of step messages going to the debug output.
  if (!_msg.stats().paused() && _msg.stats().iterations() % 1000 == 0)
  {
    igndbg << "Network iterations: " << _msg.stats().iterations()
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

      ignmsg << "Secondary [" << this->Namespace()
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
        ignmsg << "Secondary [" << this->Namespace()
               << "] unassigned affinity to performer [" << entityId << "]."
               << std::endl;
        this->performers.erase(entityId);
      }
    }
  }

  // Update info
  auto info = convert<UpdateInfo>(_msg.stats());

  // SYNC SECONDARY ECM WITH PRIMARY ECM
  this->dataPtr->ecm->SetState(_msg.state());

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
      ignerr << "Failed to get parent for performer [" << perf << "]"
             << std::endl;
      continue;
    }
    auto modelEntity = parent->Data();

    auto children = this->dataPtr->ecm->Descendants(modelEntity);
    entities.insert(children.begin(), children.end());
  }

  // msgs::SerializedStateMap stateMsg;
  // if (!entities.empty())
  //   this->dataPtr->ecm->State(stateMsg, entities);
  // // Note on merging forward:
  // // `has_one_time_component_changes` field is available in Edifice so this
  // // workaround can be removed
  // auto data = stateMsg.mutable_header()->add_data();
  // data->set_key("has_one_time_component_changes");
  // data->add_value(
  //   this->dataPtr->ecm->HasOneTimeComponentChanges() ? "1" : "0");

  // Send completed acknowledge to the primary
  msgs::Boolean result;
  result.set_data(true);
  this->stepAckPub.Publish(result);

  this->dataPtr->ecm->SetAllComponentsUnchanged();
}

