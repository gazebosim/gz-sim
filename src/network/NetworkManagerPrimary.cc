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
#include "msgs/simulation_step.pb.h"

#include "ignition/gazebo/components/PerformerAffinity.hh"
#include "ignition/gazebo/components/PerformerLevels.hh"
#include "ignition/gazebo/Conversions.hh"
#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/Events.hh"

#include "NetworkManagerPrimary.hh"
#include "NetworkManagerPrivate.hh"
#include "PeerTracker.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManagerPrimary::NetworkManagerPrimary(
    const std::function<void(const UpdateInfo &_info)> &_stepFunction,
    EntityComponentManager &_ecm, EventManager *_eventMgr,
    const NetworkConfig &_config, const NodeOptions &_options):
  NetworkManager(_stepFunction, _ecm, _eventMgr, _config, _options),
  node(_options)
{
  this->simStepPub = this->node.Advertise<private_msgs::SimulationStep>("step");

  this->node.Subscribe("step_ack", &NetworkManagerPrimary::OnStepAck, this);
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::Handshake()
{
  auto peers = this->dataPtr->tracker->SecondaryPeers();
  for (const auto &peer : peers)
  {
    private_msgs::PeerControl req, resp;
    req.set_enable_sim(true);

    auto sc = std::make_unique<SecondaryControl>();
    sc->id = peer;
    sc->prefix = peer.substr(0, 8);

    bool result;
    std::string topic {sc->prefix + "/control"};
    unsigned int timeout = 5000;

    igndbg << "Registering secondary [" << topic << "]" << std::endl;
    bool executed = this->node.Request(topic, req, timeout, resp, result);

    if (executed)
    {
      if (result)
      {
        ignmsg << "Peer initialized [" << sc->prefix << "]" << std::endl;
        sc->ready = true;
      }
      else
      {
        ignerr << "Peer service call failed [" << sc->prefix << "]"
          << std::endl;
      }
    }
    else
    {
      ignerr << "Peer service call timed out [" << sc->prefix << "], waited "
             << timeout << " ms" << std::endl;
    }

    this->secondaries[sc->prefix] = std::move(sc);
  }
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::Ready() const
{
  // The detected number of peers in the "Secondary" role must match
  // the number exepected (set via configuration of environment).
  auto nSecondary = this->dataPtr->tracker->NumSecondary();
  return (nSecondary == this->dataPtr->config.numSecondariesExpected);
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::Step(const UpdateInfo &_info)
{
  IGN_PROFILE("NetworkManagerPrimary::Step");

  // Check all secondaries have been registered
  bool ready = true;
  for (const auto &secondary : this->secondaries)
  {
    ready &= secondary.second->ready;
  }

  // TODO(louise) Wait for peers to be ready in a loop?
  if (!ready)
  {
    ignerr << "Trying to step network primary before all peers are ready."
           << std::endl;
    return false;
  }

  private_msgs::SimulationStep step;
  step.mutable_stats()->CopyFrom(convert<msgs::WorldStatistics>(_info));

  // Affinities that changed this step
  this->PopulateAffinities(step);

  // Check all secondaries are ready to receive steps - only do this once at
  // startup
  if (!this->SecondariesCanStep())
  {
    return false;
  }

  // Send step to all secondaries
  this->secondaryStates.clear();
  this->simStepPub.Publish(step);

  // Block until all secondaries are done
  {
    IGN_PROFILE("Waiting for secondaries");

    int sleep = 0;
    int maxSleep = 10 * 1000 * 1000;
    for (; sleep < maxSleep &&
       (this->secondaryStates.size() < this->secondaries.size()); ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::nanoseconds(1));
    }

    if (sleep == maxSleep)
    {
      ignerr << "Waited 10 s and got only [" << this->secondaryStates.size()
             << " / " << this->secondaries.size()
             << "] responses from secondaries. Stopping simulation."
             << std::endl;
      this->dataPtr->eventMgr->Emit<events::Stop>();
      return false;
    }
  }

  // Update primary state with states received from secondaries
  {
    IGN_PROFILE("Updating primary state");
    for (const auto &msg : this->secondaryStates)
    {
      this->dataPtr->ecm->SetState(msg);
    }
    this->secondaryStates.clear();
  }

  // Step all systems
  this->dataPtr->stepFunction(_info);

  this->dataPtr->ecm->SetAllComponentsUnchanged();

  return true;
}

//////////////////////////////////////////////////
std::string NetworkManagerPrimary::Namespace() const
{
  return "";
}

//////////////////////////////////////////////////
std::map<std::string, SecondaryControl::Ptr>
    &NetworkManagerPrimary::Secondaries()
{
  return this->secondaries;
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::OnStepAck(const msgs::SerializedStateMap &_msg)
{
  this->secondaryStates.push_back(_msg);
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::SecondariesCanStep() const
{
  // TODO(anyone) Ideally we'd check the number of connections against the
  // number of expected secondaries, but there's no interface for that
  // on ign-transport yet:
  // https://github.com/ignitionrobotics/ign-transport/issues/39
  return this->simStepPub.HasConnections();
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::PopulateAffinities(
    private_msgs::SimulationStep &_msg)
{
  IGN_PROFILE("NetworkManagerPrimary::PopulateAffinities");

  // p: performer
  // l: level
  // s: secondary

  // Previous performer-to-secondary mapping - may need updating
  std::map<Entity, std::string> pToSPrevious;

  // Updated performer-to-level mapping - used to update affinities
  std::map<Entity, std::set<Entity>> lToPNew;

  // All performers
  std::set<Entity> allPerformers;

  // Go through performers and assign affinities
  this->dataPtr->ecm->Each<
        components::PerformerLevels>(
    [&](const Entity &_entity,
        const components::PerformerLevels *_perfLevels) -> bool
    {
      allPerformers.insert(_entity);

      // Previous affinity
      auto currentAffinityComp =
          this->dataPtr->ecm->Component<components::PerformerAffinity>(_entity);
      if (currentAffinityComp)
      {
        pToSPrevious[_entity] = currentAffinityComp->Data();
      }

      // New levels
      for (const auto &level : _perfLevels->Data())
      {
        lToPNew[level].insert(_entity);
      }

      return true;
    });

  // First assignment: distribute levels evenly across secondaries
  if (pToSPrevious.empty())
  {
    auto secondaryIt = this->secondaries.begin();

    for (const auto &[level, performers] : lToPNew)
    {
      for (const auto &performer : performers)
      {
        this->SetAffinity(performer, secondaryIt->second->prefix,
            _msg.add_affinity());

        // Remove performers as they are assigned
        allPerformers.erase(performer);
      }

      // Round-robin levels
      secondaryIt++;
      if (secondaryIt == this->secondaries.end())
      {
        secondaryIt = this->secondaries.begin();
      }
    }

    // Also assign level-less performers
    for (auto performer : allPerformers)
    {
      this->SetAffinity(performer, secondaryIt->second->prefix,
          _msg.add_affinity());

      // Round-robin performers
      secondaryIt++;
      if (secondaryIt == this->secondaries.end())
      {
        secondaryIt = this->secondaries.begin();
      }
    }
    return;
  }

  // TODO(louise) Process level changes
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::SetAffinity(Entity _performer,
    const std::string &_secondary, private_msgs::PerformerAffinity *_msg)
{
  // Populate message
  _msg->mutable_entity()->set_id(_performer);
  _msg->set_secondary_prefix(_secondary);

  // Set component
  this->dataPtr->ecm->RemoveComponent<components::PerformerAffinity>(
      _performer);

  auto newAffinity = components::PerformerAffinity(_secondary);
  this->dataPtr->ecm->CreateComponent(_performer, newAffinity);
}

