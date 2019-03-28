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
#include "NetworkManagerPrimary.hh"

#include <algorithm>
#include <string>

#include "ignition/common/Console.hh"
#include "ignition/common/Util.hh"
#include "ignition/gazebo/Events.hh"

#include "msgs/peer_control.pb.h"
#include "msgs/simulation_step.pb.h"

#include "NetworkManagerPrivate.hh"
#include "PeerTracker.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManagerPrimary::NetworkManagerPrimary(
    EventManager *_eventMgr, const NetworkConfig &_config,
    const NodeOptions &_options):
  NetworkManager(_eventMgr, _config, _options),
  node(_options)
{
  this->simStepPub = this->node.Advertise<private_msgs::SimulationStep>("step");

  auto eventMgr = this->dataPtr->eventMgr;
  if (eventMgr)
  {
    this->dataPtr->peerRemovedConn = eventMgr->Connect<PeerRemoved>(
        [this](PeerInfo _info)
        {
          if (_info.role == NetworkRole::SimulationSecondary)
          {
            ignerr << "Secondary removed, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
        });

    this->dataPtr->peerStaleConn = eventMgr->Connect<PeerStale>(
        [this](PeerInfo _info)
        {
          if (_info.role == NetworkRole::SimulationSecondary)
          {
            ignerr << "Secondary went stale, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
        });
  }
  else
  {
    ignwarn << "NetworkManager started without EventManager. "
      << "Distributed environment may not terminate correctly" << std::endl;
  }
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::Initialize()
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

    igndbg << "Attempting to register secondary [" << topic << "]" << std::endl;
    bool executed = this->node.Request(topic, req, timeout, resp, result);

    if (executed)
    {
      if (result)
      {
        igndbg << "Peer initialized [" << sc->prefix << "]" << std::endl;
        sc->ready = true;
      }
      else
      {
        igndbg << "Peer service call failed [" << sc->prefix << "]"
          << std::endl;
      }
    }
    else
    {
      igndbg << "Peer service call timed out [" << sc->prefix << "]"
        << std::endl;
    }

    auto ackTopic = std::string {sc->prefix + "/stepAck"};
    std::function<void(const private_msgs::SimulationStep&)> fcn =
        std::bind(&NetworkManagerPrimary::OnStepAck, this, sc->prefix,
            std::placeholders::_1);
    this->node.Subscribe<private_msgs::SimulationStep>(ackTopic, fcn);

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
bool NetworkManagerPrimary::Step(UpdateInfo &_info)
{
  bool ready = true;
  for (const auto &secondary : this->secondaries)
  {
    ready &= secondary.second->ready;
  }

  if (ready)
  {
    // Throttle the number of step messages going to the debug output.
    if (!_info.paused && _info.iterations % 1000 == 0)
    {
      igndbg << "Network iterations: " << _info.iterations << std::endl;
    }

    auto step = private_msgs::SimulationStep();
    step.set_iteration(_info.iterations);
    step.set_paused(_info.paused);

    auto stepSizeSecNsec =
      ignition::math::durationToSecNsec(_info.dt);
    step.set_stepsize(stepSizeSecNsec.second);

    auto simTimeSecNsec =
      ignition::math::durationToSecNsec(_info.simTime);
    step.mutable_simtime()->set_sec(simTimeSecNsec.first);
    step.mutable_simtime()->set_nsec(simTimeSecNsec.second);
    this->simStepPub.Publish(step);
  }
  return ready;
}

//////////////////////////////////////////////////
bool NetworkManagerPrimary::StepAck(uint64_t _iteration)

{
  bool stepAck = true;
  bool iters = true;
  for (const auto &secondary : this->secondaries)
  {
    stepAck &= secondary.second->recvStepAck;
    iters &= (_iteration == secondary.second->recvIter);
  }

  if (stepAck && iters)
  {
    for (auto &secondary : this->secondaries)
    {
      secondary.second->recvStepAck = false;
    }
  }

  return (stepAck && iters);
}

//////////////////////////////////////////////////
std::string NetworkManagerPrimary::Namespace() const
{
  return "";
}

//////////////////////////////////////////////////
void NetworkManagerPrimary::OnStepAck(const std::string &_secondary,
      const private_msgs::SimulationStep &_msg)
{
  this->secondaries[_secondary]->recvStepAck = true;
  this->secondaries[_secondary]->recvIter = _msg.iteration();
}

//////////////////////////////////////////////////
std::map<std::string, SecondaryControl::Ptr>&
NetworkManagerPrimary::Secondaries()
{
  return this->secondaries;
}
