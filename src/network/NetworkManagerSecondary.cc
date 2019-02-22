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
#include "NetworkManagerSecondary.hh"

#include <algorithm>
#include <string>

#include "ignition/common/Console.hh"
#include "ignition/common/Util.hh"
#include "ignition/gazebo/Events.hh"

#include "msgs/peer_control.pb.h"

#include "NetworkManagerPrivate.hh"
#include "PeerTracker.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
NetworkManagerSecondary::NetworkManagerSecondary(
    EventManager *_eventMgr, const NetworkConfig &_config,
    const NodeOptions &_options):
  NetworkManager(_eventMgr, _config, _options),
  node(_options)
{
  std::string topic { this->Namespace() + "/control" };

  if (!this->node.Advertise(topic, &NetworkManagerSecondary::OnControl, this))
  {
    ignerr << "Error advertising PeerControl service" << std::endl;
  }
  else
  {
    igndbg << "Advertised PeerControl service on [" << topic << "]"
      << std::endl;
  }

  this->node.Subscribe("step", &NetworkManagerSecondary::OnStep, this);

  std::string ackTopic { this->Namespace() + "/stepAck" };
  this->stepAckPub = this->node.Advertise<msgs::SimulationStep>(ackTopic);

  auto eventMgr = this->dataPtr->eventMgr;
  if (eventMgr)
  {
    this->dataPtr->peerRemovedConn = eventMgr->Connect<PeerRemoved>(
        [this](PeerInfo _info){
          if (_info.role == NetworkRole::SimulationPrimary)
          {
            ignerr << "Primary removed, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
    });

    this->dataPtr->peerStaleConn = eventMgr->Connect<PeerStale>(
        [this](PeerInfo _info){
          if (_info.role == NetworkRole::SimulationPrimary)
          {
            ignerr << "Secondary went stale, stopping simulation" << std::endl;
            this->dataPtr->eventMgr->Emit<events::Stop>();
          }
    });
  }
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::Ready() const
{
  // The detected number of peers in the "Primary" role must be 1
  auto primaries = this->dataPtr->tracker->NumPrimary();
  return (primaries == 1);
}

//////////////////////////////////////////////////
void NetworkManagerSecondary::Initialize()
{
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::Step(
    uint64_t &_iteration,
    std::chrono::steady_clock::duration &_stepSize,
    std::chrono::steady_clock::duration &_simTime)
{
  if (!this->enableSim)
  {
    return false;
  }

  std::unique_lock<std::mutex> lock(this->stepMutex);
  auto status = this->stepCv.wait_for(lock,
      std::chrono::nanoseconds(500),
      [this](){return this->currentStep != nullptr;});

  if (status) {
    if (_iteration != 0 && _iteration % 1000 == 0)
    {
      igndbg << "NetworkStep: " << _iteration << std::endl;
    }
    _iteration = this->currentStep->iteration();
    _stepSize = std::chrono::steady_clock::duration(
        std::chrono::nanoseconds(this->currentStep->stepsize()));
    _simTime = std::chrono::steady_clock::duration(
        std::chrono::seconds(this->currentStep->simtime().sec()) +
        std::chrono::nanoseconds(this->currentStep->simtime().nsec()));
    this->currentStep.reset();
  }

  return status;
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::StepAck(uint64_t _iteration)
{
  auto step = msgs::SimulationStep();
  step.set_iteration(_iteration);
  this->stepAckPub.Publish(step);
  return true;
}

//////////////////////////////////////////////////
std::string NetworkManagerSecondary::Namespace() const
{
  return this->dataPtr->peerInfo.id.substr(0, 8);
}

//////////////////////////////////////////////////
bool NetworkManagerSecondary::OnControl(const msgs::PeerControl &_req,
                                        ignition::msgs::Empty &/*_resp*/)
{
  this->enableSim = _req.enable_sim();
  this->pauseSim = _req.pause_sim();
  return true;
}

//////////////////////////////////////////////////
void NetworkManagerSecondary::OnStep(const msgs::SimulationStep &_msg)
{
  std::unique_lock<std::mutex> lock(this->stepMutex);
  this->currentStep = std::make_unique<msgs::SimulationStep>(_msg);
  lock.unlock();
  this->stepCv.notify_all();
}
