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
#include <deque>
#include <mutex>
#include <string>
#include <tuple>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/common/Profiler.hh>

#include "msgs/peer_control.pb.h"
#include "msgs/secondary_step.pb.h"

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

  this->stepAckPub = this->node.Advertise<private_msgs::SecondaryStep>("step_ack");

  this->steppingThread = std::thread{&NetworkManagerSecondary::AsyncStepTask, this};
}

//////////////////////////////////////////////////
NetworkManagerSecondary::~NetworkManagerSecondary()
{
  {
    std::lock_guard<std::mutex> guard{this->stepsMutex};
    this->stopAsyncStepThread = true;
  }
  this->moreStepsCv.notify_one();
  this->steppingThread.join();
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

//////////////////////////////////////////////////
void NetworkManagerSecondary::AsyncStepTask()
{
  std::deque<private_msgs::SimulationStep> next_steps;
  uint64_t maxIteration{0};
  UpdateInfo lastUpdateInfo;

  while (!this->stopAsyncStepThread)
  {
    IGN_PROFILE("NetworkManagerSecondary::AsyncStepTaskLoop");

    if (next_steps.size()) {
      // Ack message received
      auto next_step = next_steps.front();
      next_steps.pop_front();
      auto info = convert<UpdateInfo>(next_step.stats());
      lastUpdateInfo = info;
      // TODO(ivanpauno): Maybe, instead of N=1000 harcoded the number of steps that the simulation
      // can move forward should be part of the message.
      maxIteration = info.iterations + 1000u;

      // Throttle the number of step messages going to the debug output.
      if (!info.paused && info.iterations % 1000 == 0)
      {
        igndbg << "Network iterations: " << info.iterations
              << std::endl;
      }

      // Update affinities
      // TODO(ivanpauno):
      // This needs to be redone based on if performers can "far apart", "can view each other" or "are interacting".
      for (int i = 0; i < next_step.affinity_size(); ++i)
      {
        const auto &affinityMsg = next_step.affinity(i);
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
          if (parent) {
            this->dataPtr->ecm->RequestRemoveEntity(parent->Data());
          }

          if (this->performers.find(entityId) != this->performers.end())
          {
            ignmsg << "Secondary [" << this->Namespace()
                  << "] unassigned affinity to performer [" << entityId << "]."
                  << std::endl;
            this->performers.erase(entityId);
          }
        }
      }
    }
    while (lastUpdateInfo.iterations < maxIteration) {
      if (this->stopAsyncStepThread) {
        return;
      }
      this->dataPtr->stepFunction(lastUpdateInfo);

      // Update state with all the performer's entities
      std::unordered_set<Entity> entities;
      for (const auto &perf : this->performers)
      {
        // Performer model
        auto parent = this->dataPtr->ecm->Component<components::ParentEntity>(perf);
        if (parent == nullptr)  // TODO(ivanpauno): why is this now needed?
        {
          ignerr << "Failed to get parent for performer [" << perf << "]"
                << std::endl;
          continue;
        }
        auto modelEntity = parent->Data();

        auto children = this->dataPtr->ecm->Descendants(modelEntity);
        entities.insert(children.begin(), children.end());
      }

      private_msgs::SecondaryStep secondaryStep;
      if (!entities.empty()) {
        this->dataPtr->ecm->State(*secondaryStep.mutable_serialized_map(), entities);
      }
      secondaryStep.set_secondary_prefix(this->Namespace());
      secondaryStep.mutable_stats()->CopyFrom(convert<msgs::WorldStatistics>(lastUpdateInfo));
      this->stepAckPub.Publish(secondaryStep);
      this->dataPtr->ecm->SetAllComponentsUnchanged();

      ++lastUpdateInfo.iterations;
      lastUpdateInfo.simTime += lastUpdateInfo.dt;
      // TODO(ivanpauno): realTime?
    }
    if (!next_steps.size()) {
      std::unique_lock<std::mutex> guard{this->stepsMutex};
      this->moreStepsCv.wait(guard, [this]{return !this->steps.empty() || this->stopAsyncStepThread;});
      next_steps = std::move(this->steps);
    }
  }
}

/////////////////////////////////////////////////
void NetworkManagerSecondary::OnStep(
    const private_msgs::SimulationStep &_msg)
{
  IGN_PROFILE("NetworkManagerSecondary::OnStep");
  {
    std::lock_guard<std::mutex> guard{this->stepsMutex};
    this->steps.emplace_back(_msg);
  }
  this->moreStepsCv.notify_one();
}
