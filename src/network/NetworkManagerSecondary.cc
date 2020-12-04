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

  this->stepAckPub = this->node.Advertise<msgs::SerializedStateMap>("step_ack");

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
  bool first_run = true;
  while (!this->stopAsyncStepThread)
  {
    IGN_PROFILE("NetworkManagerSecondary::AsyncStepTaskLoop");

    if (next_steps.size()) {
      auto next_step = next_steps.front();
      auto info = convert<UpdateInfo>(next_step.stats());
      auto it = this->history.find(info.iterations);
      if (this->history.end() != it) {
        // TODO: paused? rewind? seek? Maybe, checking dt and simTime is enough
        if (std::get<1>(it->second) == info.dt && std::get<2>(it->second) == info.simTime) {
          this->stepAckPub.Publish(std::get<0>(it->second));
          next_steps.pop_front();
          // Remove previous iterations, taking advantage of ordered map.
          this->history.erase(this->history.begin(), it);
        } else {
          this->history.clear(); // step size or sim time changed, flush history
          this->lastUpdateInfo = info;
        }
      } else if (first_run) {
        this->lastUpdateInfo = info;
        first_run = false;
      }
      // TODO(ivanpauno): how many iterations can the secondary move ahead?
      this->maxIteration = info.iterations + 10000u;
      ignerr << "iterations: " << info.iterations << std::endl;
      ignerr << "last update iterations: " << this->lastUpdateInfo.iterations << std::endl;
      ignerr << "max iteration: " << this->maxIteration << std::endl;
      ignerr << "cached: " << this->history.size() << std::endl;

      // Throttle the number of step messages going to the debug output.
      if (!info.paused && info.iterations % 1000 == 0)
      {
        igndbg << "Network iterations: " << info.iterations
              << std::endl;
      }

      // Update affinities
      // TODO(ivanpauno): flush history if affinities changed
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
    } else {
      {
        std::lock_guard<std::mutex> guard{this->stepsMutex};
        next_steps = std::move(this->steps);
      }
    }
    if (this->lastUpdateInfo.iterations < this->maxIteration) {
      this->dataPtr->stepFunction(this->lastUpdateInfo);

      // Update state with all the performer's entities
      std::unordered_set<Entity> entities;
      for (const auto &perf : this->performers)
      {
        // Performer model
        auto parent = this->dataPtr->ecm->Component<components::ParentEntity>(perf);
        if (parent == nullptr)  // TODO: why is this now needed?
        {
          ignerr << "Failed to get parent for performer [" << perf << "]"
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

      this->history.emplace(
        this->lastUpdateInfo.iterations,
        std::make_tuple(stateMsg, this->lastUpdateInfo.dt, this->lastUpdateInfo.simTime));

      this->dataPtr->ecm->SetAllComponentsUnchanged();

      ++this->lastUpdateInfo.iterations;
      this->lastUpdateInfo.simTime += this->lastUpdateInfo.dt;
      // TODO: realTime?
    } else if (next_steps.empty()) {
      std::unique_lock<std::mutex> guard{this->stepsMutex};
      this->moreStepsCv.wait(guard, [this]{return !this->steps.empty() || this->stopAsyncStepThread;});
    }
  }
}

/////////////////////////////////////////////////
void NetworkManagerSecondary::OnStep(
    const private_msgs::SimulationStep &_msg)
{
  IGN_PROFILE("NetworkManagerSecondary::OnStep");
  ignerr << "ASYNCSTEPTASK: Got step" << std::endl;
  {
    std::lock_guard<std::mutex> guard{this->stepsMutex};
    this->steps.emplace_back(_msg);
  }
  this->moreStepsCv.notify_one();
}
