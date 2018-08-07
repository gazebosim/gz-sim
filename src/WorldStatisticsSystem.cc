/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <list>
#include <ignition/msgs.hh>
#include <ignition/math/Stopwatch.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/WorldStatisticsSystem.hh"
#include "WorldStatisticsComponent.hh"

using namespace ignition::gazebo;
using namespace std::chrono_literals;

// Private data class.
class ignition::gazebo::WorldStatisticsSystemPrivate
{
  public: void OnUpdate(const EntityQuery &_result,
              EntityComponentManager &_ecMgr);

  /// \brief List of simulation times used to compute averages.
  public: std::list<std::chrono::steady_clock::duration> simTimes;

  /// \brief List of real times used to compute averages.
  public: std::list<std::chrono::steady_clock::duration> realTimes;

  /// \brief Node for communication.
  public: ignition::transport::Node node;

  public: ignition::transport::Node::Publisher publisher;
};

//////////////////////////////////////////////////
WorldStatisticsSystem::WorldStatisticsSystem()
  : System("WorldStatistics"),
    dataPtr(new WorldStatisticsSystemPrivate)
{
  transport::AdvertiseMessageOptions advertOpts;
  advertOpts.SetMsgsPerSec(5);
  this->dataPtr->publisher =
    this->dataPtr->node.Advertise<ignition::msgs::WorldStatistics>(
        "/world_stats", advertOpts);
}

//////////////////////////////////////////////////
WorldStatisticsSystem::~WorldStatisticsSystem()
{
}

//////////////////////////////////////////////////
void WorldStatisticsSystem::Init(EntityQueryRegistrar &_registrar)
{
  EntityQuery query;
  query.AddComponentType(
      EntityComponentManager::ComponentType<WorldStatisticsComponent>());
  _registrar.Register(query,
      std::bind(&WorldStatisticsSystemPrivate::OnUpdate, this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2));
}

//////////////////////////////////////////////////
void WorldStatisticsSystemPrivate::OnUpdate(const EntityQuery &_result,
    EntityComponentManager &_ecMgr)
{
  // Get the world stats component.
  const auto *worldStats = _ecMgr.ComponentMutable<WorldStatisticsComponent>(
        *_result.Entities().begin());

  // Get the real time duration
  const std::chrono::steady_clock::duration &realTime =
    worldStats->RealTime().ElapsedRunTime();

  // Get the sim time duration
  const std::chrono::steady_clock::duration &simTime = worldStats->SimTime();

  // Store the real time, and maintain a list size of 20.
  this->realTimes.push_back(realTime);
  if (this->realTimes.size() > 20)
    this->realTimes.pop_front();

  // Store the sim time, and maintain a window size of 20.
  this->simTimes.push_back(simTime);
  if (this->simTimes.size() > 20)
    this->simTimes.pop_front();

  // Compute the average sim ang real times.
  std::chrono::steady_clock::duration simAvg, realAvg;
  std::list<std::chrono::steady_clock::duration>::iterator simIter, realIter;
  simIter = ++(this->simTimes.begin());
  realIter = ++(this->realTimes.begin());
  while (simIter != this->simTimes.end() && realIter != this->realTimes.end())
  {
    simAvg += ((*simIter) - this->simTimes.front());
    realAvg += ((*realIter) - this->realTimes.front());
    ++simIter;
    ++realIter;
  }

  if (this->publisher.WillPublish())
  {
    // Create the world statistics message.
    ignition::msgs::WorldStatistics msg;
    if (realAvg != 0ns)
    {
      msg.set_real_time_factor(
          static_cast<double>(simAvg.count()) / realAvg.count());
    }

    std::pair<int64_t, int64_t> realTimeSecNsec =
      ignition::math::durationToSecNsec(realTime);

    std::pair<int64_t, int64_t> simTimeSecNsec =
      ignition::math::durationToSecNsec(simTime);

    msg.mutable_real_time()->set_sec(realTimeSecNsec.first);
    msg.mutable_real_time()->set_nsec(realTimeSecNsec.second);

    msg.mutable_sim_time()->set_sec(simTimeSecNsec.first);
    msg.mutable_sim_time()->set_nsec(simTimeSecNsec.second);

    // Publish the message
    this->publisher.Publish(msg);
  }
}
