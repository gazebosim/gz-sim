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
#include "ignition/gazebo/systems/WorldStatistics.hh"

#include <ignition/msgs/world_stats.pb.h>

#include <list>
#include <ignition/math/Stopwatch.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/SystemQueryResponse.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/WorldStatistics.hh"

using namespace ignition::gazebo::systems;
using namespace std::chrono_literals;

// Private data class.
class ignition::gazebo::systems::WorldStatisticsPrivate
{
  /// \brief Entity query callback for all worlds.
  /// \param[in] _response The system query response data.
  public: void OnUpdate(SystemQueryResponse &_response);

  /// \brief Local storage for statistics computation and publication.
  public: class Stats
  {
    /// \brief List of simulation times used to compute averages.
    public: std::list<std::chrono::steady_clock::duration> simTimes;

    /// \brief List of real times used to compute averages.
    public: std::list<std::chrono::steady_clock::duration> realTimes;

    /// \brief Publisher for this data.
    public: ignition::transport::Node::Publisher publisher;
  };

  /// \brief Node for communication.
  public: ignition::transport::Node node;

  /// \brief Stats for all the worlds, where the key is the world name.
  public: std::map<std::string, Stats> stats;
};

//////////////////////////////////////////////////
WorldStatistics::WorldStatistics()
  : System(),
    dataPtr(new WorldStatisticsPrivate)
{
}

//////////////////////////////////////////////////
WorldStatistics::~WorldStatistics()
{
}

//////////////////////////////////////////////////
void WorldStatistics::Init(EntityQueryRegistrar &_registrar)
{
  // Register a query that will get all entities with
  // a WorldStatistics component. This should be just world entities, which
  // is usually a single entity on the server.
  EntityQuery query;
  query.AddComponentType(
      EntityComponentManager::ComponentType<components::WorldStatistics>());
  _registrar.Register(query,
      std::bind(&WorldStatisticsPrivate::OnUpdate, this->dataPtr.get(),
        std::placeholders::_1));
}

//////////////////////////////////////////////////
void WorldStatisticsPrivate::OnUpdate(SystemQueryResponse &_response)
{
  std::map<std::string, Stats>::iterator iter;

  // Process each entity.
  for (const EntityId &entity : _response.Query().Entities())
  {
    // Get the world stats component.
    auto *worldStats = _response
        .EntityComponentMgr()
        .ComponentMutable<components::WorldStatistics>(entity);

    if (!worldStats)
    {
      ignerr << "A world entity does not have a WorldStatistics component.\n"
        << std::endl;
      continue;
    }

    // Get the world component.
    const auto *world =
      _response.EntityComponentMgr().Component<components::World>(entity);

    if (!world)
    {
      ignerr << "A world entity does not have a World component.\n"
        << std::endl;
      continue;
    }

    worldStats->RealTime().Start();

    // Find the local world stats information.
    iter = this->stats.find(world->Name());

    // Create a world stats if it doesn't exist.
    if (iter == this->stats.end())
    {
      // Create the world statistics publisher.
      transport::AdvertiseMessageOptions advertOpts;
      advertOpts.SetMsgsPerSec(5);
      this->stats[world->Name()].publisher =
        this->node.Advertise<ignition::msgs::WorldStatistics>(
            "/world/" + world->Name() + "/stats", advertOpts);
      iter = this->stats.find(world->Name());
    }

    // Get the real time duration
    const std::chrono::steady_clock::duration &realTime =
      worldStats->RealTime().ElapsedRunTime();

    // Get the sim time duration
    const std::chrono::steady_clock::duration &simTime = worldStats->SimTime();

    Stats &entityStats = iter->second;

    // Store the real time, and maintain a window size of 20.
    entityStats.realTimes.push_back(realTime);
    if (entityStats.realTimes.size() > 20)
      entityStats.realTimes.pop_front();

    // Store the sim time, and maintain a window size of 20.
    entityStats.simTimes.push_back(simTime);
    if (entityStats.simTimes.size() > 20)
      entityStats.simTimes.pop_front();

    // Compute the average sim ang real times.
    std::chrono::steady_clock::duration simAvg{0}, realAvg{0};
    std::list<std::chrono::steady_clock::duration>::iterator simIter,
      realIter;

    simIter = ++(entityStats.simTimes.begin());
    realIter = ++(entityStats.realTimes.begin());
    while (simIter != entityStats.simTimes.end() &&
        realIter != entityStats.realTimes.end())
    {
      simAvg += ((*simIter) - entityStats.simTimes.front());
      realAvg += ((*realIter) - entityStats.realTimes.front());
      ++simIter;
      ++realIter;
    }

    // Create the world statistics message.
    ignition::msgs::WorldStatistics msg;
    if (realAvg != 0ns)
    {
      msg.set_real_time_factor(math::precision(
            static_cast<double>(simAvg.count()) / realAvg.count(), 4));
    }

    std::pair<int64_t, int64_t> realTimeSecNsec =
      ignition::math::durationToSecNsec(realTime);

    std::pair<int64_t, int64_t> simTimeSecNsec =
      ignition::math::durationToSecNsec(simTime);

    msg.mutable_real_time()->set_sec(realTimeSecNsec.first);
    msg.mutable_real_time()->set_nsec(realTimeSecNsec.second);

    msg.mutable_sim_time()->set_sec(simTimeSecNsec.first);
    msg.mutable_sim_time()->set_nsec(simTimeSecNsec.second);

    msg.set_iterations(worldStats->Iterations());

    // Publish the message
    entityStats.publisher.Publish(msg);
  }
}
