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
#include <ignition/msgs.hh>
#include <ignition/math/Stopwatch.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/WorldStatisticsSystem.hh"
#include "WorldStatisticsComponentType.hh"

// Private data class.
class ignition::gazebo::WorldStatisticsSystemPrivate
{
  public: void OnUpdate(const EntityQuery &_result,
              EntityComponentManager &_ecMgr);

  /// \brief Realtime watch.
  public: ignition::math::Stopwatch realTimeWatch;

  /// \brief Node for communication.
  public: ignition::transport::Node node;

  public: ignition::transport::Node::Publisher publisher;
};

using namespace ignition::gazebo;

//////////////////////////////////////////////////
WorldStatisticsSystem::WorldStatisticsSystem()
  : System("WorldStatistics"),
    dataPtr(new WorldStatisticsSystemPrivate)
{
  this->dataPtr->publisher =
    this->dataPtr->node.Advertise<ignition::msgs::WorldStatistics>(
        "/ign/gazebo/stats");
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
      EntityComponentManager::ComponentType<WorldStatisticsComponentType>());
  _registrar.Register(query,
      std::bind(&WorldStatisticsSystemPrivate::OnUpdate, this->dataPtr.get(),
        std::placeholders::_1, std::placeholders::_2));
}

//////////////////////////////////////////////////
void WorldStatisticsSystemPrivate::OnUpdate(const EntityQuery &_result,
    EntityComponentManager &/*_ecMgr*/)
{
  std::cout << "WorldStatsUpdate[" << _result.Entities().size() << "]\n";
  ignition::msgs::WorldStatistics msg;
  msg.mutable_real_time()->set_sec(0);
  msg.mutable_real_time()->set_nsec(0);
  this->publisher.Publish(msg);
}
