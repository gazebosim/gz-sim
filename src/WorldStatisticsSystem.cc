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
#include "ignition/gazebo/WorldStatisticsSystem.hh"
#include <ignition/msgs.hh>
#include <ignition/math/Stopwatch.hh>
#include <ignition/transport/Node.hh>

// Private data class.
class ignition::gazebo::WorldStatisticsSystemPrivate
{
  /// \brief Realtime watch.
  public: ignition::math::Stopwatch realTimeWatch;

  /// \brief Node for communication.
  public: ignition::transport::Node node;

  public: ignition::transport::Node::Publisher publisher;
};

using namespace ignition::gazebo;

//////////////////////////////////////////////////
WorldStatisticsSystem::WorldStatisticsSystem(const SystemConfig &_config)
  : System("WorldStatistics", _config),
    dataPtr(new WorldStatisticsSystemPrivate)
{
  this->dataPtr->publisher =
    this->dataPtr->node.Advertise<ignition::msgs::WorldStatistics>(
        "/ign/gazebo/stats");
/*
   NEED TO GET the stopwatch PR into gz11 branch on ign-math
   CHECK condition wait, need to add lambda.
  */
}

//////////////////////////////////////////////////
WorldStatisticsSystem::~WorldStatisticsSystem()
{
}

//////////////////////////////////////////////////
void WorldStatisticsSystem::Init()
{
}

//////////////////////////////////////////////////
/*void WorldStatisticsSystem::Update()
{
  if (!this->dataPtr->realTimeWatch.Running())
    this->dataPtr->realTimeWatch.Start();
  ignition::msgs::WorldStats msg;
  msg.mutable_real_time()->set_sec(0)
  msg.mutable_real_time()->set_nsec(0)
  this->dataPtr->node->Publish(msg);
}
  */
