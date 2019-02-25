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

#include <sdf/Geometry.hh>
#include <sdf/Light.hh>
#include <sdf/Model.hh>
#include <sdf/World.hh>

#include "ignition/gazebo/Events.hh"
#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/Gravity.hh"
#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/LevelBuffer.hh"
#include "ignition/gazebo/components/LevelEntityNames.hh"
#include "ignition/gazebo/components/MagneticField.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"

#include "SyncManager.hh"
#include "SimulationRunner.hh"

#include "network/NetworkManagerPrimary.hh"
#include "network/NetworkManagerSecondary.hh"

#include "msgs/performer_affinity.pb.h"


using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
SyncManager::SyncManager(SimulationRunner *_runner, bool _useLevels,
    bool _useDistSim)
    : runner(_runner),
      useLevels(_useLevels),
      useDistSim(_useDistSim)
{
  this->entityCreator = std::make_unique<SdfEntityCreator>(
      this->runner->entityCompMgr,
      this->runner->eventMgr);

  if (!_useDistSim)
  {
    igndbg << "Distributed Sim off, SyncManager disabled." << std::endl;
    return;
  }

  if (_useDistSim && !this->runner->networkMgr) {
    ignerr << "Cannot start distributed simulation. " <<
      "Server was given --distributed flag, " <<
      "but invalid configuration detected." << std::endl;
    return;
  }

  this->role = this->runner->networkMgr->Role();

  if (this->role != NetworkRole::None)
  {
    //this->posePub = this->node.Advertise<ignition::msgs::Pose_V>("pose_update");
  }

  if (this->role == NetworkRole::SimulationPrimary)
  {
    //this->node.Subscribe("pose_update", &SyncManager::OnPose, this);
  }

  this->worldEntity = this->runner->entityCompMgr.EntityByComponents(components::World());
}
