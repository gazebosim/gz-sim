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
#include <iomanip>

#include <ignition/math/Pose3.hh>
#include <ignition/plugin/RegisterMore.hh>

#include "ignition/gazebo/EntityComponentManager.hh"

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/systems/Physics.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"

using namespace ignition::gazebo::systems;

using namespace std::chrono_literals;

// Private data class.
class ignition::gazebo::systems::PhysicsPrivate
{
};

//////////////////////////////////////////////////
Physics::Physics()
  : System(), dataPtr(new PhysicsPrivate)
{
}

//////////////////////////////////////////////////
Physics::~Physics()
{
}

//////////////////////////////////////////////////
void Physics::Update(const UpdateInfo &_info,
    EntityComponentManager &_manager)
{
  igndbg << "Sim time ["
         << std::chrono::duration<double>(_info.simTime).count()
         << "] Real time ["
         << std::chrono::duration<double>(_info.realTime).count()
         << "] Iterations ["
         << _info.iterations
         << "] dt ["
         << std::chrono::duration<double>(_info.dt).count()
         << "]" << std::endl;

  // Skip physics update if paused
  if (_info.dt.count() == 0)
  {
    return;
  }

  // Sleep for some amount of time to simulate the computation needed to
  // update physics.
  std::this_thread::sleep_for(50us);

  _manager.Each<components::Name, components::Pose>(
    [&](const EntityId &/*_entity*/,
        const components::Name *_name,
        const components::Pose *_pose)
    {
      igndbg << "  --  " << _name->Data() << " pose [" << _pose->Data()
             << "]\n";
    });

  // \todo(louise) Step ign-physics world by _info.dt

  // \todo(nkoenig) AcutallyUpdate dynamics

  // \todo(nkoenig) Update collisions

  // \todo(nkoenig) Update entity pose information.
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Physics,
                    ignition::gazebo::System,
                    Physics::ISystemUpdate)
