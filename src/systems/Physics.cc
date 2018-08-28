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

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/EntityQuery.hh"
#include "ignition/gazebo/SystemQueryResponse.hh"

#include "ignition/gazebo/systems/Physics.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"

using namespace ignition::gazebo::systems;

using namespace std::chrono_literals;

// Private data class.
class ignition::gazebo::systems::PhysicsPrivate
{
  /// \brief Query callback for entity that has physics components.
  /// \param[in] _response The system query response data.
  public: void OnUpdate(const UpdateInfo _info,
      SystemQueryResponse &_response);

  /// \brief Query callback to update time.
  /// \param[in] _response The system query response data.
  public: void OnUpdateTime(SystemQueryResponse &_response);
};

//////////////////////////////////////////////////
Physics::Physics()
  : System("Physics"), dataPtr(new PhysicsPrivate)
{
}

//////////////////////////////////////////////////
Physics::~Physics()
{
}

//////////////////////////////////////////////////
void Physics::Init(EntityQueryRegistrar &_registrar)
{
  {
    /// \todo(nkoenig) support curly-bracket initialization of EntityQuery.
    EntityQuery query;
    query.AddComponentType(
        EntityComponentManager::ComponentType<components::Pose>());

    _registrar.Register(query,
        std::bind(&PhysicsPrivate::OnUpdate, this->dataPtr.get(),
          std::placeholders::_1, std::placeholders::_2));
  }
}

//////////////////////////////////////////////////
void PhysicsPrivate::OnUpdate(const UpdateInfo _info,
    SystemQueryResponse &_response)
{
  std::chrono::seconds sec(1);
  igndbg << "Sim time ["
         << std::chrono::duration<double>(_info.simTime).count()
         << "] Real time ["
         << std::chrono::duration<double>(_info.realTime).count()
         << "] Iterations ["
         << _info.iterations
         << "]" << std::endl;

  // Sleep for some amount of time to simulate the computation needed to
  // update physics.
  _response.EntityComponentMgr().Each<components::Name, components::Pose>(
    [&](const EntityId &/*_entity*/,
        const components::Name *_name,
        const components::Pose *_pose)
    {
      igndbg << "  --  " << _name->Data() << " pose [" << _pose->Data()
             << "]\n";
      std::this_thread::sleep_for(50us);
    });

  // \todo(louise) Step ign-physics world by _info.dt

  // \todo(nkoenig) AcutallyUpdate dynamics

  // \todo(nkoenig) Update collisions

  // \todo(nkoenig) Update entity pose information.
}
