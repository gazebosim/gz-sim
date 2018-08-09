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
#include <ignition/math/Pose3.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/EntityQuery.hh"
#include "ignition/gazebo/PhysicsSystem.hh"
#include "ignition/gazebo/SystemQueryResponse.hh"

#include "ignition/gazebo/PoseComponent.hh"
#include "ignition/gazebo/WorldStatisticsComponent.hh"

using namespace ignition::gazebo;
using namespace std::chrono_literals;

// Private data class.
class ignition::gazebo::PhysicsSystemPrivate
{
  /// \brief Query callback for entity that have physics components.
  /// \param[in] _response The system query response data.
  public: void OnUpdate(SystemQueryResponse &_response);

  /// \brief Query callback to update time.
  /// \param[in] _response The system query response data.
  public: void OnUpdateTime(SystemQueryResponse &_response);
};

//////////////////////////////////////////////////
PhysicsSystem::PhysicsSystem()
  : System("Physics"), dataPtr(new PhysicsSystemPrivate)
{
}

//////////////////////////////////////////////////
PhysicsSystem::~PhysicsSystem()
{
}

//////////////////////////////////////////////////
void PhysicsSystem::Init(EntityQueryRegistrar &_registrar)
{
  {
    /// \todo(nkoenig) support curly-bracket initialization of EntityQuery.
    EntityQuery query;
    query.AddComponentType(
        EntityComponentManager::ComponentType<PoseComponent>());

    _registrar.Register(query,
        std::bind(&PhysicsSystemPrivate::OnUpdate, this->dataPtr.get(),
          std::placeholders::_1));
  }

  {
    EntityQuery query;
    query.AddComponentType(
        EntityComponentManager::ComponentType<WorldStatisticsComponent>());
    _registrar.Register(query,
        std::bind(&PhysicsSystemPrivate::OnUpdateTime, this->dataPtr.get(),
          std::placeholders::_1));
  }
}

//////////////////////////////////////////////////
void PhysicsSystemPrivate::OnUpdateTime(SystemQueryResponse &_response)
{
  auto *worldStats =
    _response.EntityComponentMgr().ComponentMutable<WorldStatisticsComponent>(
      *_response.Query().Entities().begin());
  worldStats->AddSimTime(10ms);
  worldStats->AddIterations(1u);
}

//////////////////////////////////////////////////
void PhysicsSystemPrivate::OnUpdate(SystemQueryResponse &/*_response*/)
{
  // Sleep for some amount of time to simulate the computation needed to
  // update physics.
  std::this_thread::sleep_for(8ms);

  // \todo(nkoenig) AcutallyUpdate dynamics

  // \todo(nkoenig) Update collisions

  // \todo(nkoenig) Update entity pose information.
}
