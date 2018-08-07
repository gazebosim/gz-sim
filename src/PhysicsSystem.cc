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

#include "ignition/gazebo/PhysicsSystem.hh"
#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/EntityQuery.hh"

#include "PoseComponent.hh"
#include "WorldStatisticsComponent.hh"

using namespace ignition::gazebo;
using namespace std::chrono_literals;

// Private data class.
class ignition::gazebo::PhysicsSystemPrivate
{
  public: void OnUpdate(const EntityQuery &_result,
                        EntityComponentManager &_ecMgr);

  public: void OnUpdateTime(const EntityQuery &_result,
                            EntityComponentManager &_ecMgr);
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
          std::placeholders::_1, std::placeholders::_2));
  }

  {
    EntityQuery query;
    query.AddComponentType(
        EntityComponentManager::ComponentType<WorldStatisticsComponent>());
    _registrar.Register(query,
        std::bind(&PhysicsSystemPrivate::OnUpdateTime, this->dataPtr.get(),
          std::placeholders::_1, std::placeholders::_2));
  }
}

//////////////////////////////////////////////////
void PhysicsSystemPrivate::OnUpdateTime(const EntityQuery &_result,
    EntityComponentManager &_ecMgr)
{
  auto *worldStats = _ecMgr.ComponentMutable<WorldStatisticsComponent>(
      *_result.Entities().begin());
  worldStats->AddSimTime(10ms);
  worldStats->AddIterations(1u);
}

//////////////////////////////////////////////////
void PhysicsSystemPrivate::OnUpdate(const EntityQuery & /*_result*/,
    EntityComponentManager & /*_ecMgr*/)
{
  // \todo(nkoenig) Update dynamics
  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // \todo(nkoenig) Update collisions

  // Move all the entities.
  // for (const EntityId &entity : _result.Entities())
  // {
  //   // \todo(nkoenig) Support modification of components.
  //   const auto *pose = _ecMgr.Component<PoseComponent>(entity);
  // }
}
