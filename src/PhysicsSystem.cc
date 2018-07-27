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
#include "ignition/gazebo/PoseComponentType.hh"
#include "ignition/gazebo/PhysicsSystem.hh"
#include "ignition/gazebo/EntityQuery.hh"

// Private data class.
class ignition::gazebo::PhysicsSystemPrivate
{
};

using namespace ignition::gazebo;

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
void PhysicsSystem::Init(EntityQueryRegistrar &_registrar,
    EntityComponentManager *_ecMgr)
{
  /// \todo(nkoenig) support curly-bracket initialization.
  /// \todo(nkoenig) It would be nice to fix the ComponentMgr shared pointer
  /// issue.
  EntityQuery query;
  query.AddComponentType(PoseComponentType(_ecMgr));
  _registrar.Register(query,
      std::bind(&PhysicsSystem::OnUpdate, this, std::placeholders::_1,
        std::placeholders::_2));
}

//////////////////////////////////////////////////
void PhysicsSystem::OnUpdate(const EntityQuery &_result,
    EntityComponentManager *_ecMgr)
{
  std::cout << "Physics System on update Entities[";
  for (const EntityId &entity : _result.Entities())
  {
    const ignition::math::Pose3d *pose =
      _ecMgr->Component<ignition::math::Pose3d>(entity);
    std::cout << *pose << std::endl;
  }
  std::cout << "]\n";
}
