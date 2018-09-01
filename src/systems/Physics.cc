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
#include <ignition/plugin/Loader.hh>
#include <ignition/plugin/PluginPtr.hh>
#include <ignition/physics/RequestEngine.hh>

#include "ignition/gazebo/systems/Physics.hh"

#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

#include "ignition/gazebo/EntityComponentManager.hh"
#include "ignition/gazebo/EntityQuery.hh"
#include "ignition/gazebo/SystemQueryResponse.hh"

#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/components/Collision.hh"
#include "ignition/gazebo/components/Geometry.hh"
#include "ignition/gazebo/components/WorldStatistics.hh"

using namespace ignition::gazebo::systems;
namespace components = ignition::gazebo::components;

// Private data class.
class ignition::gazebo::systems::PhysicsPrivate
{
};

//////////////////////////////////////////////////
Physics::Physics() : System(), dataPtr(new PhysicsPrivate)
{
}

//////////////////////////////////////////////////
Physics::~Physics()
{
}

//////////////////////////////////////////////////
void Physics::Init()
{
}

//////////////////////////////////////////////////
void Physics::EntityAdded(const Entity &_entity,
                          const EntityComponentManager &_ecm)
{
  (void)_entity;
  (void)_ecm;
}

//////////////////////////////////////////////////
void Physics::EntityRemoved(const Entity &_entity,
                            const EntityComponentManager &_ecm)
{
  (void)_entity;
  (void)_ecm;
}

void Physics::PreUpdate(const ignition::common::Time &_dt,
                        const EntityComponentManager &_ecm)
{
  (void)_dt;
  (void)_ecm;
}

void Physics::Update(const ignition::common::Time &_dt,
                     EntityComponentManager &_ecm)
{
  (void)_dt;
  (void)_ecm;
}

void Physics::PostUpdate(const ignition::common::Time &_dt,
                         const EntityComponentManager &_ecm)
{
  (void)_dt;
  (void)_ecm;
}

IGNITION_ADD_PLUGIN(ignition::gazebo::systems::Physics,
                    ignition::gazebo::System)
