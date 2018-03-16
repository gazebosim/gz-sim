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
#include <vector>
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/System.hh"
#include "ignition/gazebo/PhysicsSystem.hh"
#include "ignition/gazebo/Entity.hh"

using namespace ignition::gazebo;

class ignition::gazebo::ServerPrivate
{
  public: std::vector<System *> systems;
  public: std::vector<Entity> entities;
};

/////////////////////////////////////////////////
Server::Server()
  : dataPtr(new ServerPrivate)
{
  this->dataPtr->systems.push_back(new PhysicsSystem());
}

/////////////////////////////////////////////////
Entity Server::CreateEntity(const sdf::Model & /*_model*/)
{
  Entity entity;

  // Notify systems that an entity has been created.
  // \todo: We could move this into batch style process that happens
  // once before systems are updated.
  for (System *system : this->dataPtr->systems)
  {
    system->EntityCreated(entity);
  }

  this->dataPtr->entities.push_back(entity);
  return this->dataPtr->entities.back();
}

/////////////////////////////////////////////////
int Server::Step(const unsigned int _iterations)
{
  // For each iteration ...
  for (unsigned int iter = 0; iter < _iterations; ++iter)
  {
    // Update systems
    for (System *system : this->dataPtr->systems)
    {
      system->Update();
    }
  }

  return 0;
}
