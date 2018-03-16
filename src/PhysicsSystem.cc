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
#include <iostream>
#include "ignition/gazebo/PhysicsSystem.hh"
#include "ignition/gazebo/Entity.hh"

using namespace ignition::gazebo;

class ignition::gazebo::PhysicsSystemPrivate
{
  // \todo: Do we want pointers here?
  public: std::vector<Entity> entities;
};

//////////////////////////////////////////////////
PhysicsSystem::PhysicsSystem()
  : System(), dataPtr(new PhysicsSystemPrivate)
{
}

//////////////////////////////////////////////////
PhysicsSystem::~PhysicsSystem()
{
}

/////////////////////////////////////////////////
void PhysicsSystem::EntityCreated(const Entity &_entity)
{
  // \todo: Add an entity if its components match this system's
  // requirements.
  this->dataPtr->entities.push_back(_entity);
}


//////////////////////////////////////////////////
bool PhysicsSystem::Update()
{
  // Process all entities
  for (const auto &entity : this->dataPtr->entities)
  {
    std::cout << entity.Id() << std::endl;
  }


  // Process entities that match my requirements.
  return true;
}
