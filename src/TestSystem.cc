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
#include "ignition/gazebo/TestSystem.hh"

#include <vector>
#include <iostream>
#include "ignition/gazebo/Entity.hh"

using namespace ignition::gazebo;

/// \brief Private data.
class ignition::gazebo::TestSystemPrivate
{
  // \todo(nkoenig): Need to make sure that these are data-aligned.
  public: std::vector<Entity> entities;
};

//////////////////////////////////////////////////
TestSystem::TestSystem()
  : System(), dataPtr(new TestSystemPrivate)
{
}

//////////////////////////////////////////////////
TestSystem::~TestSystem()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

/////////////////////////////////////////////////
void TestSystem::EntityCreated(const Entity &_entity)
{
  // The test system adds all entities, for now.
  this->dataPtr->entities.push_back(_entity);
}

//////////////////////////////////////////////////
bool TestSystem::Update()
{
  // Process all entities...just output some information.
  for (const auto &entity : this->dataPtr->entities)
    std::cout << entity.Id() << std::endl;

  return true;
}
