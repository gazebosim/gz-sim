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
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/TestSystem.hh"
#include "ignition/gazebo/Entity.hh"
#include "ServerPrivate.hh"

using namespace ignition::gazebo;

/////////////////////////////////////////////////
Server::Server()
  : dataPtr(new ServerPrivate)
{
  // \todo(nkoenig) Remove this once we can dynamically load systems.
  this->dataPtr->systems.push_back(
      std::unique_ptr<System>(new TestSystem));

  // This is the scene service
  this->dataPtr->node.Advertise("/ign/gazebo/scene",
      &ServerPrivate::SceneService, this->dataPtr.get());
}

/////////////////////////////////////////////////
Server::~Server()
{
}

/////////////////////////////////////////////////
void Server::Run(const uint64_t _iterations, const bool _blocking)
{
  if (_blocking)
  {
    this->dataPtr->Run(_iterations);
  }
  else
  {
    this->dataPtr->runThread =
      std::thread(&ServerPrivate::Run, this->dataPtr.get(), _iterations);
  }
}

/////////////////////////////////////////////////
bool Server::Running() const
{
  return this->dataPtr->running;
}

/////////////////////////////////////////////////
uint64_t Server::IterationCount() const
{
  return this->dataPtr->iterations;
}
