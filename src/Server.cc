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

#include <sdf/Root.hh>
#include "ServerPrivate.hh"

using namespace ignition::gazebo;

/////////////////////////////////////////////////
Server::Server()
  : dataPtr(new ServerPrivate)
{
}

/////////////////////////////////////////////////
Server::Server(const ServerConfig &_config)
  : dataPtr(new ServerPrivate)
{
  if (!_config.SdfFile().empty())
  {
    sdf::Root root;
    root.Load(_config.SdfFile());

    this->dataPtr->CreateEntities(root);
  }

  this->dataPtr->InitSystems();
}

/////////////////////////////////////////////////
Server::~Server()
{
}

/////////////////////////////////////////////////
bool Server::Run(const bool _blocking, const uint64_t _iterations)
{
  // Check the current state, and return early if preconditions are not met.
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->runMutex);
    if (!this->dataPtr->sigHandler.Initialized())
    {
      ignerr << "Signal handlers were not created. The server won't run.\n";
      return false;
    }

    // Do not allow running more than once.
    if (this->dataPtr->running)
    {
      ignwarn << "The server is already runnnng.\n";
      return false;
    }
  }

  if (_blocking)
    return this->dataPtr->Run(_iterations);

  // Make sure two threads are not created
  std::unique_lock<std::mutex> lock(this->dataPtr->runMutex);
  if (this->dataPtr->runThread.get_id() == std::thread::id())
  {
    std::condition_variable cond;
    this->dataPtr->runThread =
      std::thread(&ServerPrivate::Run, this->dataPtr.get(), _iterations, &cond);

    // Wait for the thread to start. We do this to guarantee that the
    // running variable gets updated before this function returns. With
    // a small number of iterations it is possible that the run thread
    // successfuly completes before this function returns.
    cond.wait(lock);
    return true;
  }

  return false;
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

/////////////////////////////////////////////////
size_t Server::EntityCount() const
{
  return this->dataPtr->entityCompMgr->EntityCount();
}

/////////////////////////////////////////////////
size_t Server::SystemCount() const
{
  return this->dataPtr->systems.size();
}

/////////////////////////////////////////////////
EntityComponentManager &Server::EntityComponentMgr() const
{
  return *(this->dataPtr->entityCompMgr.get());
}

/////////////////////////////////////////////////
void Server::SetUpdatePeriod(
    const std::chrono::steady_clock::duration &_updatePeriod)
{
  this->dataPtr->updatePeriod = _updatePeriod;
}
