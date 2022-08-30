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

#include <numeric>

#include <gz/common/SystemPaths.hh>
#include <gz/fuel_tools/Interface.hh>
#include <gz/fuel_tools/ClientConfig.hh>
#include <sdf/Root.hh>
#include <sdf/Error.hh>

#include "gz/sim/config.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/Util.hh"

#include "ServerPrivate.hh"
#include "SimulationRunner.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
Server::Server(const ServerConfig &_config)
  : dataPtr(new ServerPrivate)
{
  this->dataPtr->config = _config;

  // Configure the fuel client
  fuel_tools::ClientConfig config;
  if (!_config.ResourceCache().empty())
    config.SetCacheLocation(_config.ResourceCache());
  this->dataPtr->fuelClient = std::make_unique<fuel_tools::FuelClient>(config);

  // Turn on/off downloads based on the WaitForAssets flag. If WaitForAssets
  // is true, then simulation assets should be downloaded when the SDF
  // file/string is parsed.
  this->dataPtr->enableDownload = _config.WaitForAssets();

  // Configure SDF to fetch assets from Gazebo Fuel.
  sdf::setFindCallback(std::bind(&ServerPrivate::FetchResource,
        this->dataPtr.get(), std::placeholders::_1));
  common::addFindFileURICallback(std::bind(&ServerPrivate::FetchResourceUri,
      this->dataPtr.get(), std::placeholders::_1));

  addResourcePaths();

  std::string outputMsgs;

  // Ignore the sdf::Errors returned by this function. The errors will be
  // displayed later in the downloadThread.
  //
  // If `enableDownload` is true at this point, then simulation assets will
  // be downloaded. If false, simulation assets will be downloaded in
  // a thread later in this function.
  sdf::Errors errors = this->dataPtr->LoadSdfRootHelper(_config,
      this->dataPtr->sdfRoot, outputMsgs);
  gzmsg << outputMsgs;

  // Exit here if errors were found while parsing SDF and we were also
  // waiting for assets to download.
  if (_config.WaitForAssets() && !errors.empty())
  {
    for (auto &err : errors)
      gzerr << err << "\n";
    return;
  }

  // Add record plugin
  if (_config.UseLogRecord())
  {
    this->dataPtr->AddRecordPlugin(_config);
  }

  // Remove all the models, lights, and actors from the primary sdfRoot object
  // so that they can be downloaded and added to simulation in the background.
  // Do this before the `CreateEntities` function call.
  if (!_config.WaitForAssets())
  {
    for (uint64_t i = 0; i < this->dataPtr->sdfRoot.WorldCount(); ++i)
    {
      this->dataPtr->sdfRoot.WorldByIndex(i)->ClearModels();
      this->dataPtr->sdfRoot.WorldByIndex(i)->ClearActors();
      this->dataPtr->sdfRoot.WorldByIndex(i)->ClearLights();
    }
  }

  this->dataPtr->CreateEntities();

  if (!_config.WaitForAssets())
  {
    // Remove all the models from the primary sdfRoot object so that they can
    // be downloaded and added to simulation in the background.
    for (uint64_t i = 0; i < this->dataPtr->sdfRoot.WorldCount(); ++i)
    {
      this->dataPtr->sdfRoot.WorldByIndex(i)->ClearModels();
      this->dataPtr->sdfRoot.WorldByIndex(i)->ClearActors();
      this->dataPtr->sdfRoot.WorldByIndex(i)->ClearLights();
    }

    // Turn on downloads.
    this->dataPtr->enableDownload = true;

    // Force each simulation runner to remain paused until the downloads are
    // complete.
    for (auto &runner : this->dataPtr->simRunners)
    {
      runner->SetForcedPause(true);
    }

    // Download models in a separate thread.
    this->dataPtr->downloadThread = std::thread([&]()
    {
      // Reload the SDF root, which will cause the models to download.
      sdf::Root localRoot;
      std::string ignoreMessages;
      sdf::Errors localErrors = this->dataPtr->LoadSdfRootHelper(_config,
          localRoot, ignoreMessages);

      // Output any errors.
      if (!localErrors.empty())
      {
        for (auto &err : localErrors)
          gzerr << err << "\n";
      }
      else
      {
        // Add the models back into the worlds.
        for (auto &runner : this->dataPtr->simRunners)
        {
          std::string worldName = runner->WorldSdf().Name();
          const sdf::World *world = localRoot.WorldByName(worldName);
          if (world)
          {
            for (uint64_t i = 0; i < world->ActorCount(); ++i)
              runner->CreateEntity(*world->ActorByIndex(i));
            for (uint64_t i = 0; i < world->LightCount(); ++i)
              runner->CreateEntity(*world->LightByIndex(i));
            for (uint64_t i = 0; i < world->ModelCount(); ++i)
              runner->CreateEntity(*world->ModelByIndex(i));
          }
          else
          {
            gzerr << "Unable to find world with name[" << worldName << "]. "
              << "Downloaded models may not appear.\n";
          }

          // Allow the runner to resume normal operations.
          runner->SetForcedPause(false);
        }
      }
    });
  }

  // Set the desired update period, this will override the desired RTF given in
  // the world file which was parsed by CreateEntities.
  if (_config.UpdatePeriod())
  {
    this->SetUpdatePeriod(_config.UpdatePeriod().value());
  }

  // Establish publishers and subscribers.
  this->dataPtr->SetupTransport();


}

/////////////////////////////////////////////////
Server::~Server() = default;

/////////////////////////////////////////////////
bool Server::Run(const bool _blocking, const uint64_t _iterations,
    const bool _paused)
{
  // Set the initial pause state of each simulation runner.
  for (std::unique_ptr<SimulationRunner> &runner : this->dataPtr->simRunners)
    runner->SetPaused(_paused);

  // Check the current state, and return early if preconditions are not met.
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->runMutex);
    if (!this->dataPtr->sigHandler.Initialized())
    {
      gzerr << "Signal handlers were not created. The server won't run.\n";
      return false;
    }

    // Do not allow running more than once.
    if (this->dataPtr->running)
    {
      gzwarn << "The server is already runnnng.\n";
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
bool Server::RunOnce(const bool _paused)
{
  if (_paused)
  {
    for (auto &runner : this->dataPtr->simRunners)
      runner->SetNextStepAsBlockingPaused(true);
  }

  return this->Run(true, 1, _paused);
}

/////////////////////////////////////////////////
void Server::SetUpdatePeriod(
    const std::chrono::steady_clock::duration &_updatePeriod,
    const unsigned int _worldIndex)
{
  if (_worldIndex < this->dataPtr->simRunners.size())
    this->dataPtr->simRunners[_worldIndex]->SetUpdatePeriod(_updatePeriod);
}

//////////////////////////////////////////////////
bool Server::Running() const
{
  return this->dataPtr->running;
}

//////////////////////////////////////////////////
std::optional<bool> Server::Running(const unsigned int _worldIndex) const
{
  if (_worldIndex < this->dataPtr->simRunners.size())
    return this->dataPtr->simRunners[_worldIndex]->Running();
  return std::nullopt;
}

//////////////////////////////////////////////////
bool Server::SetPaused(const bool _paused,
    const unsigned int _worldIndex) const
{
  if (_worldIndex < this->dataPtr->simRunners.size())
  {
    this->dataPtr->simRunners[_worldIndex]->SetPaused(_paused);
    return true;
  }

  return false;
}

//////////////////////////////////////////////////
std::optional<bool> Server::Paused(const unsigned int _worldIndex) const
{
  if (_worldIndex < this->dataPtr->simRunners.size())
    return this->dataPtr->simRunners[_worldIndex]->Paused();
  return std::nullopt;
}

//////////////////////////////////////////////////
std::optional<uint64_t> Server::IterationCount(
    const unsigned int _worldIndex) const
{
  if (_worldIndex < this->dataPtr->simRunners.size())
    return this->dataPtr->simRunners[_worldIndex]->IterationCount();
  return std::nullopt;
}

//////////////////////////////////////////////////
std::optional<size_t> Server::EntityCount(const unsigned int _worldIndex) const
{
  if (_worldIndex < this->dataPtr->simRunners.size())
    return this->dataPtr->simRunners[_worldIndex]->EntityCount();
  return std::nullopt;
}

//////////////////////////////////////////////////
std::optional<size_t> Server::SystemCount(const unsigned int _worldIndex) const
{
  if (_worldIndex < this->dataPtr->simRunners.size())
    return this->dataPtr->simRunners[_worldIndex]->SystemCount();
  return std::nullopt;
}

//////////////////////////////////////////////////
std::optional<bool> Server::AddSystem(const SystemPluginPtr &_system,
                                      const unsigned int _worldIndex)
{
  // Check the current state, and return early if preconditions are not met.
  std::lock_guard<std::mutex> lock(this->dataPtr->runMutex);
  // Do not allow running more than once.
  if (this->dataPtr->running)
  {
    gzerr << "Cannot add system while the server is runnnng.\n";
    return false;
  }

  if (_worldIndex < this->dataPtr->simRunners.size())
  {
    this->dataPtr->simRunners[_worldIndex]->AddSystem(_system);
    return true;
  }

  return std::nullopt;
}

//////////////////////////////////////////////////
std::optional<bool> Server::AddSystem(const std::shared_ptr<System> &_system,
                                      const unsigned int _worldIndex)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->runMutex);
  if (this->dataPtr->running)
  {
    gzerr << "Cannot add system while the server is runnnng.\n";
    return false;
  }

  if (_worldIndex < this->dataPtr->simRunners.size())
  {
    this->dataPtr->simRunners[_worldIndex]->AddSystem(_system);
    return true;
  }

  return std::nullopt;
}

//////////////////////////////////////////////////
bool Server::HasEntity(const std::string &_name,
                       const unsigned int _worldIndex) const
{
  if (_worldIndex < this->dataPtr->simRunners.size())
    return this->dataPtr->simRunners[_worldIndex]->HasEntity(_name);

  return false;
}

//////////////////////////////////////////////////
std::optional<Entity> Server::EntityByName(const std::string &_name,
    const unsigned int _worldIndex) const
{
  if (_worldIndex < this->dataPtr->simRunners.size())
    return this->dataPtr->simRunners[_worldIndex]->EntityByName(_name);

  return std::nullopt;
}

//////////////////////////////////////////////////
bool Server::RequestRemoveEntity(const std::string &_name,
    bool _recursive, const unsigned int _worldIndex)
{
  if (_worldIndex < this->dataPtr->simRunners.size())
  {
    return this->dataPtr->simRunners[_worldIndex]->RequestRemoveEntity(_name,
        _recursive);
  }

  return false;
}

//////////////////////////////////////////////////
bool Server::RequestRemoveEntity(const Entity _entity,
    bool _recursive, const unsigned int _worldIndex)
{
  if (_worldIndex < this->dataPtr->simRunners.size())
  {
    return this->dataPtr->simRunners[_worldIndex]->RequestRemoveEntity(_entity,
        _recursive);
  }

  return false;
}

//////////////////////////////////////////////////
void Server::Stop()
{
  this->dataPtr->Stop();
}
