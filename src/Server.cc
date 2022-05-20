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

using namespace ignition;
using namespace gazebo;

/// \brief This struct provides access to the default world.
struct DefaultWorld
{
  /// \brief Get the default world as a string.
  /// Plugins will be loaded from the server.config file.
  /// \return An SDF string that contains the default world.
  public: static std::string &World(std::string _worldName="default")
  {
    static std::string world = std::string("<?xml version='1.0'?>"
      "<sdf version='1.6'>"
        "<world name='" + _worldName + "'>") +
        "</world>"
      "</sdf>";

    return world;
  }
};

/////////////////////////////////////////////////
Server::Server(const ServerConfig &_config)
  : dataPtr(new ServerPrivate)
{
  this->dataPtr->config = _config;
  this->Init();
}

/////////////////////////////////////////////////
Server::Server(bool _downloadInParallel, const ServerConfig &_config)
  : dataPtr(new ServerPrivate)
{
  this->dataPtr->config = _config;
  this->dataPtr->downloadInParallel = _downloadInParallel;
  this->Init();
}

/////////////////////////////////////////////////
bool Server::DownloadModels()
{
  sdf::Root root;
  sdf::Errors errors;
  std::cerr << "this->dataPtr->config.Source() "
            << static_cast<int>(this->dataPtr->config.Source()) << '\n';
  switch (this->dataPtr->config.Source())
  {
    // Load a world if specified. Check SDF string first, then SDF file
    case ServerConfig::SourceType::kSdfRoot:
    {
      this->dataPtr->sdfRoot = this->dataPtr->config.SdfRoot()->Clone();
      ignmsg << "Loading SDF world from SDF DOM.\n";
      break;
    }

    case ServerConfig::SourceType::kSdfString:
    {
      std::string msg = "Loading SDF string. ";
      if (this->dataPtr->config.SdfFile().empty())
      {
        msg += "File path not available.\n";
      }
      else
      {
        msg += "File path [" + this->dataPtr->config.SdfFile() + "].\n";
      }
      ignmsg <<  msg;
      errors = root.LoadSdfString(
        this->dataPtr->config.SdfString());
      this->dataPtr->sdfRoot = root.Clone();
      break;
    }

    case ServerConfig::SourceType::kSdfFile:
    {
      std::string filePath = resolveSdfWorldFile(this->dataPtr->config.SdfFile(),
          this->dataPtr->config.ResourceCache());

      if (filePath.empty())
      {
        ignerr << "Failed to find world ["
               << this->dataPtr->config.SdfFile() << "]"
               << std::endl;
        return false;
      }

      ignmsg << "Loading SDF world file[" << filePath << "].\n";

      while (this->dataPtr->downloadInParallel &&
             this->dataPtr->simRunners.size() == 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      errors = root.Load(filePath);
      this->dataPtr->sdfRoot = root.Clone();

      if (!this->dataPtr->downloadInParallel)
        return true;

      if (this->dataPtr->sdfRoot.WorldCount() == 0)
      {
        ignerr << "There is no world available" << "\n";
        return false;
      }

      while (this->dataPtr->simRunners.size() == 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      for (std::unique_ptr<SimulationRunner> &runner :
           this->dataPtr->simRunners)
      {
        for (size_t j = 0; j < this->dataPtr->sdfRoot.WorldCount(); ++j)
        {
            runner->AddWorld(this->dataPtr->sdfRoot.WorldByIndex(j));
        }
        runner->SetFetchedAllIncludes(true);
      }
      ignmsg << "Download models in parallel has finished. "
             << "Now you can start the simulation" << std::endl;
      break;
    }

    case ServerConfig::SourceType::kNone:
    default:
    {
      ignmsg << "Loading default world.\n";
      // Load an empty world.
      /// \todo(nkoenig) Add a "AddWorld" function to sdf::Root.
      errors = root.LoadSdfString(DefaultWorld::World());
      this->dataPtr->sdfRoot = root.Clone();
      break;
    }
  }

  if (!errors.empty())
  {
    for (auto &err : errors)
      ignerr << err << "\n";
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void Server::Init()
{
  // Configure the fuel client
  fuel_tools::ClientConfig config;
  if (!this->dataPtr->config.ResourceCache().empty())
    config.SetCacheLocation(this->dataPtr->config.ResourceCache());
  this->dataPtr->fuelClient = std::make_unique<fuel_tools::FuelClient>(config);

  // Configure SDF to fetch assets from ignition fuel.
  sdf::setFindCallback(std::bind(&ServerPrivate::FetchResource,
        this->dataPtr.get(), std::placeholders::_1));
  common::addFindFileURICallback(std::bind(&ServerPrivate::FetchResourceUri,
      this->dataPtr.get(), std::placeholders::_1));

  addResourcePaths();

  sdf::Errors errors;

  if (this->dataPtr->downloadInParallel)
  {
    this->dataPtr->downloadModelsThread = std::thread([&]()
    {
      this->DownloadModels();
    });

    ignmsg << "Loading default world.\n";
    // Load an empty world.
    /// \todo(nkoenig) Add a "AddWorld" function to sdf::Root.
    // std::string filePath = resolveSdfWorldFile(this->dataPtr->config.SdfFile(),
    //     this->dataPtr->config.ResourceCache());
    //
    // if (filePath.empty())
    // {
    //   ignerr << "Failed to find world ["
    //          << this->dataPtr->config.SdfFile() << "]"
    //          << std::endl;
    //   return;
    // }

    common::SystemPaths systemPaths;

    // Worlds from environment variable
    systemPaths.SetFilePathEnv(kResourcePathEnv);

    // Worlds installed with ign-gazebo
    systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);

    std::string filePath = systemPaths.FindFile(this->dataPtr->config.SdfFile());

    ignerr << "filePath " << filePath << std::endl;

    std::string worldName;
    auto errors2 = this->dataPtr->sdfRoot.GetWorldName(filePath, worldName);

    errors = this->dataPtr->sdfRoot.LoadSdfString(
      DefaultWorld::World(worldName));
  }
  else
  {
    if (!this->DownloadModels())
    {
      return;
    }
  }

  if (!errors.empty())
  {
    for (auto &err : errors)
      ignerr << err << "\n";
    return;
  }

  // Add record plugin
  if (this->dataPtr->config.UseLogRecord())
  {
    this->dataPtr->AddRecordPlugin(this->dataPtr->config);
  }

  this->dataPtr->CreateEntities();

  // Set the desired update period, this will override the desired RTF given in
  // the world file which was parsed by CreateEntities.
  if (this->dataPtr->config.UpdatePeriod())
  {
    this->SetUpdatePeriod(this->dataPtr->config.UpdatePeriod().value());
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
    ignerr << "Cannot add system while the server is runnnng.\n";
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
    ignerr << "Cannot add system while the server is runnnng.\n";
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
