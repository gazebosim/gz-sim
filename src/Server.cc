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

#include <ignition/common/SystemPaths.hh>
#include <ignition/fuel_tools/Interface.hh>
#include <ignition/fuel_tools/ClientConfig.hh>
#include <sdf/Root.hh>
#include <sdf/Error.hh>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/Util.hh"

#include "ServerPrivate.hh"
#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
// Getting the first .sdf file in the path
std::string findFuelResourceSdf(const std::string &_path)
{
  if (!common::exists(_path))
    return "";

  for (common::DirIter file(_path); file != common::DirIter(); ++file)
  {
    std::string current(*file);
    if (!common::isFile(current))
      continue;

    auto fileName = common::basename(current);
    auto fileExtensionIndex = fileName.rfind(".");
    auto fileExtension = fileName.substr(fileExtensionIndex + 1);

    if (fileExtension == "sdf")
    {
      return current;
    }
  }
  return "";
}

/// \brief This struct provides access to the default world.
struct DefaultWorld
{
  /// \brief Get the default plugins as a string.
  /// \return An SDF string that contains the default plugins.
  public: static std::string &DefaultPlugins(const ServerConfig &_config)
  {
    std::vector<std::string> pluginsV = {
      {
        std::string("<plugin filename='libignition-gazebo") +
        IGNITION_GAZEBO_MAJOR_VERSION_STR + "-scene-broadcaster-system.so' "
        "name='ignition::gazebo::systems::SceneBroadcaster'></plugin>"
      }};

    // The set of default gazebo plugins.
    if (_config.LogPlaybackPath().empty())
    {
      pluginsV.push_back(std::string("<plugin filename='libignition-gazebo") +
        IGNITION_GAZEBO_MAJOR_VERSION_STR + "-physics-system.so' "
        "name='ignition::gazebo::systems::Physics'></plugin>");
      pluginsV.push_back(std::string("<plugin filename='libignition-gazebo") +
        IGNITION_GAZEBO_MAJOR_VERSION_STR + "-user-commands-system.so' " +
        "name='ignition::gazebo::systems::UserCommands'></plugin>");
    }

    // Playback plugin
    else
    {
      pluginsV.push_back(std::string("<plugin filename='libignition-gazebo") +
        IGNITION_GAZEBO_MAJOR_VERSION_STR + "-log-system.so' "
        "name='ignition::gazebo::systems::LogPlayback'><path>" +
        _config.LogPlaybackPath() + "</path></plugin>");
    }

    static std::string plugins = std::accumulate(pluginsV.begin(),
      pluginsV.end(), std::string(""));
    return plugins;
  }

  /// \brief Get the default world as a string.
  /// \return An SDF string that contains the default world.
  public: static std::string &World(const ServerConfig &_config)
  {
    static std::string world = std::string("<?xml version='1.0'?>"
      "<sdf version='1.6'>"
        "<world name='default'>") +
          DefaultPlugins(_config) +
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

  // Configure the fuel client
  fuel_tools::ClientConfig config;
  if (!_config.ResourceCache().empty())
    config.SetCacheLocation(_config.ResourceCache());
  this->dataPtr->fuelClient = std::make_unique<fuel_tools::FuelClient>(config);

  // Configure SDF to fetch assets from ignition fuel.
  sdf::setFindCallback(std::bind(&ServerPrivate::FetchResource,
        this->dataPtr.get(), std::placeholders::_1));
  common::addFindFileURICallback(std::bind(&ServerPrivate::FetchResourceUri,
      this->dataPtr.get(), std::placeholders::_1));

  addResourcePaths();

  sdf::Errors errors;

  // Load a world if specified. Check SDF string first, then SDF file
  if (!_config.SdfString().empty())
  {
    std::string msg = "Loading SDF string. ";
    if (_config.SdfFile().empty())
    {
      msg += "File path not available.\n";
    }
    else
    {
      msg += "File path [" + _config.SdfFile() + "].\n";
    }
    ignmsg <<  msg;
    errors = this->dataPtr->sdfRoot.LoadSdfString(_config.SdfString());
  }
  else if (!_config.SdfFile().empty())
  {
    std::string filePath;

    // Check Fuel if it's a URL
    auto sdfUri = common::URI(_config.SdfFile());
    if (sdfUri.Scheme() == "http" || sdfUri.Scheme() == "https")
    {
      std::string fuelCachePath;
      if (this->dataPtr->fuelClient->CachedWorld(common::URI(_config.SdfFile()),
          fuelCachePath))
      {
        filePath = findFuelResourceSdf(fuelCachePath);
      }
      else if (auto result = this->dataPtr->fuelClient->DownloadWorld(
          common::URI(_config.SdfFile()), fuelCachePath))
      {
        filePath = findFuelResourceSdf(fuelCachePath);
      }
      else
      {
        ignwarn << "Fuel couldn't download URL [" << _config.SdfFile()
                << "], error: [" << result.ReadableResult() << "]"
                << std::endl;
      }
    }

    if (filePath.empty())
    {
      common::SystemPaths systemPaths;

      // Worlds from environment variable
      systemPaths.SetFilePathEnv(kResourcePathEnv);

      // Worlds installed with ign-gazebo
      systemPaths.AddFilePaths(IGN_GAZEBO_WORLD_INSTALL_DIR);

      filePath = systemPaths.FindFile(_config.SdfFile());
    }

    if (filePath.empty())
    {
      ignerr << "Failed to find world [" << _config.SdfFile() << "]"
             << std::endl;
      return;
    }

    ignmsg << "Loading SDF world file[" << filePath << "].\n";

    // \todo(nkoenig) Async resource download.
    // This call can block for a long period of time while
    // resources are downloaded. Blocking here causes the GUI to block with
    // a black screen (search for "Async resource download" in
    // 'src/gui_main.cc'.
    errors = this->dataPtr->sdfRoot.Load(filePath);
  }
  else
  {
    ignmsg << "Loading default world.\n";
    // Load an empty world.
    /// \todo(nkoenig) Add a "AddWorld" function to sdf::Root.
    errors = this->dataPtr->sdfRoot.LoadSdfString(DefaultWorld::World(_config));
  }

  if (!errors.empty())
  {
    for (auto &err : errors)
      ignerr << err << "\n";
    return;
  }

  // Add record plugin
  if (_config.UseLogRecord())
  {
    this->dataPtr->AddRecordPlugin(_config);
  }

  this->dataPtr->CreateEntities();

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

