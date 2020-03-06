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
#include "ServerPrivate.hh"

#include <tinyxml2.h>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <ignition/common/Console.hh>

#include <ignition/fuel_tools/Interface.hh>

#include <ignition/gui/Application.hh>

#include "SimulationRunner.hh"

using namespace ignition;
using namespace gazebo;

/// \brief This struct provides access to the record plugin SDF string
struct LoggingPlugin
{
  /// \brief Get the record plugin file name as a string
  /// \return A string that contains the record plugin file name.
  public: static std::string &LoggingPluginFileName()
  {
    static std::string recordPluginFileName =
      std::string("libignition-gazebo") +
      IGNITION_GAZEBO_MAJOR_VERSION_STR + "-log-system.so";
    return recordPluginFileName;
  }

  /// \brief Get the record plugin file name suffix as a string, without
  /// major version number.
  /// \return A string that contains the record plugin suffix.
  public: static std::string &LoggingPluginSuffix()
  {
    static std::string recordPluginSuffix = "-log-system.so";
    return recordPluginSuffix;
  }

  /// \brief Get the record plugin name as a string
  /// \return A string that contains the record plugin name.
  public: static std::string &RecordPluginName()
  {
    static std::string recordPluginName =
      "ignition::gazebo::systems::LogRecord";
    return recordPluginName;
  }

  /// \brief Get the record plugin as a string
  /// \return An SDF string that contains the record plugin.
  public: static std::string &RecordPluginSdf()
  {
    static std::string recordPlugin =
      std::string("<plugin filename='") + LoggingPluginFileName() +
      "' name='" + RecordPluginName() + "'></plugin>";
    return recordPlugin;
  }

  public: static std::string &PlaybackPluginName()
  {
    static std::string playbackPluginName =
      "ignition::gazebo::systems::LogPlayback";
    return playbackPluginName;
  }
};

//////////////////////////////////////////////////
ServerPrivate::ServerPrivate()
: systemLoader(std::make_shared<SystemLoader>())
{
  // Add the signal handler
  this->sigHandler.AddCallback(
      std::bind(&ServerPrivate::OnSignal, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
ServerPrivate::~ServerPrivate()
{
  this->Stop();
  if (this->runThread.joinable())
  {
    this->runThread.join();
  }
}

//////////////////////////////////////////////////
void ServerPrivate::OnSignal(int _sig)
{
  igndbg << "Server received signal[" << _sig  << "]\n";
  this->Stop();
}

/////////////////////////////////////////////////
void ServerPrivate::Stop()
{
  this->running = false;
  for (std::unique_ptr<SimulationRunner> &runner : this->simRunners)
  {
    runner->Stop();
  }
}

/////////////////////////////////////////////////
bool ServerPrivate::Run(const uint64_t _iterations,
    std::optional<std::condition_variable *> _cond)
{
  this->runMutex.lock();
  this->running = true;
  if (_cond)
    _cond.value()->notify_all();
  this->runMutex.unlock();

  bool result = true;

  if (this->config.UseDistributedSimulation())
  {
    // Check for network ready (needed for distributed sim)
    bool networkReady = false;
    bool receivedStop = false;
    while (this->running && !networkReady && !receivedStop)
    {
      networkReady = true;
      for (const auto &runner : this->simRunners)
      {
        receivedStop |= runner->StopReceived();
        networkReady &= runner->Ready();
      }

      if (!networkReady && !receivedStop)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    if (!networkReady || receivedStop)
    {
      ignerr << "Failed to start network, simulation terminating" << std::endl;
      return false;
    }
  }

  // Minor performance tweak. In many situations there will only be one
  // simulation runner, and we can avoid using the thread pool.
  if (this->simRunners.size() == 1)
  {
    result = this->simRunners[0]->Run(_iterations);
  }
  else
  {
    for (std::unique_ptr<SimulationRunner> &runner : this->simRunners)
    {
      this->workerPool.AddWork([&runner, &_iterations] ()
        {
          runner->Run(_iterations);
        });
    }

    // Wait for the runner to complete.
    result = this->workerPool.WaitForResults();
  }

  this->running = false;
  return result;
}

//////////////////////////////////////////////////
void ServerPrivate::AddRecordPlugin(const ServerConfig &_config)
{
  const sdf::World *sdfWorld = this->sdfRoot.WorldByIndex(0);
  sdf::ElementPtr worldElem = sdfWorld->Element();

  // Check if there is already a record plugin specified
  if (worldElem->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = worldElem->GetElement("plugin");
    while (pluginElem != nullptr)
    {
      sdf::ParamPtr pluginName = pluginElem->GetAttribute("name");
      sdf::ParamPtr pluginFileName = pluginElem->GetAttribute("filename");

      if (pluginName != nullptr && pluginFileName != nullptr)
      {
        // Found a logging plugin
        if (pluginFileName->GetAsString().find(
            LoggingPlugin::LoggingPluginSuffix()) != std::string::npos)
        {
          // If record plugin already specified in SDF, no need to add
          if (pluginName->GetAsString() == LoggingPlugin::RecordPluginName())
          {
            // If a custom path is passed in through command line, overwrite
            //   the path in SDF
            if (!_config.LogRecordPath().empty())
            {
              pluginElem->AddAttribute("path", "string", "", false);
              sdf::ParamPtr recordParam = pluginElem->GetAttribute("path");
              recordParam->SetFromString(_config.LogRecordPath());
            }
            return;
          }

          // If playback plugin also specified, do not add a record plugin
          if (pluginName->GetAsString() == LoggingPlugin::PlaybackPluginName())
          {
            ignwarn << "Both record and playback are specified. "
              << "Ignoring record.\n";
            return;
          }
        }
      }

      pluginElem = pluginElem->GetNextElement();
    }
  }

  // Add record plugin
  sdf::ElementPtr recordElem = worldElem->AddElement("plugin");
  sdf::ParamPtr recordName = recordElem->GetAttribute("name");
  recordName->SetFromString(LoggingPlugin::RecordPluginName());
  sdf::ParamPtr recordFileName = recordElem->GetAttribute("filename");
  recordFileName->SetFromString(LoggingPlugin::LoggingPluginFileName());

  // Add custom record path
  if (!_config.LogRecordPath().empty())
  {
    recordElem->AddAttribute("path", "string", "", false);
    sdf::ParamPtr recordParam = recordElem->GetAttribute("path");
    recordParam->SetFromString(_config.LogRecordPath());
  }
}

//////////////////////////////////////////////////
void ServerPrivate::CreateEntities()
{
  // Create a simulation runner for each world.
  for (uint64_t worldIndex = 0; worldIndex <
       this->sdfRoot.WorldCount(); ++worldIndex)
  {
    auto world = this->sdfRoot.WorldByIndex(worldIndex);

    {
      std::lock_guard<std::mutex> lock(this->worldsMutex);
      this->worldNames.push_back(world->Name());
    }

    this->simRunners.push_back(std::make_unique<SimulationRunner>(
        world, this->systemLoader, this->config));
  }
}

//////////////////////////////////////////////////
void ServerPrivate::SetupTransport()
{
  // Advertise available worlds.
  this->node.Advertise("/gazebo/worlds", &ServerPrivate::WorldsService, this);
}

//////////////////////////////////////////////////
bool ServerPrivate::WorldsService(ignition::msgs::StringMsg_V &_res)
{
  std::lock_guard<std::mutex> lock(this->worldsMutex);

  _res.Clear();

  for (const auto &name : this->worldNames)
  {
    _res.add_data(name);
  }

  return true;
}

//////////////////////////////////////////////////
std::string ServerPrivate::FetchResource(const std::string &_uri)
{
  return fuel_tools::fetchResourceWithClient(_uri, *this->fuelClient.get());
}

//////////////////////////////////////////////////
std::string ServerPrivate::FetchResourceUri(const common::URI &_uri)
{
  return this->FetchResource(_uri.Str());
}
