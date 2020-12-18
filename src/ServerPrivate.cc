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
#include <ignition/common/Util.hh>

#include <ignition/fuel_tools/Interface.hh>

#include <ignition/gui/Application.hh>

#include "ignition/gazebo/Util.hh"
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
      std::string("ignition-gazebo") +
      IGNITION_GAZEBO_MAJOR_VERSION_STR + "-log-system";
    return recordPluginFileName;
  }

  /// \brief Get the record plugin file name suffix as a string, without
  /// major version number.
  /// \return A string that contains the record plugin suffix.
  public: static std::string &LoggingPluginSuffix()
  {
    static std::string recordPluginSuffix = "-log-system";
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
          // If record plugin already specified in SDF, and record flags are
          //   specified on command line, replace SDF parameters with those on
          //   command line. (If none specified on command line, use those in
          //   SDF.)
          if (pluginName->GetAsString() == LoggingPlugin::RecordPluginName())
          {
            std::string recordPath = _config.LogRecordPath();
            std::string cmpPath = _config.LogRecordCompressPath();

            // Set record path
            if (!_config.LogRecordPath().empty())
            {
              bool overwriteSdf = false;
              // If <path> is specified in SDF, check whether to replace it
              if (pluginElem->HasElement("path") &&
                  !pluginElem->Get<std::string>("path").empty())
              {
                // If record path came from command line, overwrite SDF <path>
                if (_config.LogIgnoreSdfPath())
                {
                  overwriteSdf = true;
                }
                // TODO(anyone) In Ignition-D, remove this. <path> will be
                //   permanently ignored in favor of common::ignLogDirectory().
                //   Always overwrite SDF.
                // Otherwise, record path is same as the default timestamp log
                //   path. Take the path in SDF <path>.
                // Deprecated.
                else
                {
                  ignwarn << "--record-path is not specified on command line. "
                    << "<path> is specified in SDF. Will record to <path>. "
                    << "Console will be logged to [" << ignLogDirectory()
                    << "]. Note: In Ignition-D, <path> will be ignored, and "
                    << "all recordings will be written to default console log "
                    << "path if no path is specified on command line.\n";
                  overwriteSdf = false;

                  // Take <path> in SDF
                  recordPath = pluginElem->Get<std::string>("path");

                  // Update path for compressed file to match record path
                  cmpPath = std::string(recordPath);
                  if (!std::string(1, cmpPath.back()).compare(
                    ignition::common::separator("")))
                  {
                    // Remove the separator at end of path
                    cmpPath = cmpPath.substr(0, cmpPath.length() - 1);
                  }
                  cmpPath += ".zip";
                }
              }
              else
              {
                overwriteSdf = true;
              }

              if (overwriteSdf)
              {
                sdf::ElementPtr pathElem = std::make_shared<sdf::Element>();
                pathElem->SetName("path");
                pluginElem->AddElementDescription(pathElem);
                pathElem = pluginElem->GetElement("path");
                pathElem->AddValue("string", "", false, "");
                pathElem->Set<std::string>(recordPath);
              }
            }

            // If resource flag specified on command line, replace in SDF
            if (_config.LogRecordResources())
            {
              sdf::ElementPtr resourceElem = std::make_shared<sdf::Element>();
              resourceElem->SetName("record_resources");
              pluginElem->AddElementDescription(resourceElem);
              resourceElem = pluginElem->GetElement("record_resources");
              resourceElem->AddValue("bool", "false", false, "");
              resourceElem->Set<bool>(_config.LogRecordResources()
                ? true : false);
            }

            // If compress flag specified on command line, replace in SDF
            if (!_config.LogRecordCompressPath().empty())
            {
              sdf::ElementPtr compressElem = std::make_shared<sdf::Element>();
              compressElem->SetName("compress");
              pluginElem->AddElementDescription(compressElem);
              compressElem = pluginElem->GetElement("compress");
              compressElem->AddValue("bool", "false", false, "");
              compressElem->Set<bool>(true);

              sdf::ElementPtr cPathElem = std::make_shared<sdf::Element>();
              cPathElem->SetName("compress_path");
              pluginElem->AddElementDescription(cPathElem);
              cPathElem = pluginElem->GetElement("compress_path");
              cPathElem->AddValue("string", "", false, "");
              cPathElem->Set<std::string>(cmpPath);
            }

            // If record topics specified, add in SDF
            for (const std::string &topic : _config.LogRecordTopics())
            {
              sdf::ElementPtr topicElem = std::make_shared<sdf::Element>();
              topicElem->SetName("record_topic");
              pluginElem->AddElementDescription(topicElem);
              topicElem = pluginElem->AddElement("record_topic");
              topicElem->AddValue("string", "false", false, "");
              topicElem->Set<std::string>(topic);
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
    auto runner = std::make_unique<SimulationRunner>(
        world, this->systemLoader, this->config);
    runner->SetFuelUriMap(this->fuelUriMap);
    this->simRunners.push_back(std::move(runner));
  }
}

//////////////////////////////////////////////////
void ServerPrivate::SetupTransport()
{
  // Advertise available worlds.
  std::string worldsService{"/gazebo/worlds"};
  if (this->node.Advertise(worldsService, &ServerPrivate::WorldsService, this))
  {
    ignmsg << "Serving world names on [" << worldsService << "]" << std::endl;
  }
  else
  {
    ignerr << "Something went wrong, failed to advertise [" << worldsService
           << "]" << std::endl;
  }

  // Resource path management
  std::string addPathService{"/gazebo/resource_paths/add"};
  if (this->node.Advertise(addPathService,
      &ServerPrivate::AddResourcePathsService, this))
  {
    ignmsg << "Resource path add service on [" << addPathService << "]."
           << std::endl;
  }
  else
  {
    ignerr << "Something went wrong, failed to advertise [" << addPathService
           << "]" << std::endl;
  }

  std::string getPathService{"/gazebo/resource_paths/get"};
  if (this->node.Advertise(getPathService,
      &ServerPrivate::ResourcePathsService, this))
  {
    ignmsg << "Resource path get service on [" << addPathService << "]."
           << std::endl;
  }
  else
  {
    ignerr << "Something went wrong, failed to advertise [" << getPathService
           << "]" << std::endl;
  }

  std::string pathTopic{"/gazebo/resource_paths"};
  this->pathPub = this->node.Advertise<msgs::StringMsg_V>(pathTopic);

  if (this->pathPub)
  {
    ignmsg << "Resource paths published on [" << pathTopic << "]."
           << std::endl;
  }
  else
  {
    ignerr << "Something went wrong, failed to advertise [" << pathTopic
           << "]" << std::endl;
  }
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
void ServerPrivate::AddResourcePathsService(
    const ignition::msgs::StringMsg_V &_req)
{
  std::vector<std::string> paths;
  for (int i = 0; i < _req.data_size(); ++i)
  {
    paths.push_back(_req.data(i));
  }
  addResourcePaths(paths);

  // Notify new paths
  msgs::StringMsg_V msg;
  auto gzPaths = resourcePaths();
  for (const auto &path : gzPaths)
  {
    if (!path.empty())
      msg.add_data(path);
  }

  this->pathPub.Publish(msg);
}

//////////////////////////////////////////////////
bool ServerPrivate::ResourcePathsService(
    ignition::msgs::StringMsg_V &_res)
{
  _res.Clear();

  // Update paths
  addResourcePaths();

  // Get paths
  auto gzPaths = resourcePaths();
  for (const auto &path : gzPaths)
  {
    if (!path.empty())
      _res.add_data(path);
  }

  return true;
}

//////////////////////////////////////////////////
std::string ServerPrivate::FetchResource(const std::string &_uri)
{
  auto path =
      fuel_tools::fetchResourceWithClient(_uri, *this->fuelClient.get());

  if (!path.empty())
  {
    for (auto &runner : this->simRunners)
    {
      runner->AddToFuelUriMap(path, _uri);
    }
    fuelUriMap[path] = _uri;
  }
  return path;
}

//////////////////////////////////////////////////
std::string ServerPrivate::FetchResourceUri(const common::URI &_uri)
{
  return this->FetchResource(_uri.Str());
}
