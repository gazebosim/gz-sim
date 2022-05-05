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
  if (this->stopThread && this->stopThread->joinable())
  {
    this->stopThread->join();
  }

  for (auto &t : threadDownloadModels)
  {
    t.join();
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
sdf::ElementPtr GetRecordPluginElem(sdf::Root &_sdfRoot)
{
  sdf::ElementPtr rootElem = _sdfRoot.Element();

  if (rootElem->HasElement("world"))
  {
    sdf::ElementPtr worldElem = rootElem->GetElement("world");

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
            if (pluginName->GetAsString() == LoggingPlugin::RecordPluginName())
            {
              return pluginElem;
            }
          }
        }

        pluginElem = pluginElem->GetNextElement();
      }
    }
  }
  return nullptr;
}

//////////////////////////////////////////////////
void ServerPrivate::AddRecordPlugin(const ServerConfig &_config)
{
  auto recordPluginElem = GetRecordPluginElem(this->sdfRoot);
  bool sdfUseLogRecord = (recordPluginElem != nullptr);

  bool hasRecordPath {false};
  bool hasCompressPath {false};
  bool hasRecordResources {false};
  bool hasRecordTopics {false};

  std::string sdfRecordPath;
  std::string sdfCompressPath;
  bool sdfRecordResources;
  std::vector<std::string> sdfRecordTopics;

  if (sdfUseLogRecord)
  {
    std::tie(sdfRecordPath, hasRecordPath) =
      recordPluginElem->Get<std::string>("path", "");
    std::tie(sdfCompressPath, hasCompressPath) =
      recordPluginElem->Get<std::string>("compress_path", "");
    std::tie(sdfRecordResources, hasRecordResources) =
      recordPluginElem->Get<bool>("record_resources", false);

    hasRecordTopics = recordPluginElem->HasElement("record_topic");
    if (hasRecordTopics)
    {
      sdf::ElementPtr recordTopicElem =
        recordPluginElem->GetElement("record_topic");
      while (recordTopicElem)
      {
        auto topic = recordTopicElem->Get<std::string>();
        sdfRecordTopics.push_back(topic);
      }

      recordTopicElem = recordTopicElem->GetNextElement();
    }

    // Remove from SDF
    recordPluginElem->RemoveFromParent();
    recordPluginElem->Reset();
  }

  // Set the config based on what is in the SDF:
  if (hasRecordPath)
    this->config.SetLogRecordPath(sdfRecordPath);
  if (hasCompressPath)
    this->config.SetLogRecordCompressPath(sdfCompressPath);
  if (hasRecordResources)
    this->config.SetLogRecordResources(sdfRecordResources);

  if (hasRecordTopics)
  {
    this->config.ClearLogRecordTopics();
    for (auto topic : sdfRecordTopics)
    {
      this->config.AddLogRecordTopic(topic);
    }
  }

  if (!_config.LogRecordPath().empty() && hasRecordPath)
  {
    if (hasRecordPath)
    {
      // If record path came from command line, overwrite SDF <path>
      if (_config.LogIgnoreSdfPath())
      {
        this->config.SetLogRecordPath(_config.LogRecordPath());
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

        // In the case that the --compress flag is set, then
        // this field will be populated with just the file extension
        if (_config.LogRecordCompressPath() == ".zip")
        {
          sdfCompressPath = std::string(sdfRecordPath);
          if (!std::string(1, sdfCompressPath.back()).compare(
            ignition::common::separator("")))
          {
            // Remove the separator at end of path
            sdfCompressPath = sdfCompressPath.substr(0,
                sdfCompressPath.length() - 1);
          }
          sdfCompressPath += ".zip";
          this->config.SetLogRecordCompressPath(sdfCompressPath);
        }
      }
    }
    else
    {
      this->config.SetLogRecordPath(_config.LogRecordPath());
    }
  }

  if (_config.LogRecordResources())
    this->config.SetLogRecordResources(true);

  if (_config.LogRecordCompressPath() != ".zip")
    this->config.SetLogRecordCompressPath(_config.LogRecordCompressPath());

  if (_config.LogRecordTopics().size())
  {
    this->config.ClearLogRecordTopics();
    for (auto topic : _config.LogRecordTopics())
    {
      this->config.AddLogRecordTopic(topic);
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

    if (this->numberOfModelToDownload == this->currentNumberOfModelToDownload)
    {
      runner->SetFetchedAllIncludes(true);
    }
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
    ignmsg << "Resource path get service on [" << getPathService << "]."
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

  std::string serverControlService{"/server_control"};
  if (this->node.Advertise(serverControlService,
                           &ServerPrivate::ServerControlService, this))
  {
    ignmsg << "Server control service on [" << serverControlService << "]."
           << std::endl;
  }
  else
  {
    ignerr << "Something went wrong, failed to advertise ["
           << serverControlService << "]" << std::endl;
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
bool ServerPrivate::ServerControlService(
  const ignition::msgs::ServerControl &_req, msgs::Boolean &_res)
{
  _res.set_data(false);

  if (_req.stop())
  {
    if (!this->stopThread)
    {
      this->stopThread = std::make_shared<std::thread>([this]{
        ignlog << "Stopping Gazebo" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        this->Stop();
      });
    }
    _res.set_data(true);
  }

  // TODO(chapulina): implement world cloning
  if (_req.clone() || _req.new_port() != 0 || !_req.save_world_name().empty())
  {
    ignerr << "ServerControl::clone is not implemented" << std::endl;
    _res.set_data(false);
  }

  // TODO(chapulina): implement adding a new world
  if (_req.new_world())
  {
    ignerr << "ServerControl::new_world is not implemented" << std::endl;
    _res.set_data(false);
  }

  // TODO(chapulina): implement loading a world
  if (!_req.open_filename().empty())
  {
    ignerr << "ServerControl::open_filename is not implemented" << std::endl;
    _res.set_data(false);
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
// Getting the first .sdf file in the path
std::string findFuelResourceSdf2(const std::string &_path)
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

//////////////////////////////////////////////////
std::string ServerPrivate::FetchResource(const std::string &_uri)
{
  numberOfModelToDownload++;
  std::string uri = _uri;
  std::string pathReturn;

  // TODO(ahcorde) Review this part again
  // <mesh>
  //   <uri>https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Radio/4/files/meshes/Radio.dae</uri>
  // </mesh>
  // This kind of models are not returning an error when the model is loaded
  // the first time
  std::string basename = ignition::common::basename(uri);
  std::vector<std::string> baseNameTokens =
    ignition::common::split(basename, ".");

  if (baseNameTokens.size() > 1)
  {
    auto path =
        fuel_tools::fetchResourceWithClient(uri, *this->fuelClient.get());
    currentNumberOfModelToDownload++;
    if (!path.empty())
    {
      for (auto &runner : this->simRunners)
      {
        runner->AddToFuelUriMap(path, _uri);
      }
      this->fuelUriMap[path] = _uri;
    }
    return path;
  }

  if (this->fuelClient->CachedModel(ignition::common::URI(_uri), pathReturn) ||
      this->fuelClient->CachedModelFile(ignition::common::URI(_uri), pathReturn))
  {
    currentNumberOfModelToDownload++;
  }
  else
  {
    threadDownloadModels.emplace_back([&, uri]()
    {
      auto path =
          fuel_tools::fetchResourceWithClient(uri, *this->fuelClient.get());

      currentNumberOfModelToDownload++;

      if (path.empty())
      {
        ignerr << "Not able to find the resource [" << uri << "]\n";
        return;
      }
      while (this->simRunners.size() == 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      if (!allModelDownloaded)
      {
        for (auto &runner : this->simRunners)
        {
          if (runner)
          {
            runner->SetFetchedAllIncludes(false);
          }
        }
      }

      for (auto &runner : this->simRunners)
      {
        if (!runner)
        {
          ignerr << "Runner is null" << '\n';
          continue;
        }
        runner->AddToFuelUriMap(path, uri);
        if (currentNumberOfModelToDownload != numberOfModelToDownload)
        {
          continue;
        }
        allModelDownloaded = true;
        sdf::Errors errors;
        sdf::Root rootTmp;
        if (!config.SdfString().empty())
        {
          if (!config.SdfFile().empty())
          {
            errors = rootTmp.LoadSdfString(config.SdfString());
          }
        }
        else if (!config.SdfFile().empty())
        {
          std::string filePath;

          // Check Fuel if it's a URL
          auto sdfUri = common::URI(config.SdfFile());
          if (sdfUri.Scheme() == "http" || sdfUri.Scheme() == "https")
          {
            std::string fuelCachePath;
            if (this->fuelClient->CachedWorld(common::URI(config.SdfFile()),
                fuelCachePath))
            {
              filePath = findFuelResourceSdf2(fuelCachePath);
            }
            else if (auto result = this->fuelClient->DownloadWorld(
                common::URI(config.SdfFile()), fuelCachePath))
            {
              filePath = findFuelResourceSdf2(fuelCachePath);
            }
            else
            {
              ignwarn << "Fuel couldn't download URL [" << config.SdfFile()
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

            filePath = systemPaths.FindFile(config.SdfFile());
          }

          errors = rootTmp.Load(filePath);
          if (!errors.empty())
          {
            for (auto &err : errors)
              ignerr << err << "\n";
          }
        }

        if (rootTmp.WorldCount() > 0)
        {
          for (size_t i = 0; i < rootTmp.WorldByIndex(0)->LightCount(); i++)
          {
            auto light = rootTmp.WorldByIndex(0)->LightByIndex(i);
            if(!this->sdfRoot.LightNameExists(light->Name()) &&
               (!this->sdfRoot.WorldByIndex(0)->LightNameExists(
                 light->Name())))
            {
              runner->AddLight(*light);
            }
          }
          for (size_t i = 0; i < rootTmp.WorldByIndex(0)->ModelCount(); i++)
          {
            auto m = rootTmp.WorldByIndex(0)->ModelByIndex(i);
            if(!this->sdfRoot.ModelNameExists(m->Name()) &&
               (!this->sdfRoot.WorldByIndex(0)->ModelNameExists(m->Name())))
            {
              runner->AddModel(*m);
            }
          }
          for (size_t i = 0; i < rootTmp.WorldByIndex(0)->ActorCount(); i++)
          {
            auto actor = rootTmp.WorldByIndex(0)->ActorByIndex(i);
            if(!this->sdfRoot.ActorNameExists(actor->Name()) &&
               (!this->sdfRoot.WorldByIndex(0)->ActorNameExists(
                 actor->Name())))
            {
              runner->AddActor(*actor);
            }
          }
        }
        runner->SetFetchedAllIncludes(true);
      }
      this->fuelUriMap[path] = uri;
    });
  }

  return pathReturn;
}

//////////////////////////////////////////////////
std::string ServerPrivate::FetchResourceUri(const common::URI &_uri)
{
  return this->FetchResource(_uri.Str());
}
