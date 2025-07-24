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
#include "MeshInertiaCalculator.hh"
#include "ServerPrivate.hh"

#include <tinyxml2.h>

#include <string>
#include <utility>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/server_control.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/stringmsg_v.pb.h>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>

#include <gz/fuel_tools/Interface.hh>

#include "gz/sim/Util.hh"
#include "SimulationRunner.hh"

using namespace gz;
using namespace sim;

const char ServerPrivate::kClassicMaterialScriptUri[] =
    "file://media/materials/scripts/gazebo.material";

/// \brief This struct provides access to the record plugin SDF string
struct LoggingPlugin
{
  /// \brief Get the record plugin file name as a string
  /// \return A string that contains the record plugin file name.
  public: static std::string &LoggingPluginFileName()
  {
    static std::string recordPluginFileName =
      std::string("gz-sim") +
      GZ_SIM_MAJOR_VERSION_STR + "-log-system";
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
      "gz::sim::systems::LogRecord";
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
      "gz::sim::systems::LogPlayback";
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
}

//////////////////////////////////////////////////
void ServerPrivate::OnSignal(int _sig)
{
  gzdbg << "Server received signal[" << _sig  << "]\n";
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
      gzerr << "Failed to start network, simulation terminating" << std::endl;
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
    if (!this->workerPool.has_value())
    {
      // Initialize the workerpool if we do have multiple simulation runners and
      // it hasn't been initialized before
      this->workerPool.emplace(2);
    }
    for (std::unique_ptr<SimulationRunner> &runner : this->simRunners)
    {
      this->workerPool->AddWork([&runner, &_iterations] ()
        {
          runner->Run(_iterations);
        });
    }

    // Wait for the runner to complete.
    result = this->workerPool->WaitForResults();
  }

  this->running = false;
  return result;
}

//////////////////////////////////////////////////
void ServerPrivate::AddRecordPlugin(const ServerConfig &_config)
{
  bool hasRecordResources {false};
  bool hasRecordTopics {false};

  bool sdfRecordResources;
  std::vector<std::string> sdfRecordTopics;

  for (uint64_t worldIndex = 0; worldIndex < this->sdfRoot.WorldCount();
       ++worldIndex)
  {
    sdf::World *world = this->sdfRoot.WorldByIndex(worldIndex);
    sdf::Plugins &plugins = world->Plugins();

    for (sdf::Plugins::iterator iter = plugins.begin();
         iter != plugins.end(); ++iter)
    {
      std::string fname = iter->Filename();
      std::string name = iter->Name();
      if (fname.find(
            LoggingPlugin::LoggingPluginSuffix()) != std::string::npos &&
          name == LoggingPlugin::RecordPluginName())
      {
        sdf::ElementPtr recordPluginElem = iter->ToElement();

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
            recordTopicElem = recordTopicElem->GetNextElement();
          }
        }

        // Remove the plugin, which will be added back in by ServerConfig.
        plugins.erase(iter);
        break;
      }
    }
  }

  // Set the config based on what is in the SDF:
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

  if (!_config.LogRecordPath().empty())
  {
    this->config.SetLogRecordPath(_config.LogRecordPath());
  }

  if (_config.LogRecordPeriod() > std::chrono::steady_clock::duration::zero())
  {
    this->config.SetLogRecordPeriod(_config.LogRecordPeriod());
  }

  if (_config.LogRecordResources())
    this->config.SetLogRecordResources(true);

  if (_config.LogRecordCompressPath() != ".zip")
  {
    this->config.SetLogRecordCompressPath(_config.LogRecordCompressPath());
  }

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
    sdf::World *world = this->sdfRoot.WorldByIndex(worldIndex);

    {
      std::lock_guard<std::mutex> lock(this->worldsMutex);
      this->worldNames.push_back(world->Name());
    }
    auto runner = std::make_unique<SimulationRunner>(
        *world, this->systemLoader, this->config);
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
    gzmsg << "Serving world names on [" << worldsService << "]" << std::endl;
  }
  else
  {
    gzerr << "Something went wrong, failed to advertise [" << worldsService
           << "]" << std::endl;
  }

  // Resource path management
  std::string addPathService{"/gazebo/resource_paths/add"};
  if (this->node.Advertise(addPathService,
      &ServerPrivate::AddResourcePathsService, this))
  {
    gzmsg << "Resource path add service on [" << addPathService << "]."
           << std::endl;
  }
  else
  {
    gzerr << "Something went wrong, failed to advertise [" << addPathService
           << "]" << std::endl;
  }

  std::string getPathService{"/gazebo/resource_paths/get"};
  if (this->node.Advertise(getPathService,
      &ServerPrivate::ResourcePathsService, this))
  {
    gzmsg << "Resource path get service on [" << getPathService << "]."
           << std::endl;
  }
  else
  {
    gzerr << "Something went wrong, failed to advertise [" << getPathService
           << "]" << std::endl;
  }

  // Advertise a service that returns the full path, on the Gazebo server's
  // host machine, based on a provided URI.
  std::string resolvePathService{"/gazebo/resource_paths/resolve"};
  if (this->node.Advertise(resolvePathService,
      &ServerPrivate::ResourcePathsResolveService, this))
  {
    gzmsg << "Resource path resolve service on [" << resolvePathService << "]."
           << std::endl;
  }
  else
  {
    gzerr << "Something went wrong, failed to advertise [" << getPathService
           << "]" << std::endl;
  }

  std::string pathTopic{"/gazebo/resource_paths"};
  this->pathPub = this->node.Advertise<msgs::StringMsg_V>(pathTopic);

  if (this->pathPub)
  {
    gzmsg << "Resource paths published on [" << pathTopic << "]."
           << std::endl;
  }
  else
  {
    gzerr << "Something went wrong, failed to advertise [" << pathTopic
           << "]" << std::endl;
  }

  std::string serverControlService{"/server_control"};
  if (this->node.Advertise(serverControlService,
                           &ServerPrivate::ServerControlService, this))
  {
    gzmsg << "Server control service on [" << serverControlService << "]."
           << std::endl;
  }
  else
  {
    gzerr << "Something went wrong, failed to advertise ["
           << serverControlService << "]" << std::endl;
  }
}

//////////////////////////////////////////////////
bool ServerPrivate::WorldsService(msgs::StringMsg_V &_res)
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
  const msgs::ServerControl &_req, msgs::Boolean &_res)
{
  _res.set_data(false);

  if (_req.stop())
  {
    if (!this->stopThread)
    {
      this->stopThread = std::make_shared<std::thread>([this]{
        gzlog << "Stopping Gazebo" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        this->Stop();
      });
    }
    _res.set_data(true);
  }

  // TODO(chapulina): implement world cloning
  if (_req.clone() || _req.new_port() != 0 || !_req.save_world_name().empty())
  {
    gzerr << "ServerControl::clone is not implemented" << std::endl;
    _res.set_data(false);
  }

  // TODO(chapulina): implement adding a new world
  if (_req.new_world())
  {
    gzerr << "ServerControl::new_world is not implemented" << std::endl;
    _res.set_data(false);
  }

  // TODO(chapulina): implement loading a world
  if (!_req.open_filename().empty())
  {
    gzerr << "ServerControl::open_filename is not implemented" << std::endl;
    _res.set_data(false);
  }

  return true;
}

//////////////////////////////////////////////////
void ServerPrivate::AddResourcePathsService(
    const msgs::StringMsg_V &_req)
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
    msgs::StringMsg_V &_res)
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
bool ServerPrivate::ResourcePathsResolveService(
    const msgs::StringMsg &_req,
    msgs::StringMsg &_res)
{
  // Get the request
  std::string req = _req.data();

  // Handle the case where the request is already a valid path
  if (common::exists(common::absPath(req)))
  {
    _res.set_data(common::absPath(req));
    return true;
  }

  // Try Fuel
  std::string path =
      fuel_tools::fetchResourceWithClient(req, *this->fuelClient.get());
  if (!path.empty() && common::exists(path))
  {
    _res.set_data(path);
    return true;
  }

  // Check for the file:// prefix.
  std::string prefix = "file://";
  if (req.find(prefix) == 0)
  {
    req = req.substr(prefix.size());
    // Check to see if the path exists
    if (common::exists(req))
    {
      _res.set_data(req);
      return true;
    }
  }

  // Check for the model:// prefix
  prefix = "model://";
  if (req.find(prefix) == 0)
    req = req.substr(prefix.size());

  // Checkout resource paths
  std::vector<std::string> gzPaths = resourcePaths();
  for (const std::string &gzPath : gzPaths)
  {
    std::string fullPath = common::joinPaths(gzPath, req);
    if (common::exists(fullPath))
    {
      _res.set_data(fullPath);
      return true;
    }
  }

  // Otherwise the resource could not be found
  return false;
}

//////////////////////////////////////////////////
std::string ServerPrivate::FetchResource(const std::string &_uri)
{
  // Handle gazebo classic material URIs.
  // Return original URI string as the SdfEntityCreator checks for this URI
  if (_uri == kClassicMaterialScriptUri)
    return _uri;

  // Fetch resource from fuel
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

//////////////////////////////////////////////////
sdf::Errors ServerPrivate::LoadSdfRootHelper(const ServerConfig &_config)
{
  sdf::Errors errors;

  sdf::ParserConfig sdfParserConfig = sdf::ParserConfig::GlobalConfig();
  sdfParserConfig.SetStoreResolvedURIs(true);
  sdfParserConfig.SetCalculateInertialConfiguration(
    sdf::ConfigureResolveAutoInertials::SKIP_CALCULATION_IN_LOAD);
  MeshInertiaCalculator meshInertiaCalculator;
  sdfParserConfig.RegisterCustomInertiaCalc(meshInertiaCalculator);

  switch (_config.Source())
  {
    // Load a world if specified. Check SDF string first, then SDF file
    case ServerConfig::SourceType::kSdfRoot:
    {
      this->sdfRoot = _config.SdfRoot()->Clone();
      gzmsg << "Loading SDF world from SDF DOM.\n";
      break;
    }

    case ServerConfig::SourceType::kSdfString:
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
      gzmsg << msg;
      errors = this->sdfRoot.LoadSdfString(
        _config.SdfString(), sdfParserConfig);
      this->sdfRoot.ResolveAutoInertials(errors, sdfParserConfig);
      break;
    }

    case ServerConfig::SourceType::kSdfFile:
    {
      std::string filePath = resolveSdfWorldFile(_config.SdfFile(),
          _config.ResourceCache());

      if (filePath.empty())
      {
        std::string errStr =  "Failed to find world ["
          + _config.SdfFile() + "]";
        gzerr << errStr << std::endl;
        errors.push_back({sdf::ErrorCode::FILE_READ, errStr});
        return errors;
      }

      gzmsg << "Loading SDF world file[" << filePath << "].\n";

      sdf::Root sdfRootLocal;
      // \todo(nkoenig) Async resource download.
      // This call can block for a long period of time while
      // resources are downloaded. Blocking here causes the GUI to block with
      // a black screen (search for "Async resource download" in
      // 'src/gui_main.cc'.
      errors = sdfRootLocal.Load(filePath, sdfParserConfig);
      if (errors.empty() || _config.BehaviorOnSdfErrors() !=
          ServerConfig::SdfErrorBehavior::EXIT_IMMEDIATELY)
      {
        if (sdfRootLocal.Model() == nullptr) {
          this->sdfRoot = std::move(sdfRootLocal);
        }
        else
        {
          sdf::World defaultWorld;
          defaultWorld.SetName("default");

          // If the specified file only contains a model, load the default
          // world and add the model to it.
          errors = this->sdfRoot.AddWorld(defaultWorld);
          sdf::World *world = this->sdfRoot.WorldByIndex(0);
          if (world == nullptr) {
            errors.push_back({sdf::ErrorCode::FATAL_ERROR,
              "sdf::World pointer is null"});
            return errors;
          }
          world->AddModel(*sdfRootLocal.Model());
          if (errors.empty() || _config.BehaviorOnSdfErrors() !=
              ServerConfig::SdfErrorBehavior::EXIT_IMMEDIATELY)
          {
            errors = this->sdfRoot.UpdateGraphs();
          }
        }
      }

      this->sdfRoot.ResolveAutoInertials(errors, sdfParserConfig);
      break;
    }

    case ServerConfig::SourceType::kNone:
    default:
    {
      gzmsg << "Loading default world.\n";

      sdf::World defaultWorld;
      defaultWorld.SetName("default");

      // Load an empty world.
      errors = this->sdfRoot.AddWorld(defaultWorld);
      break;
    }
  }

  return errors;
}
