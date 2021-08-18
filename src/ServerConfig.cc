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
#include "ignition/gazebo/ServerConfig.hh"

#include <tinyxml2.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include <ignition/fuel_tools/FuelClient.hh>
#include <ignition/fuel_tools/Result.hh>
#include <ignition/math/Rand.hh>

#include "ignition/gazebo/Util.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Private data for PluginInfoConfig.
class ignition::gazebo::ServerConfig::PluginInfoPrivate
{
  /// \brief Default constructor.
  public: PluginInfoPrivate() = default;

  /// \brief Copy constructor
  /// \param[in] _info Plugin to copy.
  public: explicit PluginInfoPrivate(
              const std::unique_ptr<ServerConfig::PluginInfoPrivate> &_info)
          : entityName(_info->entityName),
            entityType(_info->entityType),
            filename(_info->filename),
            name(_info->name)
  {
    if (_info->sdf)
      this->sdf = _info->sdf->Clone();
  }

  /// \brief Constructor based on values.
  /// \param[in] _entityName Name of the entity which should receive
  /// this plugin. The name is used in conjuction with _entityType to
  /// uniquely identify an entity.
  /// \param[in] _entityType Entity type which should receive  this
  /// plugin. The type is used in conjuction with _entityName to
  /// uniquely identify an entity.
  /// \param[in] _filename Plugin library filename.
  /// \param[in] _name Name of the interface within the plugin library
  /// to load.
  /// \param[in] _sdf Plugin XML elements associated with this plugin.
  // cppcheck-suppress passedByValue
  public: PluginInfoPrivate(std::string _entityName,
  // cppcheck-suppress passedByValue
                            std::string _entityType,
  // cppcheck-suppress passedByValue
                            std::string _filename,
  // cppcheck-suppress passedByValue
                            std::string _name)
          : entityName(std::move(_entityName)),
            entityType(std::move(_entityType)),
            filename(std::move(_filename)),
            name(std::move(_name))
  {
  }

  /// \brief The name of the entity.
  public: std::string entityName = "";

  /// \brief The type of entity.
  public: std::string entityType = "";

  /// \brief _filename The plugin library.
  public: std::string filename = "";

  /// \brief Name of the plugin implementation.
  public: std::string name = "";

  /// \brief XML elements associated with this plugin
  public: sdf::ElementPtr sdf = nullptr;
};

//////////////////////////////////////////////////
ServerConfig::PluginInfo::PluginInfo()
: dataPtr(new ServerConfig::PluginInfoPrivate)
{
}

//////////////////////////////////////////////////
ServerConfig::PluginInfo::~PluginInfo() = default;

//////////////////////////////////////////////////
ServerConfig::PluginInfo::PluginInfo(const std::string &_entityName,
                       const std::string &_entityType,
                       const std::string &_filename,
                       const std::string &_name,
                       const sdf::ElementPtr &_sdf)
  : dataPtr(new ServerConfig::PluginInfoPrivate(_entityName, _entityType,
                                  _filename, _name))
{
  if (_sdf)
    this->dataPtr->sdf = _sdf->Clone();
}

//////////////////////////////////////////////////
ServerConfig::PluginInfo::PluginInfo(const ServerConfig::PluginInfo &_info)
  : dataPtr(new ServerConfig::PluginInfoPrivate(_info.dataPtr))
{
}

//////////////////////////////////////////////////
ServerConfig::PluginInfo &ServerConfig::PluginInfo::operator=(
    const ServerConfig::PluginInfo &_info)
{
  this->dataPtr = std::make_unique<ServerConfig::PluginInfoPrivate>(
      _info.dataPtr);
  return *this;
}

//////////////////////////////////////////////////
const std::string &ServerConfig::PluginInfo::EntityName() const
{
  return this->dataPtr->entityName;
}

//////////////////////////////////////////////////
void ServerConfig::PluginInfo::SetEntityName(const std::string &_entityName)
{
  this->dataPtr->entityName = _entityName;
}

//////////////////////////////////////////////////
const std::string &ServerConfig::PluginInfo::EntityType() const
{
  return this->dataPtr->entityType;
}

//////////////////////////////////////////////////
void ServerConfig::PluginInfo::SetEntityType(const std::string &_entityType)
{
  this->dataPtr->entityType = _entityType;
}

//////////////////////////////////////////////////
const std::string &ServerConfig::PluginInfo::Filename() const
{
  return this->dataPtr->filename;
}

//////////////////////////////////////////////////
void ServerConfig::PluginInfo::SetFilename(const std::string &_filename)
{
  this->dataPtr->filename = _filename;
}

//////////////////////////////////////////////////
const std::string &ServerConfig::PluginInfo::Name() const
{
  return this->dataPtr->name;
}

//////////////////////////////////////////////////
void ServerConfig::PluginInfo::SetName(const std::string &_name)
{
  this->dataPtr->name = _name;
}

//////////////////////////////////////////////////
const sdf::ElementPtr &ServerConfig::PluginInfo::Sdf() const
{
  return this->dataPtr->sdf;
}

//////////////////////////////////////////////////
void ServerConfig::PluginInfo::SetSdf(const sdf::ElementPtr &_sdf)
{
  if (_sdf)
    this->dataPtr->sdf = _sdf->Clone();
  else
    this->dataPtr->sdf = nullptr;
}

/// \brief Private data for ServerConfig.
class ignition::gazebo::ServerConfigPrivate
{
  /// \brief Default constructor.
  public: ServerConfigPrivate()
  {
    std::string home;
    common::env(IGN_HOMEDIR, home);

    this->timestamp = IGN_SYSTEM_TIME();

    // Set a default log record path
    this->logRecordPath = common::joinPaths(home,
        ".ignition", "gazebo", "log", common::timeToIso(this->timestamp));

    // If directory already exists, do not overwrite. This could potentially
    // happen if multiple simulation instances are started in rapid
    // succession.
    if (common::exists(this->logRecordPath))
    {
      this->logRecordPath = common::uniqueDirectoryPath(this->logRecordPath);
    }
  }

  /// \brief Copy constructor.
  /// \param[in] _cfg Configuration to copy.
  public: explicit ServerConfigPrivate(
              const std::unique_ptr<ServerConfigPrivate> &_cfg)
          : sdfFile(_cfg->sdfFile),
            updateRate(_cfg->updateRate),
            useLevels(_cfg->useLevels),
            useLogRecord(_cfg->useLogRecord),
            logRecordPath(_cfg->logRecordPath),
            logIgnoreSdfPath(_cfg->logIgnoreSdfPath),
            logPlaybackPath(_cfg->logPlaybackPath),
            logRecordResources(_cfg->logRecordResources),
            logRecordCompressPath(_cfg->logRecordCompressPath),
            resourceCache(_cfg->resourceCache),
            physicsEngine(_cfg->physicsEngine),
            renderEngineServer(_cfg->renderEngineServer),
            renderEngineGui(_cfg->renderEngineGui),
            plugins(_cfg->plugins),
            networkRole(_cfg->networkRole),
            networkSecondaries(_cfg->networkSecondaries),
            seed(_cfg->seed),
            logRecordTopics(_cfg->logRecordTopics) { }

  // \brief The SDF file that the server should load
  public: std::string sdfFile = "";

  // \brief The SDF string that the server should load
  public: std::string sdfString = "";

  /// \brief An optional update rate.
  public: std::optional<double> updateRate;

  /// \brief Use the level system
  public: bool useLevels{false};

  /// \brief Use the logging system to record states
  public: bool useLogRecord{false};

  /// \brief Path to place recorded states
  public: std::string logRecordPath = "";

  /// TODO(anyone) Deprecate in public APIs in Ignition-D, remove in Ignition-E
  /// \brief Whether log record path is specified from command line
  public: bool logIgnoreSdfPath{false};

  /// \brief Path to recorded states to play back using logging system
  public: std::string logPlaybackPath = "";

  /// \brief Record meshes and material files
  public: bool logRecordResources{false};

  /// \brief Path to compress log files to
  public: std::string logRecordCompressPath = "";

  /// \brief Path to where simulation resources, such as models downloaded
  /// from fuel.ignitionrobotics.org, should be stored.
  public: std::string resourceCache = "";

  /// \brief File containing physics engine plugin. If empty, DART will be used.
  public: std::string physicsEngine = "";

  /// \brief File containing render engine server plugin. If empty, OGRE2
  /// will be used.
  public: std::string renderEngineServer = "";

  /// \brief File containing render engine gui plugin. If empty, OGRE2
  /// will be used.
  public: std::string renderEngineGui = "";

  /// \brief List of plugins to load.
  public: std::list<ServerConfig::PluginInfo> plugins;

  /// \brief The network role.
  public: std::string networkRole = "";

  /// \brief The number of network secondaries.
  public: unsigned int networkSecondaries = 0;

  /// \brief The given random seed.
  public: unsigned int seed = 0;

  /// \brief Timestamp that marks when this ServerConfig was created.
  public: std::chrono::time_point<std::chrono::system_clock> timestamp;

  /// \brief Topics to record.
  public: std::vector<std::string> logRecordTopics;
};

//////////////////////////////////////////////////
ServerConfig::ServerConfig()
  : dataPtr(new ServerConfigPrivate)
{
}

//////////////////////////////////////////////////
ServerConfig::ServerConfig(const ServerConfig &_config)
  : dataPtr(new ServerConfigPrivate(_config.dataPtr))
{
}

//////////////////////////////////////////////////
ServerConfig::~ServerConfig() = default;

//////////////////////////////////////////////////
bool ServerConfig::SetSdfFile(const std::string &_file)
{
  this->dataPtr->sdfFile = _file;
  this->dataPtr->sdfString = "";
  return true;
}

/////////////////////////////////////////////////
std::string ServerConfig::SdfFile() const
{
  return this->dataPtr->sdfFile;
}

//////////////////////////////////////////////////
bool ServerConfig::SetSdfString(const std::string &_sdfString)
{
  this->dataPtr->sdfFile = "";
  this->dataPtr->sdfString = _sdfString;
  return true;
}

/////////////////////////////////////////////////
std::string ServerConfig::SdfString() const
{
  return this->dataPtr->sdfString;
}

//////////////////////////////////////////////////
void ServerConfig::SetUpdateRate(const double &_hz)
{
  if (_hz > 0)
    this->dataPtr->updateRate = _hz;
}

/////////////////////////////////////////////////
std::optional<double> ServerConfig::UpdateRate() const
{
  return this->dataPtr->updateRate;
}

/////////////////////////////////////////////////
std::optional<std::chrono::steady_clock::duration>
    ServerConfig::UpdatePeriod() const
{
  if (this->dataPtr->updateRate)
  {
    std::chrono::duration<double, std::ratio<1>> seconds(
        1.0 / this->dataPtr->updateRate.value());
    return std::chrono::duration_cast<std::chrono::nanoseconds>(seconds);
  }

  return std::nullopt;
}

/////////////////////////////////////////////////
bool ServerConfig::UseLevels() const
{
  return this->dataPtr->useLevels;
}

/////////////////////////////////////////////////
void ServerConfig::SetUseLevels(const bool _levels)
{
  this->dataPtr->useLevels = _levels;
}

/////////////////////////////////////////////////
void ServerConfig::SetNetworkSecondaries(unsigned int _secondaries)
{
  this->dataPtr->networkSecondaries = _secondaries;
}

/////////////////////////////////////////////////
unsigned int ServerConfig::NetworkSecondaries() const
{
  return this->dataPtr->networkSecondaries;
}

/////////////////////////////////////////////////
void ServerConfig::SetNetworkRole(const std::string &_role)
{
  this->dataPtr->networkRole = _role;
}

/////////////////////////////////////////////////
std::string ServerConfig::NetworkRole() const
{
  return this->dataPtr->networkRole;
}

/////////////////////////////////////////////////
bool ServerConfig::UseDistributedSimulation() const
{
  // We just check that network role is not empty.
  // src/network/NetworkConfig.cc checks if this value is valid.
  return !this->dataPtr->networkRole.empty();
}

/////////////////////////////////////////////////
bool ServerConfig::UseLogRecord() const
{
  return this->dataPtr->useLogRecord;
}

/////////////////////////////////////////////////
void ServerConfig::SetUseLogRecord(const bool _record)
{
  this->dataPtr->useLogRecord = _record;
}

/////////////////////////////////////////////////
const std::string ServerConfig::LogRecordPath() const
{
  return this->dataPtr->logRecordPath;
}

/////////////////////////////////////////////////
void ServerConfig::SetLogRecordPath(const std::string &_recordPath)
{
  this->dataPtr->logRecordPath = _recordPath;
}

/////////////////////////////////////////////////
bool ServerConfig::LogIgnoreSdfPath() const
{
  return this->dataPtr->logIgnoreSdfPath;
}

/////////////////////////////////////////////////
void ServerConfig::SetLogIgnoreSdfPath(bool _ignore)
{
  this->dataPtr->logIgnoreSdfPath = _ignore;
}

/////////////////////////////////////////////////
const std::string ServerConfig::LogPlaybackPath() const
{
  return this->dataPtr->logPlaybackPath;
}

/////////////////////////////////////////////////
void ServerConfig::SetLogPlaybackPath(const std::string &_playbackPath)
{
  this->dataPtr->logPlaybackPath = _playbackPath;
}

/////////////////////////////////////////////////
bool ServerConfig::LogRecordResources() const
{
  return this->dataPtr->logRecordResources;
}

/////////////////////////////////////////////////
void ServerConfig::SetLogRecordResources(bool _recordResources)
{
  this->dataPtr->logRecordResources = _recordResources;
}

/////////////////////////////////////////////////
std::string ServerConfig::LogRecordCompressPath() const
{
  return this->dataPtr->logRecordCompressPath;
}

/////////////////////////////////////////////////
void ServerConfig::SetLogRecordCompressPath(const std::string &_path)
{
  this->dataPtr->logRecordCompressPath = _path;
}

/////////////////////////////////////////////////
unsigned int ServerConfig::Seed() const
{
  return this->dataPtr->seed;
}

/////////////////////////////////////////////////
void ServerConfig::SetSeed(unsigned int _seed)
{
  this->dataPtr->seed = _seed;
  ignition::math::Rand::Seed(_seed);
}

/////////////////////////////////////////////////
const std::string &ServerConfig::ResourceCache() const
{
  return this->dataPtr->resourceCache;
}

/////////////////////////////////////////////////
void ServerConfig::SetResourceCache(const std::string &_path)
{
  this->dataPtr->resourceCache = _path;
}

/////////////////////////////////////////////////
const std::string &ServerConfig::PhysicsEngine() const
{
  return this->dataPtr->physicsEngine;
}

/////////////////////////////////////////////////
void ServerConfig::SetPhysicsEngine(const std::string &_physicsEngine)
{
  this->dataPtr->physicsEngine = _physicsEngine;
}

/////////////////////////////////////////////////
const std::string &ServerConfig::RenderEngineServer() const
{
  return this->dataPtr->renderEngineServer;
}

/////////////////////////////////////////////////
void ServerConfig::SetRenderEngineServer(const std::string &_renderEngineServer)
{
  this->dataPtr->renderEngineServer = _renderEngineServer;
}

/////////////////////////////////////////////////
const std::string &ServerConfig::RenderEngineGui() const
{
  return this->dataPtr->renderEngineGui;
}

/////////////////////////////////////////////////
void ServerConfig::SetRenderEngineGui(const std::string &_renderEngineGui)
{
  this->dataPtr->renderEngineGui = _renderEngineGui;
}

/////////////////////////////////////////////////
void ServerConfig::AddPlugin(const ServerConfig::PluginInfo &_info)
{
  this->dataPtr->plugins.push_back(_info);
}

/////////////////////////////////////////////////
ServerConfig::PluginInfo
ServerConfig::LogPlaybackPlugin() const
{
  auto entityName = "*";
  auto entityType = "world";
  auto pluginName = "ignition::gazebo::systems::LogPlayback";
  auto pluginFilename = "ignition-gazebo-log-system";

  sdf::ElementPtr playbackElem;
  playbackElem = std::make_shared<sdf::Element>();
  playbackElem->SetName("plugin");

  if (!this->LogPlaybackPath().empty())
  {
    sdf::ElementPtr pathElem = std::make_shared<sdf::Element>();
    pathElem->SetName("playback_path");
    playbackElem->AddElementDescription(pathElem);
    pathElem = playbackElem->GetElement("playback_path");
    pathElem->AddValue("string", "", false, "");
    pathElem->Set<std::string>(this->LogPlaybackPath());
  }

  return ServerConfig::PluginInfo(entityName,
      entityType,
      pluginFilename,
      pluginName,
      playbackElem);
}

/////////////////////////////////////////////////
ServerConfig::PluginInfo
ServerConfig::LogRecordPlugin() const
{
  auto entityName = "*";
  auto entityType = "world";
  auto pluginName = "ignition::gazebo::systems::LogRecord";
  auto pluginFilename = "ignition-gazebo-log-system";

  sdf::ElementPtr recordElem;

  recordElem = std::make_shared<sdf::Element>();
  recordElem->SetName("plugin");

  igndbg << "Generating LogRecord SDF:" << std::endl;

  if (!this->LogRecordPath().empty())
  {
    sdf::ElementPtr pathElem = std::make_shared<sdf::Element>();
    pathElem->SetName("record_path");
    recordElem->AddElementDescription(pathElem);
    pathElem = recordElem->GetElement("record_path");
    pathElem->AddValue("string", "", false, "");
    pathElem->Set<std::string>(this->LogRecordPath());
  }

  // Set whether to record resources
  sdf::ElementPtr resourceElem = std::make_shared<sdf::Element>();
  resourceElem->SetName("record_resources");
  recordElem->AddElementDescription(resourceElem);
  resourceElem = recordElem->GetElement("record_resources");
  resourceElem->AddValue("bool", "false", false, "");
  resourceElem->Set<bool>(this->LogRecordResources() ? true : false);

  if (!this->LogRecordCompressPath().empty())
  {
    // Set whether to compress
    sdf::ElementPtr compressElem = std::make_shared<sdf::Element>();
    compressElem->SetName("compress");
    recordElem->AddElementDescription(compressElem);
    compressElem = recordElem->GetElement("compress");
    compressElem->AddValue("bool", "false", false, "");
    compressElem->Set<bool>(true);

  // Set compress path
    sdf::ElementPtr cPathElem = std::make_shared<sdf::Element>();
    cPathElem->SetName("compress_path");
    recordElem->AddElementDescription(cPathElem);
    cPathElem = recordElem->GetElement("compress_path");
    cPathElem->AddValue("string", "", false, "");
    cPathElem->Set<std::string>(this->LogRecordCompressPath());
  }

  // If record topics specified, add in SDF
  for (const std::string &topic : this->LogRecordTopics())
  {
    sdf::ElementPtr topicElem = std::make_shared<sdf::Element>();
    topicElem->SetName("record_topic");
    recordElem->AddElementDescription(topicElem);
    topicElem = recordElem->AddElement("record_topic");
    topicElem->AddValue("string", "false", false, "");
    topicElem->Set<std::string>(topic);
  }

  igndbg << recordElem->ToString("") << std::endl;

  return ServerConfig::PluginInfo(entityName,
      entityType,
      pluginFilename,
      pluginName,
      recordElem);
}

/////////////////////////////////////////////////
const std::list<ServerConfig::PluginInfo> &ServerConfig::Plugins() const
{
  return this->dataPtr->plugins;
}

/////////////////////////////////////////////////
ServerConfig &ServerConfig::operator=(const ServerConfig &_cfg)
{
  this->dataPtr = std::make_unique<ServerConfigPrivate>(_cfg.dataPtr);
  return *this;
}

/////////////////////////////////////////////////
const std::chrono::time_point<std::chrono::system_clock> &
ServerConfig::Timestamp() const
{
  return this->dataPtr->timestamp;
}

/////////////////////////////////////////////////
void ServerConfig::AddLogRecordTopic(const std::string &_topic)
{
  this->dataPtr->logRecordTopics.push_back(_topic);
}

/////////////////////////////////////////////////
void ServerConfig::ClearLogRecordTopics()
{
  this->dataPtr->logRecordTopics.clear();
}

/////////////////////////////////////////////////
const std::vector<std::string> &ServerConfig::LogRecordTopics() const
{
  return this->dataPtr->logRecordTopics;
}

/////////////////////////////////////////////////
void copyElement(sdf::ElementPtr _sdf, const tinyxml2::XMLElement *_xml)
{
  _sdf->SetName(_xml->Value());
  if (_xml->GetText() != nullptr)
    _sdf->AddValue("string", _xml->GetText(), "1");

  for (const tinyxml2::XMLAttribute *attribute = _xml->FirstAttribute();
       attribute; attribute = attribute->Next())
  {
    _sdf->AddAttribute(attribute->Name(), "string", "", 1, "");
    _sdf->GetAttribute(attribute->Name())->SetFromString(
        attribute->Value());
  }

  // Iterate over all the child elements
  const tinyxml2::XMLElement *elemXml = nullptr;
  for (elemXml = _xml->FirstChildElement(); elemXml;
      elemXml = elemXml->NextSiblingElement())
  {
    sdf::ElementPtr element(new sdf::Element);
    element->SetParent(_sdf);

    copyElement(element, elemXml);
    _sdf->InsertElement(element);
  }
}

/////////////////////////////////////////////////
std::list<ServerConfig::PluginInfo>
parsePluginsFromDoc(const tinyxml2::XMLDocument &_doc)
{
  auto ret =  std::list<ServerConfig::PluginInfo>();
  auto root = _doc.RootElement();
  if (root == nullptr)
  {
    ignerr << "No <server_config> element found when parsing plugins\n";
    return ret;
  }

  auto plugins = root->FirstChildElement("plugins");
  if (plugins == nullptr)
  {
    ignerr << "No <plugins> element found when parsing plugins\n";
    return ret;
  }

  const tinyxml2::XMLElement *elem{nullptr};

  // Note, this was taken from ign-launch, where this type of parsing happens.
  // Process all the plugins.
  for (elem = plugins->FirstChildElement("plugin"); elem;
       elem = elem->NextSiblingElement("plugin"))
  {
    // Get the plugin's name
    const char *nameStr = elem->Attribute("name");
    std::string name = nameStr == nullptr ? "" : nameStr;
    if (name.empty())
    {
      ignerr << "Plugin is missing the name attribute. "
        << "Skipping this plugin.\n";
      continue;
    }

    // Get the plugin's filename
    const char *fileStr = elem->Attribute("filename");
    std::string file = fileStr == nullptr ? "" : fileStr;
    if (file.empty())
    {
      ignerr << "A Plugin with name[" << name << "] is "
        << "missing the filename attribute. Skipping this plugin.\n";
      continue;
    }

    // Get the plugin's entity name attachment information.
    const char *entityNameStr = elem->Attribute("entity_name");
    std::string entityName = entityNameStr == nullptr ? "" : entityNameStr;
    if (entityName.empty())
    {
      ignerr << "A Plugin with name[" << name << "] and "
        << "filename[" << file << "] is missing the entity_name attribute. "
        << "Skipping this plugin.\n";
      continue;
    }

    // Get the plugin's entity type attachment information.
    const char *entityTypeStr = elem->Attribute("entity_type");
    std::string entityType = entityTypeStr == nullptr ? "" : entityTypeStr;
    if (entityType.empty())
    {
      ignerr << "A Plugin with name[" << name << "] and "
        << "filename[" << file << "] is missing the entity_type attribute. "
        << "Skipping this plugin.\n";
      continue;
    }

    // Create an SDF element of the plugin
    sdf::ElementPtr sdf(new sdf::Element);
    copyElement(sdf, elem);

    // Add the plugin to the server config
    ret.push_back({entityName, entityType, file, name, sdf});
  }
  return ret;
}

/////////////////////////////////////////////////
std::list<ServerConfig::PluginInfo>
ignition::gazebo::parsePluginsFromFile(const std::string &_fname)
{
  tinyxml2::XMLDocument doc;
  doc.LoadFile(_fname.c_str());
  return parsePluginsFromDoc(doc);
}

/////////////////////////////////////////////////
std::list<ServerConfig::PluginInfo>
ignition::gazebo::parsePluginsFromString(const std::string &_str)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(_str.c_str());
  return parsePluginsFromDoc(doc);
}


/////////////////////////////////////////////////
std::list<ServerConfig::PluginInfo>
ignition::gazebo::loadPluginInfo(bool _isPlayback)
{
  std::list<ServerConfig::PluginInfo> ret;

  // 1. Check contents of environment variable
  std::string envConfig;
  bool configSet = ignition::common::env(gazebo::kServerConfigPathEnv,
                                         envConfig,
                                         true);

  if (configSet)
  {
    if (ignition::common::exists(envConfig))
    {
      // Parse configuration stored in environment variable
      ret = ignition::gazebo::parsePluginsFromFile(envConfig);
      if (ret.empty())
      {
        // This may be desired behavior, but warn just in case.
        // Some users may want to defer all loading until later
        // during runtime.
        ignwarn << gazebo::kServerConfigPathEnv
                << " set but no plugins found\n";
      }
      igndbg << "Loaded (" << ret.size() << ") plugins from file " <<
        "[" << envConfig << "]\n";

      return ret;
    }
    else
    {
      // This may be desired behavior, but warn just in case.
      // Some users may want to defer all loading until late
      // during runtime.
      ignwarn << gazebo::kServerConfigPathEnv
              << " set but no file found,"
              << " no plugins loaded\n";
      return ret;
    }
  }

  std::string configFilename;
  if (_isPlayback)
  {
    configFilename = "playback_server.config";
  }
  else
  {
    configFilename = "server.config";
  }

  std::string defaultConfig;
  ignition::common::env(IGN_HOMEDIR, defaultConfig);
  defaultConfig = ignition::common::joinPaths(defaultConfig, ".ignition",
    "gazebo", configFilename);

  if (!ignition::common::exists(defaultConfig))
  {
    auto installedConfig = ignition::common::joinPaths(
        IGNITION_GAZEBO_SERVER_CONFIG_PATH,
        configFilename);

    if (!ignition::common::exists(installedConfig))
    {
      ignerr << "Failed to copy installed config [" << installedConfig
             << "] to default config [" << defaultConfig << "]."
             << "(file " << installedConfig << " doesn't exist)"
             << std::endl;
      return ret;
    }
    else if (!ignition::common::copyFile(installedConfig, defaultConfig))
    {
      ignerr << "Failed to copy installed config [" << installedConfig
             << "] to default config [" << defaultConfig << "]."
             << std::endl;
      return ret;
    }
    else
    {
      ignmsg << "Copied installed config [" << installedConfig
             << "] to default config [" << defaultConfig << "]."
             << std::endl;
    }
  }

  ret = ignition::gazebo::parsePluginsFromFile(defaultConfig);

  if (ret.empty())
  {
    // This may be desired behavior, but warn just in case.
    ignwarn << "Loaded config: [" << defaultConfig
      << "], but no plugins found\n";
  }

  igndbg << "Loaded (" << ret.size() << ") plugins from file " <<
    "[" << defaultConfig << "]\n";

  return ret;
}
