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
  public: ServerConfigPrivate() = default;

  /// \brief Copy constructor.
  /// \param[in] _cfg Configuration to copy.
  public: explicit ServerConfigPrivate(
              const std::unique_ptr<ServerConfigPrivate> &_cfg)
          : sdfFile(_cfg->sdfFile),
            updateRate(_cfg->updateRate),
            useLevels(_cfg->useLevels),
            resourceCache(_cfg->resourceCache),
            plugins(_cfg->plugins) { }

  // \brief The SDF file that the server should load
  public: std::string sdfFile = "";

  // \brief The SDF string that the server should load
  public: std::string sdfString = "";

  /// \brief An optional update rate.
  public: std::optional<double> updateRate;

  /// \brief Use the level system
  public: bool useLevels{false};

  /// \brief Path to where simulation resources, such as models downloaded
  /// from fuel.ignitionrobotics.org, should be stored.
  public: std::string resourceCache = "";

  /// \brief List of plugins to load.
  public: std::list<ServerConfig::PluginInfo> plugins;
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
void ServerConfig::AddPlugin(const ServerConfig::PluginInfo &_info)
{
  this->dataPtr->plugins.push_back(_info);
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
