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
#include <iostream>
#include "ignition/gazebo/ServerConfig.hh"

using namespace ignition;
using namespace gazebo;

class ignition::gazebo::ServerConfigPrivate
{
  public: ServerConfigPrivate() = default;
  public: ServerConfigPrivate(const std::unique_ptr<ServerConfigPrivate> &_cfg)
          : sdfFile(_cfg->sdfFile),
            updateRate(_cfg->updateRate),
            useLevels(_cfg->useLevels),
            resourceCache(_cfg->resourceCache),
            plugins(_cfg->plugins) { }

  // \brief The SDF file that the server should load
  public: std::string sdfFile = "";

  /// \brief An optional update rate.
  public: std::optional<double> updateRate;

  /// \brief Use the level system
  public: bool useLevels{false};

  /// \brief Path to where simulation resources, such as models downloaded
  /// from fuel.ignitionrobotics.org, should be stored.
  public: std::string resourceCache = "";

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
  return true;
}

/////////////////////////////////////////////////
std::string ServerConfig::SdfFile() const
{
  return this->dataPtr->sdfFile;
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
  this->dataPtr.reset(new ServerConfigPrivate(_cfg.dataPtr));
  return *this;
}
