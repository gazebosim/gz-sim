/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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


#include <string>

#include <gz/common/Console.hh>
#include <gz/sim/ServerConfig.hh>
#include <sdf/sdf.hh>

#include "RuntimeConfig.hh"

using namespace gz;
using namespace sim;

RuntimeConfig sim::parseRuntimeConfig(const std::string& _filename)
{
  std::list<ServerConfig::PluginInfo> plugins =
    parsePluginsFromFile(_filename);
  if (plugins.empty())
  {
    gzerr << "Failed to load plugins from config file.\n";
    return {};
  }
  if (plugins.size() > 1)
  {
    gzerr << "Only 1 plugin is currently supported. Found "
          << plugins.size() << ".\n";
    return {};
  }
  RuntimeConfig config;
  config.plugin = plugins.front().Plugin();
  return config;
}
