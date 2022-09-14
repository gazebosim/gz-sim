/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gz/utils/cli/CLI.hpp>

#include "gz.hh"

#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>

//////////////////////////////////////////////////
/// \brief Structure to hold all available topic options
struct ServerOptions 
{
  /// \brief  Use the level system
  bool levels {false};

  /// \brief Run the simulation on start
  bool runOnStart {false};

  int iterations {0};

  /// \brief Update rate in hertz
  float updateRate {0.0}; 

  /// \brief Physics engine to use
  std::string physicsEngine {""};

  /// \brief Render engine to use
  std::string renderEngine {""};

  std::string sdfFile;
};

//////////////////////////////////////////////////
void runServer(const ServerOptions &_options)
{
  gz::sim::ServerConfig serverConfig;

  gz::common::Console::SetVerbosity(4);

  serverConfig.SetUseLevels(_options.levels);
  serverConfig.SetSdfFile(_options.sdfFile);

  if (_options.updateRate > 0.0)
    serverConfig.SetUpdateRate(_options.updateRate);

  if (!_options.renderEngine.empty())
  {
    serverConfig.SetRenderEngineServer(_options.renderEngine);
  }

  if (!_options.physicsEngine.empty())
  {
    serverConfig.SetPhysicsEngine(_options.physicsEngine);
  }

  gz::sim::Server server(serverConfig);
  server.Run(true, _options.iterations, !_options.runOnStart);
}

//////////////////////////////////////////////////
void addServerFlags(CLI::App &_app)
{
  auto opt = std::make_shared<ServerOptions>();

  _app.add_option("--levels", opt->levels, "Use the level system");
  _app.add_option("-r", opt->runOnStart, "Run the simulation on start");
  _app.add_option("-z", opt->updateRate, "Set simulation update rate");
  _app.add_option("--physics-engine", opt->physicsEngine);
  _app.add_option("--render-engine", opt->renderEngine);
  _app.add_option("file", opt->sdfFile);

  _app.callback([opt](){runServer(*opt); });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Run Gazebo Server"};

  addServerFlags(app);
  CLI11_PARSE(app, argc, argv);
}

