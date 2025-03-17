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

#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/utils/cli/CLI.hpp>
#include <gz/utils/cli/GzFormatter.hpp>
#include <gz/utils/Environment.hh>
#include <gz/utils/Subprocess.hh>

#include "gz/sim/config.hh"
#include "gz.hh"

using namespace gz;

//////////////////////////////////////////////////
ExeSubprocess launchProcess(
    const std::string &_executable,
    std::vector<std::string> _args)
{
  _args.emplace(_args.begin(), _executable);
  return ExeSubprocess(_args);
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Run and manage Gazebo simulations."};

  app.add_flag_callback("--version",
    [](){
      std::cout << GZ_SIM_VERSION_FULL << std::endl;
      throw CLI::Success();
    },
    "Print the current library version.");

  app.add_option_function<int>("-v,--verbose",
    [](const int _verbosity){
      utils::setenv(
        std::string("GZ_SIM_VERBOSITY"),
        std::to_string(_verbosity));

      cmdVerbosity(_verbosity);
    },
    "Adjust the level of console output (0~4).\n"
    "The default verbosity level is 1. Use -v\n"
    "without arguments for level 3.\n")
    ->expected(0, 1)
    ->default_val(3);

  // Dummy flags handled by gz-tools
  app.add_flag("--force-version", "Use a particular library version.");
  app.add_flag("--versions", "Show the available versions.");

  app.footer(
    "Environment variables:\n"
    "   GZ_SIM_RESOURCE_PATH          Colon separated paths used to locate\n"
    "                                 resources such as worlds and models.\n"
    "   GZ_SIM_SYSTEM_PLUGIN_PATH     Colon separated paths used to locate\n"
    "                                 system plugins.\n"
    "   GZ_SIM_SERVER_CONFIG_PATH     Path to server configuration file.\n"
    "   GZ_GUI_PLUGIN_PATH            Colon separated paths used to locate\n"
    "                                 GUI plugins.\n"
    "   GZ_GUI_RESOURCE_PATH          Colon separated paths used to locate\n"
    "                                 resources such as configuration files.");

  bool launchServer = false;
  app.add_flag("-s", launchServer, "Run and manage Gazebo Server");

  bool launchGui = false;
  app.add_flag("-g", launchGui, "Run and manage Gazebo GUI");

  app.allow_extras();

  app.formatter(std::make_shared<GzFormatter>(&app));

  CLI11_PARSE(app, argc, argv);

  std::vector<std::string> extraArgs = app.remaining();

  if(!launchServer && !launchGui)
  {
    auto guiProc = launchProcess(
        std::string(GZ_SIM_GUI_EXE), extraArgs);

    auto serverProc = launchProcess(
        std::string(GZ_SIM_SERVER_EXE), extraArgs);

    while(serverProc.Alive() && guiProc.Alive());

    if(serverProc.Alive())
    {
      serverProc.Terminate();
    }
    if(guiProc.Alive())
    {
      guiProc.Terminate();
    }
  }
  else
  {
    utils::setenv(std::string("GZ_SIM_WAIT_GUI"), std::to_string(0));

    if(launchServer)
    {
      auto proc = launchProcess(std::string(GZ_SIM_SERVER_EXE), extraArgs);
      proc.Join();
    }
    else if(launchGui)
    {
      auto proc = launchProcess(std::string(GZ_SIM_GUI_EXE), extraArgs);
      proc.Join();
    }
  }

  return 0;
}
