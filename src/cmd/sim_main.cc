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

#include <iostream>
#include <string>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include <gz/common/Console.hh>
#include <gz/utils/cli/CLI.hpp>
#include <gz/utils/cli/GzFormatter.hpp>
#include <gz/utils/Environment.hh>
#include <gz/utils/Subprocess.hh>

#include "gz/sim/config.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz.hh"

using namespace gz;

//////////////////////////////////////////////////
/// \brief Structure to hold all available launch options
struct SimOptions
{
  /// \brief Name of the SDFormat file
  std::string file{""};

  /// \brief Update rate in hertz
  double rate{-1};

  /// \brief Initial simulation time (in seconds)
  double initialSimTime{0.0};

  /// \brief Number of iterations to execute
  int iterations{0};

  /// \brief Levels system to display models
  int levels{0};

  /// \brief Participant role in distributed simulation environment
  std::string networkRole{""};

  /// \brief Number of expected secondary participants
  int networkSecondaries{0};

  /// \brief Record states and console
  int record{0};

  /// \brief Custom path to store recorded files
  std::string recordPath{""};

  /// \brief Records meshes and material files
  int recordResources{0};

  /// \brief List of topics to record
  std::vector<std::string> recordTopics{};

  /// \brief Time period between state recording (in seconds)
  float recordPeriod{-1};

  /// \brief Overwrite existing files when recording
  int logOverwrite{0};

  /// \brief Compress final log when recording
  int logCompress{0};

  /// \brief Path to recorded states for playback
  std::string playback{""};

  /// \brief Run simulation on start
  int runOnStart{0};

  /// \brief Physics engine plugin
  std::string physicsEngine{""};

  /// \brief Render engine GUI plugin
  std::string renderEngineGui{""};

  /// \brief Render engine GUI API Backend
  std::string renderEngineGuiApiBackend{""};

  /// \brief Render engine Server plugin
  std::string renderEngineServer{""};

  /// \brief Render engine Server API Backend
  std::string renderEngineServerApiBackend{""};

  /// \brief Enable headless rendering
  int headlessRendering{0};

  /// \brief Show the world loading menu
  int waitGui{0};

  /// \brief Custom seed to random value generator
  int seed{0};

  /// \brief Verbosity level
  int verbosity{-1};

  /// \brief Path to GUI configuration file
  std::string guiConfig{""};

  /// \brief Flag to launch the Server
  bool launchServer{false};

  /// \brief Flag to launch the GUI
  bool launchGui{false};

  /// \brief A value of 1 (true) will wait for simulation assets to
  /// download before starting simulation. A value of 0 (false) will
  /// download simulation assets in a separate thread.
  int waitForAssets{0};
};

#ifdef WITH_GUI
//////////////////////////////////////////////////
/// \brief Launch an executable as a separate process
int launchProcess(
  const std::string &_executable,
  std::vector<std::string> _args)
{
  std::ostringstream command;
  command << _executable;
  for(const auto &val : _args)
  {
    command << " " << val;
  }

  #ifdef _WIN32
    STARTUPINFO si;
    PROCESS_INFORMATION pi;

    ZeroMemory(&si, sizeof(si));
    si.cb = sizeof(si);

    ZeroMemory(&pi, sizeof(pi));

    std::wstring w_command = std::wstring(command.str().begin(),
                                          command.str().end());
    LPWSTR cmd_line = const_cast<LPWSTR>(w_command.c_str());

    if(!CreateProcessW(NULL, cmd_line, NULL, NULL, FALSE, 0,
                      NULL, NULL, &si, &pi))
    {
      return -1;
    }

    WaitForSingleObject(pi.hProcess, INFINITE);

    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    return 0;
  #else
    return std::system(command.str().c_str());
  #endif
}

//////////////////////////////////////////////////
/// \brief Generate list of arguments for GUI command
std::vector<std::string> createGuiCommand(
  std::shared_ptr<SimOptions> _opt)
{
  std::vector<std::string> args;

  if(_opt->verbosity >= 0)
  {
    args.push_back("--verbose");
    args.push_back(std::to_string(_opt->verbosity));
  }

  if(!_opt->renderEngineGui.empty())
  {
    args.push_back("--render-engine-gui");
    args.push_back(_opt->renderEngineGui);
  }

  if(!_opt->renderEngineGuiApiBackend.empty())
  {
    args.push_back("--render-engine-gui-api-backend");
    args.push_back(_opt->renderEngineGuiApiBackend);
  }

  if(!_opt->guiConfig.empty())
  {
    args.push_back("--gui-config");
    args.push_back(_opt->guiConfig);
  }

  if(!_opt->file.empty())
  {
    args.push_back(_opt->file);
  }

  return args;
}
#endif

//////////////////////////////////////////////////
void addSimFlags(CLI::App &_app, std::shared_ptr<SimOptions> _opt)
{
  _app.add_option("file", _opt->file,
                  "Name of the SDFormat file.");

  _app.add_option("--initial-sim-time", _opt->initialSimTime,
                  "Initial simulation time, in seconds.");

  _app.add_option("--iterations", _opt->iterations,
                  "Number of iterations to execute.");

  _app.add_flag("--levels", _opt->levels,
                "Use the level system. "
                "The default is false which loads all models. \n"
                "It is always true with --network-role.");

  auto networkRoleOpt = _app.add_option_function<std::string>("--network-role",
    [_opt](const std::string &_networkRole){
      _opt->levels = 1;
      _opt->networkRole = _networkRole;
    },
    "Participant role used in distributed simulation environment.\n")
    ->check(CLI::IsMember({"primary", "secondary"}));

  _app.add_option_function<int>("--network-secondaries",
    [_opt](const int _networkSecondaries){
      if(_opt->networkRole != "primary") {
        throw CLI::ValidationError(
          "--network-secondaries",
          "Can only be used when --network-role primary.");
      }
      _opt->networkSecondaries = _networkSecondaries;
    },
    "Number of secondary participants expected\n"
    "to join a distributed simulation environment.")
    ->needs(networkRoleOpt);

  _app.add_flag("--record", _opt->record,
                "Use logging system to record states and console\n"
                "messages to the default location, ~/.gz/sim/log.");

  _app.add_option_function<std::string>("--record-path",
    [_opt](const std::string &_recordPath){
      _opt->record = 1;
      _opt->recordPath = _recordPath;
    },
    "Specifies a custom path to store recorded files.\n"
    "This will enable console logging to a console.log\n"
    "file in the specified path.\n"
    "Note: Implicitly invokes --record");

  _app.add_flag_callback("--record-resources",
    [_opt](){
      _opt->record = 1;
      _opt->recordResources = 1;
    },
    "Records meshes and material files in addition to\n"
    "states and console messages.\n"
    "Note: Implicitly invokes --record");

  _app.add_option_function<std::vector<std::string>>("--record-topic",
    [_opt](const std::vector<std::string> &_recordTopics){
      _opt->record = 1;
      _opt->recordTopics = _recordTopics;
    },
    "Specify the name of an additional topic to record. Zero or\n"
    "more topics can be specified by using multiple --record-topic\n"
    "options. Regular expressions can be used, likely requiring quotes.\n"
    "A default set of topics are also recorded, which support simulation\n"
    "state feedback. Enable debug console output with the -v 4 option and\n"
    "look for 'Recording default topic' in order to determine the default\n"
    "set of topics.\n"
    "Examples:\n"
    "   1. Record all topics.\n"
    "       --record-topic \".*\"\n"
    "   2. Record only the /stats topic.\n"
    "       --record-topic /stats\n"
    "   3. Record the /stats and /clock topics.\n"
    "       --record-topic /stats --record-topic /clock\n"
    "Note: Implicitly invokes --record.");

  _app.add_option("--record-period", _opt->recordPeriod,
                  "Specify the time period (seconds) between\n"
                  "state recording.");

  _app.add_flag("--log-overwrite", _opt->logOverwrite,
                "Overwrite existing files when recording.\n"
                "Only valid if recording is enabled.");

  _app.add_flag("--log-compress", _opt->logCompress,
                "Compress final log when recording.\n"
                "Only valid if recording is enabled.");

  _app.add_option("--seed", _opt->seed,
                  "Pass a custom seed value to the random\n"
                  "number generator.");

  _app.add_option("--physics-engine", _opt->physicsEngine,
                  "Gazebo Physics engine plugin to load. Gazebo\n"
                  "will use DART by default (gz-physics-dartsim-plugin)\n"
                  "Make sure custom plugins are inside\n"
                  "GZ_SIM_PHYSICS_ENGINE_PATH.");
  #ifdef WITH_GUI
  _app.add_option("--render-engine-gui", _opt->renderEngineGui,
                  "Gazebo Rendering engine plugin to load for the GUI.\n"
                  "Gazebo will use OGRE2 by default. Make sure custom\n"
                  "plugins are in GZ_SIM_RENDER_ENGINE_PATH.")
                  ->default_str("ogre2");

  _app.add_option("--render-engine-gui-api-backend",
                  _opt->renderEngineGuiApiBackend,
                  "Same as --render-engine-api-backend but only\n"
                  "for the GUI.")
                  ->check(CLI::IsMember({"opengl", "vulkan", "metal"}));
  #endif

  _app.add_option("--render-engine-server", _opt->renderEngineServer,
                  "Gazebo Rendering engine plugin to load for the Server.\n"
                  "Gazebo will use OGRE2 by default. Make sure custom\n"
                  "plugins are in GZ_SIM_RENDER_ENGINE_PATH.")
                  ->default_str("ogre2");

  _app.add_option("--render-engine-server-api-backend",
                  _opt->renderEngineServerApiBackend,
                  "Same as --render-engine-api-backend but only\n"
                  "for the Server.")
                  ->check(CLI::IsMember({"opengl", "vulkan", "metal"}));

  _app.add_option_function<std::string>("--render-engine",
    [_opt](const std::string &_renderEngine){
      _opt->renderEngineGui = _renderEngine;
      _opt->renderEngineServer = _renderEngine;
    },
    "Gazebo Rendering engine plugin to load for both the Server\n"
    "and the GUI. Gazebo will use OGRE2 by default.\n"
    "Make sure custom plugins are inside\n"
    "GZ_SIM_RENDER_ENGINE_PATH.")
    ->default_str("ogre2");

  _app.add_option_function<std::string>("--render-engine-api-backend",
    [_opt](const std::string &_renderEngineApiBackend){
      _opt->renderEngineGuiApiBackend = _renderEngineApiBackend;
      _opt->renderEngineServerApiBackend = _renderEngineApiBackend;
    },
    "API to use for both Server and GUI.\n"
    "Possible values for ogre2:\n"
    "   - opengl (default)\n"
    "   - vulkan (beta)\n"
    "   - metal (Apple only. Default for Apple)\n"
    "Note: If Vulkan is being in the GUI and gz-gui was\n"
    "built against Qt < 5.15.2, it may be very slow")
    ->check(CLI::IsMember({"opengl", "vulkan", "metal"}));

  _app.add_flag("--headless-rendering", _opt->headlessRendering,
                "Run rendering in headless mode.");

  _app.add_flag("-r", _opt->runOnStart,
                "Run simulation on start.");

  _app.add_option("-z", _opt->rate,
                "Update rate in hertz.");

  #ifdef WITH_GUI
  _app.add_option("--gui-config", _opt->guiConfig,
                  "Gazebo GUI configuration file to load.\n"
                  "If no file is provided then the configuration in\n"
                  "SDF file is used. If that is also missing then\n"
                  "the default installed configuration is used.");
  #endif

  _app.add_option_function<std::string>("--playback",
    [_opt](const std::string &_playback){
      _opt->playback = _playback;

      if(_opt->guiConfig.empty())
      {
        _opt->guiConfig = "_playback_";
      }
    },
    "Use the logging system to play back states.\n"
    "Argument is the path to recorded states.");

  _app.add_flag("--wait-for-assets", _opt->waitForAssets,
                "Wait assets before starting simulation.");
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Run and manage Gazebo simulations."};

  auto opt = std::make_shared<SimOptions>();

  app.add_flag_callback("--version",
    [](){
      std::cout << GZ_SIM_VERSION_FULL << std::endl;
      throw CLI::Success();
    },
    "Print the current library version.");

  app.add_option_function<int>("-v,--verbose",
    [opt](const int _verbosity){
      cmdVerbosity(_verbosity);
      opt->verbosity = _verbosity;
    },
    "Adjust the level of console output (0~4).\n"
    "The default verbosity level is 1. Use -v\n"
    "without arguments for level 3.\n")
    ->expected(0, 1)
    ->default_val(3);

  // Dummy flags handled by gz-tools
  app.add_flag("--force-version", "Use a particular library version.");
  app.add_flag("--versions", "Show the available versions.");

  #ifdef WITH_GUI
  app.add_flag("-g", opt->launchGui, "Run and manage only the Gazebo GUI");
  #endif

  app.add_flag_callback("-s",
    [opt]{
      opt->launchServer = true;
      opt->launchGui = false;
    },
    "Run and manage only the Gazebo Server (headless mode).\n"
    "This overrides -g, if it is also present.");

  app.footer("Environment variables:\n"
    "   GZ_SIM_RESOURCE_PATH          Colon separated paths used to locate\n"
    "                                 resources such as worlds and models.\n"
    "   GZ_SIM_SYSTEM_PLUGIN_PATH     Colon separated paths used to locate\n"
    "                                 system plugins.\n"
    "   GZ_SIM_SERVER_CONFIG_PATH     Path to server configuration file.\n"
    "   GZ_GUI_PLUGIN_PATH            Colon separated paths used to locate\n"
    "                                 GUI plugins.\n"
    "   GZ_GUI_RESOURCE_PATH          Colon separated paths used to locate\n"
    "                                 resources such as configuration files.");

  addSimFlags(app, opt);

  app.formatter(std::make_shared<GzFormatter>(&app));
  CLI11_PARSE(app, argc, argv);

  std::string parsedSdfFile;
  if(!opt->launchServer && !opt->launchGui)
  {
    // If file was passed, check SDF file and parse into string
    if (opt->file != "") {
      if(checkFile(opt->file) < 0)
        return -1;

      if(parseSdfFile(opt->file, parsedSdfFile) < 0)
        return -1;
    }

    bool blocking = true;

    #ifdef WITH_GUI
    opt->waitGui = 1;
    blocking = false;

    // Launch the GUI in a separate thread
    std::thread guiThread(
      [opt]{
        utils::setenv(
            std::string("GZ_SIM_WAIT_GUI"),
            std::to_string(opt->waitGui));
        launchProcess(std::string(GZ_SIM_GUI_EXE), createGuiCommand(opt));
      });
    #endif

    // Create a Gazebo server configuration
    sim::ServerConfig serverConfig;
    if (createServerConfig(serverConfig, parsedSdfFile.c_str(),
              opt->rate, opt->initialSimTime, opt->levels,
              opt->networkRole.c_str(), opt->networkSecondaries,
              opt->record, opt->recordPath.c_str(), opt->recordResources,
              opt->logOverwrite, opt->logCompress, opt->playback.c_str(),
              opt->physicsEngine.c_str(), opt->renderEngineServer.c_str(),
              opt->renderEngineServerApiBackend.c_str(),
              opt->renderEngineGui.c_str(),
              opt->renderEngineGuiApiBackend.c_str(), opt->file.c_str(),
              opt->recordTopics, opt->waitGui, opt->headlessRendering,
              opt->recordPeriod, opt->seed, opt->waitForAssets) != 0)
    {
      return -1;
    }

    // Run the server in a separate thread
    sim::Server server(serverConfig);
    server.Run(blocking, opt->iterations, opt->runOnStart == 0);

    #ifdef WITH_GUI
    // Join the GUI thread to wait for a possible window close
    guiThread.join();

    // Shutdown server if the GUI has been closed from the screen
    if(server.Running())
    {
      server.Stop();
    }
    #endif

    gzdbg << "Shutting down gz-sim-server" << std::endl;
  }
  else
  {
    if(opt->launchServer)
    {
      if (opt->file != "") {
        // Check SDF file and parse into string
        if(checkFile(opt->file) < 0)
          return -1;

        if(parseSdfFile(opt->file, parsedSdfFile) < 0)
          return -1;
      }

      // Create a Gazebo server configuration
      sim::ServerConfig serverConfig;
      if (createServerConfig(serverConfig, parsedSdfFile.c_str(),
                opt->rate, opt->initialSimTime, opt->levels,
                opt->networkRole.c_str(), opt->networkSecondaries,
                opt->record, opt->recordPath.c_str(), opt->recordResources,
                opt->logOverwrite, opt->logCompress, opt->playback.c_str(),
                opt->physicsEngine.c_str(), opt->renderEngineServer.c_str(),
                opt->renderEngineServerApiBackend.c_str(),
                opt->renderEngineGui.c_str(),
                opt->renderEngineGuiApiBackend.c_str(), opt->file.c_str(),
                opt->recordTopics, opt->waitGui, opt->headlessRendering,
                opt->recordPeriod, opt->seed, opt->waitForAssets) != 0)
      {
        return -1;
      }

      // Run the server in the main thread
      sim::Server server(serverConfig);
      server.Run(true, opt->iterations, opt->runOnStart == 0);

      gzdbg << "Shutting down gz-sim-server" << std::endl;
    }
    else if(opt->launchGui)
    {
      #ifdef WITH_GUI
      launchProcess(std::string(GZ_SIM_GUI_EXE), createGuiCommand(opt));
      #else
      std::cerr << "This version of Gazebo does not support GUI" << std::endl;
      #endif
    }
  }

  return 0;
}
