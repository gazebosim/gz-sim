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

#include <gz/utils/cli/CLI.hpp>
#include <gz/utils/cli/GzFormatter.hpp>

#include "gz/sim/config.hh"
#include "gz.hh"

//////////////////////////////////////////////////
/// \brief Enumeration of available sim commands
enum class SimCommand
{
  kNone,
  kSimServer,
  kSimGui,
  kSimComplete,
};

//////////////////////////////////////////////////
/// \brief Structure to hold all available sim options
struct SimOptions
{
  /// \brief Command to execute
  SimCommand command{SimCommand::kNone};

  /// \brief Name of the SDFormat file
  std::string file{""};

  /// \brief Update rate in hertz
  int rate{-1};

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

  /// \brief Path to GUI configuration file
  std::string guiConfig{""};

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
  int waitGui{1};

  /// \brief Custom seed to random value generator
  int seed{1};
};

//////////////////////////////////////////////////
/// \brief Callback fired when options are successfully parsed
void runSimCommand(SimOptions &_opt)
{
  switch(_opt.command)
  {
    case SimCommand::kSimServer:
      {
        if(checkFile(_opt.file) < 0)
          return;

        std::string parsedSdfFile;
        if(parseSdfFile(_opt.file, parsedSdfFile) < 0)
          return;

        runServer(parsedSdfFile.c_str(), _opt.iterations, _opt.runOnStart, _opt.rate,
                  _opt.initialSimTime, _opt.levels, _opt.networkRole.c_str(),
                  _opt.networkSecondaries, _opt.record, _opt.recordPath.c_str(),
                  _opt.recordResources, _opt.logOverwrite, _opt.logCompress,
                  _opt.playback.c_str(), _opt.physicsEngine.c_str(),
                  _opt.renderEngineServer.c_str(),
                  _opt.renderEngineServerApiBackend.c_str(),
                  _opt.renderEngineGui.c_str(),
                  _opt.renderEngineGuiApiBackend.c_str(), _opt.file.c_str(),
                  _opt.recordTopics, _opt.waitGui, _opt.headlessRendering,
                  _opt.recordPeriod, _opt.seed);
      }
      break;
    case SimCommand::kSimGui:
      {
        if(checkFile(_opt.file) < 0)
          return;

        runGui(_opt.guiConfig.c_str(), _opt.file.c_str(), _opt.waitGui,
               _opt.renderEngineGui.c_str(),
               _opt.renderEngineGuiApiBackend.c_str());
      }
      break;
    case SimCommand::kSimComplete:
      {
        if(checkFile(_opt.file) < 0)
          return;

        std::string parsedSdfFile;
        if(parseSdfFile(_opt.file, parsedSdfFile) < 0)
          return;

        std::thread serverThread([_opt, parsedSdfFile]{
          runServer(parsedSdfFile.c_str(), _opt.iterations, _opt.runOnStart, _opt.rate,
                    _opt.initialSimTime, _opt.levels, _opt.networkRole.c_str(),
                    _opt.networkSecondaries, _opt.record,
                    _opt.recordPath.c_str(), _opt.recordResources,
                    _opt.logOverwrite, _opt.logCompress, _opt.playback.c_str(),
                    _opt.physicsEngine.c_str(), _opt.renderEngineServer.c_str(),
                    _opt.renderEngineServerApiBackend.c_str(),
                    _opt.renderEngineGui.c_str(),
                    _opt.renderEngineGuiApiBackend.c_str(),
                    _opt.file.c_str(), _opt.recordTopics, _opt.waitGui,
                    _opt.headlessRendering, _opt.recordPeriod, _opt.seed);

          // Server and GUI handle SIGINT independently.
          // When the first SIGINT is received, only one of the two processes
          // may handle it and exit while the other continues waiting.
          // To ensure both processes terminate properly, a second SIGINT is sent.
          // A short delay is added to prevent overwhelming the signal handler.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          kill(getpid(), SIGINT);
        });

        std::thread guiThread([_opt]{
          runGui(_opt.guiConfig.c_str(), _opt.file.c_str(), _opt.waitGui,
                _opt.renderEngineGui.c_str(),
                _opt.renderEngineGuiApiBackend.c_str());

          // Server and GUI handle SIGINT independently.
          // When the first SIGINT is received, only one of the two processes
          // may handle it and exit while the other continues waiting.
          // To ensure both processes terminate properly, a second SIGINT is sent.
          // A short delay is added to prevent overwhelming the signal handler.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          kill(getpid(), SIGINT);
        });

        if(guiThread.joinable()) { guiThread.join(); }
        if(serverThread.joinable()) { serverThread.join(); }
      }
      break;
    case SimCommand::kNone:
    default:
      throw CLI::CallForHelp();
  }
}

//////////////////////////////////////////////////
void addSimFlags(CLI::App &_app)
{
  auto opt = std::make_shared<SimOptions>();

  opt->command = SimCommand::kSimComplete;

  _app.add_option_function<std::string>("file",
    [opt](const std::string &_file){
      opt->file = _file;
    },
    "Name of the SDFormat file.");

  _app.add_option<double>("--initial-sim-time", opt->initialSimTime,
                          "Initial simulation time, in seconds.");

  _app.add_option<int>("--iterations", opt->iterations,
                       "Number of iterations to execute.");

  _app.add_flag("--levels", opt->levels,
                "Use the level system. "
                "The default is false which loads all models. \n"
                "It is always true with --network-role.");

  auto networkRoleOpt = _app.add_option_function<std::string>("--network-role",
    [opt](const std::string &_networkRole){
      opt->levels = 1;
      opt->networkRole = _networkRole;
    },
    "Participant role used in distributed simulation environment.\n")
    ->check(CLI::IsMember({"primary", "secondary"}));

  _app.add_option_function<int>("--network-secondaries",
    [opt](const int _networkSecondaries){
      if(opt->networkRole != "primary") {
        throw CLI::ValidationError(
          "--network-secondaries",
          "Can only be used when --network-role primary.");
      }
      opt->networkSecondaries = _networkSecondaries;
    },
    "Number of secondary participants expected\n"
    "to join a distributed simulation environment.")
    ->needs(networkRoleOpt);

  _app.add_flag("--record", opt->record,
                "Use logging system to record states and console\n"
                "to default location, ~/.gz/sim/log.");

  _app.add_option_function<std::string>("--record-path",
    [opt](const std::string &_recordPath){
      opt->record = 1;
      opt->recordPath = _recordPath;
    },
    "Specifies a custom path to store recorded files.\n"
    "This will enable console logging to a console.log\n"
    "file in the specified path.\n"
    "Note: Implicitly invokes --record");

  _app.add_flag_callback("--record-resources",
    [opt](){
      opt->record = 1;
      opt->recordResources = 1;
    },
    "Records meshes and material files in addition to\n"
    "states and console messages.\n"
    "Note: Implicitly invokes --record");

  _app.add_option_function<std::vector<std::string>>("--record-topic",
    [opt](const std::vector<std::string> &_recordTopics){
      opt->record = 1;
      opt->recordTopics = _recordTopics;
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
    "       --record-topic /stats --record-topic /clock"
    "Note: Implicitly invokes --record.");

  _app.add_option("--record-period", opt->recordPeriod,
                  "Specify the time period (seconds) between\n"
                  "state recording.");

  _app.add_flag("--log-overwrite", opt->logOverwrite,
                "Overwrite existing files when recording.\n"
                "Only valid if recording is enabled.");

  _app.add_flag("--log-compress", opt->logCompress,
                "Compress final log when recording.\n"
                "Only valid if recording is enabled.");

  _app.add_option("--seed", opt->seed,
                  "Pass a custom seed value to the random\n"
                  "number generator.");

  _app.add_option("--gui-config", opt->guiConfig,
                  "Gazebo GUI configuration file to load.\n"
                  "If no file is provided then the configuration in\n"
                  "SDF file is used. If that is also missing then\n"
                  "the default installed configuration is used.");

  _app.add_option_function<std::string>("--playback",
    [opt](const std::string &_playback){
      if(opt->guiConfig.empty())
      {
        opt->guiConfig = "_playback_";
      }
      opt->playback = _playback;
    },
    "Use the logging system to play back states.\n"
    "Argument is the path to recorded states.");

  _app.add_option("--physics-engine", opt->physicsEngine,
                  "Gazebo Physics engine plugin to load. Gazebo\n"
                  "will use DART by default (gz-physics-dartsim-plugin)\n"
                  "Make sure custom plugins are inside\n"
                  "GZ_SIM_PHYSICS_ENGINE_PATH.");

  _app.add_option("--render-engine-gui", opt->renderEngineGui,
                  "Gazebo Rendering engine plugin to load for the GUI.\n"
                  "Gazebo will use OGRE2 by default. Make sure custom\n"
                  "plugins are in GZ_SIM_RENDER_ENGINE_PATH.")
                  ->default_str("ogre2");

  _app.add_option("--render-engine-gui-api-backend",
                  opt->renderEngineGuiApiBackend,
                  "Same as --render-engine-api-backend but only\n"
                  "for the GUI.")
                  ->check(CLI::IsMember({"opengl", "vulkan", "metal"}));

  _app.add_option("--render-engine-server", opt->renderEngineServer,
                  "Gazebo Rendering engine plugin to load for the Server.\n"
                  "Gazebo will use OGRE2 by default. Make sure custom\n"
                  "plugins are in GZ_SIM_RENDER_ENGINE_PATH.")
                  ->default_str("ogre2");

  _app.add_option("--render-engine-server-api-backend",
                  opt->renderEngineServerApiBackend,
                  "Same as --render-engine-api-backend but only\n"
                  "for the Server.")
                  ->check(CLI::IsMember({"opengl", "vulkan", "metal"}));

  _app.add_option_function<std::string>("--render-engine",
    [opt](const std::string &_renderEngine){
      opt->renderEngineGui = _renderEngine;
      opt->renderEngineServer = _renderEngine;
    },
    "Gazebo Rendering engine plugin to load for both the Server\n"
    "and the GUI. Gazebo will use OGRE2 by default.\n"
    "Make sure custom plugins are inside\n"
    "GZ_SIM_RENDER_ENGINE_PATH.")
    ->default_str("ogre2");

  _app.add_option_function<std::string>("--render-engine-api-backend",
    [opt](const std::string &_renderEngineApiBackend){
      opt->renderEngineGuiApiBackend = _renderEngineApiBackend;
      opt->renderEngineServerApiBackend = _renderEngineApiBackend;
    },
    "API to use for both Server and GUI.\n"
    "Possible values for ogre2:\n"
    "   - opengl (default)\n"
    "   - vulkan (beta)\n"
    "   - metal (Apple only. Default for Apple)\n"
    "Note: If Vulkan is being in the GUI and gz-gui was\n"
    "built against Qt < 5.15.2, it may be very slow")
    ->check(CLI::IsMember({"opengl", "vulkan", "metal"}));

  _app.add_flag("--headless-rendering", opt->headlessRendering,
                "Run rendering in headless mode.");

  _app.add_flag("-r", opt->runOnStart,
                "Run simulation on start.");

  _app.add_option("-z", opt->rate,
                "Update rate in hertz.");

  auto command = _app.add_option_group("command", "Command to be executed");

  command->add_flag_callback("-g",
    [opt]() {
      opt->command = SimCommand::kSimGui;
      opt->waitGui = 0;
    },
    "Run only the GUI.");

  command->add_flag_callback("-s",
    [opt](){
      opt->command = SimCommand::kSimServer;
      opt->waitGui = 0;
    },
    "Run only the server (headless mode).\n"
    "This overrides -g, if it is also present.");

  _app.callback([opt](){ runSimCommand(*opt); });
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

  addSimFlags(app);
  app.formatter(std::make_shared<GzFormatter>(&app));
  CLI11_PARSE(app, argc, argv);
}
