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

#include <gz/utils/cli/CLI.hpp>
#include <gz/utils/cli/GzFormatter.hpp>
#include <gz/utils/Environment.hh>

#include "gz/sim/config.hh"
#include "gz.hh"

using namespace gz;

//////////////////////////////////////////////////
/// \brief Structure to hold all available Server launch options
struct ServerOptions
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
  int waitGui{1};

  /// \brief Custom seed to random value generator
  int seed{1};
};

//////////////////////////////////////////////////
void addServerFlags(CLI::App &_app)
{
  auto opt = std::make_shared<ServerOptions>();

  _app.add_option_function<std::string>("file",
    [opt](const std::string &_file){
      opt->file = _file;
    },
    "Name of the SDFormat file.");

  _app.add_option("--initial-sim-time", opt->initialSimTime,
                  "Initial simulation time, in seconds.");

  _app.add_option("--iterations", opt->iterations,
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

  _app.callback([opt](){

    // Check SDF file and parse into string
    if(checkFile(opt->file) < 0)
      return;

    std::string parsedSdfFile;
    if(parseSdfFile(opt->file, parsedSdfFile) < 0)
      return;

    // Get verbosity level from environment
    std::string verbosity;
    if(utils::env("GZ_SIM_VERBOSITY", verbosity))
    {
      cmdVerbosity(std::stoi(verbosity));
    }

    // Get flag to display Quickstart menu
    std::string waitGui;
    if(utils::env("GZ_SIM_WAIT_GUI", waitGui) && opt->file.empty())
    {
      opt->waitGui = std::stoi(waitGui);
    }

    runServer(parsedSdfFile.c_str(), opt->iterations, opt->runOnStart,
              opt->rate, opt->initialSimTime, opt->levels,
              opt->networkRole.c_str(), opt->networkSecondaries,
              opt->record, opt->recordPath.c_str(), opt->recordResources,
              opt->logOverwrite, opt->logCompress, opt->playback.c_str(),
              opt->physicsEngine.c_str(), opt->renderEngineServer.c_str(),
              opt->renderEngineServerApiBackend.c_str(),
              opt->renderEngineGui.c_str(),
              opt->renderEngineGuiApiBackend.c_str(), opt->file.c_str(),
              opt->recordTopics, opt->waitGui, opt->headlessRendering,
              opt->recordPeriod, opt->seed);
  });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Run and manage Gazebo Server."};

  app.allow_extras();

  addServerFlags(app);
  app.formatter(std::make_shared<GzFormatter>(&app));
  CLI11_PARSE(app, argc, argv);
}
