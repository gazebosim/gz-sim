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

#include <gz/utils/cli/CLI.hpp>
#include <gz/utils/cli/GzFormatter.hpp>

#include "gz/sim/config.hh"
#include "gz.hh"

//////////////////////////////////////////////////
enum class SimCommand
{
  kNone,
  kSimServer,
  kSimGui,
  kSimComplete,
};

//////////////////////////////////////////////////
struct SimOptions
{
  SimCommand command{SimCommand::kNone};

  std::string filename{""};

  int runGui{0};

  int rate{-1};

  double initialSimTime{0.0};

  int iterations{0};

  int levels{0};

  std::string networkRole{""};

  int networkSecondaries{0};

  int record{0};

  std::string recordPath{""};

  int recordResources{0};

  std::vector<std::string> recordTopics{};

  float recordPeriod{-1};

  int logOverwrite{0};

  int logCompress{0};

  std::string playback{""};

  int runOnStart{0};

  int runServer{0};

  std::string guiConfig{""};

  std::string physicsEngine{""};

  std::string renderEngineGui{""};

  std::string renderEngineGuiApiBackend{""};

  std::string renderEngineServer{""};

  std::string renderEngineServerApiBackend{""};

  int headlessRendering{0};

  int waitGui{1};

  int seed{1};
};

//////////////////////////////////////////////////
void runSimCommand(const SimOptions &_opt)
{
  switch(_opt.command)
  {
    case SimCommand::kSimServer:
      runServer(nullptr, _opt.iterations, _opt.runOnStart, _opt.rate,
                _opt.initialSimTime, _opt.levels, _opt.networkRole.c_str(),
                _opt.networkSecondaries, _opt.record, _opt.recordPath.c_str(),
                _opt.recordResources, _opt.logOverwrite, _opt.logCompress,
                _opt.playback.c_str(), _opt.physicsEngine.c_str(),
                _opt.renderEngineServer.c_str(),
                _opt.renderEngineServerApiBackend.c_str(),
                _opt.renderEngineGui.c_str(),
                _opt.renderEngineGuiApiBackend.c_str(), _opt.filename.c_str(),
                _opt.recordTopics, _opt.waitGui, _opt.headlessRendering,
                _opt.recordPeriod, _opt.seed);
      break;
    case SimCommand::kSimGui:
    case SimCommand::kSimComplete:
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

  auto filenameOpt =
    _app.add_option_function<std::string>("filename",
      [opt](const std::string &_filename){
        opt->filename = _filename;
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

  auto recordCmd = _app.add_flag("--record", opt->record,
                "Use logging system to record states and console\n"
                "to default location, ~/.gz/sim/log.");

  _app.add_option_function<std::string>("--record-path",
    [opt](const std::string &_recordPath){
      opt->record = 1;
      opt->recordPath = _recordPath;
    },
    "Specifies a custom path to put recorded files.\n"
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
    "       --record-topic /stats \\n"
    "       --record-topic /clock\n"
    "Note: Implicitly invokes--record.");

  _app.add_option("--record-period", opt->recordPeriod,
                  "Specify the time period (seconds) between\n"
                  "state recording.")
                  ->needs(recordCmd);

  _app.add_flag("--log-overwrite", opt->logOverwrite,
                "Overwrite existing files when recording.")
                ->needs(recordCmd);

  _app.add_flag("--log-compress", opt->logCompress,
                "Compress final log when recording.")
                ->needs(recordCmd);

  _app.add_option("--seed", opt->seed,
                  "Pass a custom seed value to the random\n"
                  "number generator.");

  _app.add_option("--playback", opt->playback,
                  "Use the logging system to play back states.\n"
                  "Argument is the path to recorded states.");

  _app.add_flag("--headless-rendering", opt->headlessRendering,
                "Run rendering in headless mode.");

  _app.add_flag("-r", opt->runOnStart,
                "Run simulation on start.");

  _app.add_flag("-z", opt->rate,
                "Update rate in hertz.");

  auto command = _app.add_option_group("command", "Command to be executed");

  auto runGuiCmd = command->add_flag_callback("-g",
    [opt]() {
      opt->command = SimCommand::kSimGui;
    },
    "Run only the GUI.");

  auto runServerCmd = command->add_flag_callback("-s",
    [opt](){
      opt->command = SimCommand::kSimServer;
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

  addSimFlags(app);
  app.formatter(std::make_shared<GzFormatter>(&app));
  CLI11_PARSE(app, argc, argv);
}
