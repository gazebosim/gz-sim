/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>

#include <ignition/utils/cli/CLI.hpp>

#include "ign.hh"

#include "ignition/gazebo/config.hh"

/////////////////////////////////////////////////
std::string erbExec(std::string _cmd)
{
  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != nullptr)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

//////////////////////////////////////////////////
/// \brief Structure to hold available server options
struct ServerOptions
{
  public: int updateRate = 60;

  /// Server render engine name
  public: std::string renderEngineServer;

  /// GUI render engine name
  public: std::string renderEngineGUI;

  /// Physics engine name
  public: std::string physicsEngine;

  /// Set the network role, which is one of [primary, secondary]
  /// If primary is used, then make sure to also set the numer of
  /// network secondaries
  public: std::string networkRole;

  /// Number of network secondary servers that the
  /// primary server should expect.
  public: int networkSecondaries;

  /// Use logging system to record states and console messages
  public: bool record;

  /// Implicitly invokes --record, and specifies custom path to put recorded
  /// files
  public: std::string recordPath;

  /// Implicitly invokes --record, and records
  /// meshes and material files, in addition to
  /// states and console messages
  public: bool recordResources;

  /// Specify the name of an additional topic to record. Implicitly invokes
  /// --record
  public: std::string recordTopic;

  /// Overwrite existing log files
  public: bool logOverwrite;

  /// Compress final log files
  public: bool logCompress;

  /// Use logging system to play back states
  public: std::string playback;

  /// File to load by Ignition
  public: std::string file;

  /// Run simulation on start.
  public: bool runOnStart = false;

  /// Use the level system
  public: int levels;

  /// Number of iterations to execute
  public: int iterations;

  /// Verbosity level
  int verboseLevel = 1;
};

//////////////////////////////////////////////////
void runServerCommand(const ServerOptions &_opt)
{
  cmdVerbosity(_opt.verboseLevel);

  std::string path;
  std::string parsed;
  igndbg << "Loading " << _opt.file << std::endl;
  if (!_opt.file.empty())
  {
    if (ignition::common::exists(_opt.file))
    {
      path = _opt.file;
    }
    else
    {
      std::string resourcePathEnv;
      ignition::common::env("IGN_GAZEBO_RESOURCE_PATH", resourcePathEnv);
      if (!resourcePathEnv.empty())
      {
        auto resourcePaths = ignition::common::split(resourcePathEnv, ":");
        for (auto & resourcePath: resourcePaths)
        {
          std::string filePath = ignition::common::joinPaths(
              resourcePath, _opt.file);
          if (ignition::common::exists(filePath))
          {
            path = filePath;
            break;
          }
        }
      }
      if (path.empty())
      {
        path = ignition::common::joinPaths(
          std::string(worldInstallDir()), _opt.file);
        if (!ignition::common::exists(path))
        {
          path = std::string(findFuelResource(_opt.file.c_str()));
          if (path.empty())
          {
            ignerr << "Unable to find or download file "
                   << _opt.file << std::endl;
            exit(-1);
          }
        }
      }
    }
    std::string cmd = "erb -v ";
    cmd += " " + path;
    igndbg << "cmd: " << cmd << std::endl;
    parsed = erbExec(cmd);
  }

  runServer(parsed.c_str(), _opt.iterations, _opt.runOnStart, _opt.updateRate,
    _opt.levels, _opt.networkRole.c_str(), _opt.networkSecondaries,
    _opt.record, _opt.recordPath.c_str(), _opt.recordResources,
    _opt.logOverwrite, _opt.logCompress, _opt.playback.c_str(),
    _opt.physicsEngine.c_str(), _opt.renderEngineServer.c_str(),
    _opt.renderEngineGUI.c_str(), path.c_str(), _opt.recordTopic.c_str());
}

//////////////////////////////////////////////////
void addServerFlags(CLI::App &_app)
{
  auto opt = std::make_shared<ServerOptions>();

  _app.add_option("-v,--verbose",
                  opt->verboseLevel,
                  "Adjust the level of console output (0~4).\n"
                  "The default verbosity is 1, use -v without \n"
                  "arguments for level 3.");

  _app.add_option("-z",
                  opt->updateRate,
                  "Update rate in Hertz.");

  _app.add_flag_callback(
    "-r",
    [opt](){
      opt->runOnStart = true;
    },
    "Run simulation on start");

  _app.add_option("--render-engine-gui",
                  opt->renderEngineGUI,
                  "Ignition Rendering engine plugin to load for"
                  "the server. Gazebo will use OGRE2 by default."
                  "(ogre2)"
                  "Make sure custom plugins are in"
                  "IGN_GAZEBO_RENDER_ENGINE_PATH.");

  _app.add_option("--render-engine-server",
                  opt->renderEngineServer,
                  "Ignition Rendering engine plugin to load for"
                  "the server. Gazebo will use OGRE2 by default."
                  "(ogre2)"
                  "Make sure custom plugins are in"
                  "IGN_GAZEBO_RENDER_ENGINE_PATH.");

  _app.add_option("--physics-engine",
                  opt->physicsEngine,
                  "Ignition Physics engine plugin to load."
                  "Gazebo will use DART by default."
                  "(ignition-physics-dartsim-plugin)"
                  "Make sure custom plugins are in"
                  "IGN_GAZEBO_PHYSICS_ENGINE_PATH.");

  _app.add_option("--iterations",
                  opt->iterations,
                  "Number of iterations to execute.");

  _app.add_option("--network-role",
                  opt->networkRole,
                  "Participant role used in a distributed"
                  "simulation environment. Role is one of"
                  "[primary, secondary]. It implies --levels.");

  _app.add_flag_callback(
    "--network-secondaries",
    [opt](){
      opt->networkSecondaries = true;
    },
    "Number of secondary participants expected"
    "to join a distributed simulation"
    "environment. (Primary only).");

  _app.add_option("--playback",
                  opt->playback,
                  "Use logging system to play back states"
                  "Argument is path to recorded states");

  _app.add_flag_callback(
    "--log-compress",
    [opt](){
      opt->logCompress = true;
    },
    "When recording, compress final log files."
    "Only valid if recording is enabled.");

  _app.add_flag_callback(
    "--log-overwrite",
    [opt](){
      opt->logOverwrite = true;
    },
    "When recording, overwrite existing files."
    "Only valid if recording is enabled.");

  _app.add_option("--record-topic",
                  opt->recordTopic,
                  "Specify the name of an additional topic to"
                  "record. Implicitly invokes --record."
                  "Zero or more topics can be specified by"
                  "using multiple --record-topic options."
                  "Regular expressions can be used, which"
                  "likely requires quotes. A default set of"
                  "topics are also recorded, which support"
                  "simulation state playback. Enable debug"
                  "console output with the -v 4 option"
                  "and look for 'Recording default topic' in"
                  "order to determine the default set of"
                  "topics."
                  "Examples:"
                  "  1. Record all topics."
                  "      --record-topic \".*\""
                  "  2. Record only the /stats topic."
                  "      --record-topic /stats"
                  "  3. Record the /stats and /clock topics."
                  "      --record-topic /clock");

  _app.add_flag_callback(
    "--record-resources",
    [opt](){
      opt->recordResources = true;
    },
    "Implicitly invokes --record, and records"
    "meshes and material files, in addition to"
    "states and console messages.");

  _app.add_option("--record-path",
                  opt->recordPath,
                  "Implicitly invokes --record, and specifies"
                  "custom path to put recorded files. Argument"
                  "is path to record states and console"
                  "messages. Specifying this argument will"
                  "enable console logging to a console.log"
                  "file in the specified path.");

  _app.add_flag_callback(
    "--record",
    [opt](){
      opt->record = true;
    },
    "Use logging system to record states and"
    "console messages to the default location,"
    "in ~/.ignition/gazebo/log.");

  _app.add_option("file", opt->file, "launch_file")->take_last();

  _app.callback([opt](){
    runServerCommand(*opt);
  });
}

//////////////////////////////////////////////////
int main(int argc, char** argv)
{
  CLI::App app{"Start Ignition Gazebo Server"};

  app.set_help_all_flag("--help-all", "Show all help");

  app.add_flag_callback("--version", [](){
    std::cout << IGNITION_GAZEBO_VERSION_FULL << std::endl;
    throw CLI::Success();
  });

  addServerFlags(app);
  CLI11_PARSE(app, argc, argv);
}
