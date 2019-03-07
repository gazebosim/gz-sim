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
#include <gflags/gflags.h>

#include <ignition/common/Console.hh>

#include <iostream>

#include "ignition/gazebo/config.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"

// Gflag command line argument definitions
// This flag is an abbreviation for the longer gflags built-in help flag.
DEFINE_bool(h, false, "");
DEFINE_int32(verbose, 1, "");
DEFINE_int32(v, 1, "");
DEFINE_double(z, -1, "Update rate in Hertz.");
DEFINE_uint64(iterations, 0, "Number of iterations to execute.");
DEFINE_string(f, "", "Load an SDF file on start.");
DEFINE_bool(r, false, "Run simulation on start. "
    "The default is false, which starts simulation paused.");
DEFINE_bool(levels, false, "Use levels");
DEFINE_bool(distributed, false, "Use distributed simulation.");

//////////////////////////////////////////////////
void help()
{
  std::cout
  << "ign-gazebo-server -- Run the Gazebo server (headless mode)." << std::endl
  << std::endl
  << "`ign-gazebo-server` [options]" << std::endl
  << std::endl
  << std::endl
  << "Options:" << std::endl
  << "  -h [ --help ]          Print help message."
  << std::endl
  << "  --version              Print version information."
  << std::endl
  << "  -v [--verbose] arg     Adjust the level of console output (0~4)."
  << " The default verbosity is 1"
  << std::endl
  << "  --iterations arg       Number of iterations to execute."
  << std::endl
  << "  -f arg                 Load an SDF file on start. "
  << std::endl
  << "  -z arg                 Update rate in Hertz."
  << std::endl
  << "  -r                     Run simulation on start."
  << " The default is false, which starts simulation paused."
  << std::endl
  << "  --levels               Use the level system."
  << " The default is false, which loads all models."
  << std::endl
  << "  --distributed          Use the distributed simulation system."
  << " The default is false, which disables all distributed simulation."
  << std::endl
  << "Environment variables:" << std::endl
  << "  IGN_GAZEBO_RESOURCE_PATH    Colon separated paths used to locate "
  << " resources. Can be useful with the -f option to find an SDF file."
  << std::endl
  << "  IGN_GAZEBO_NETWORK_ROLE     Participant role used in a distributed "
  << " simulation environment. Role is one of [PRIMARY, SECONDARY]."
  << std::endl
  << "  IGN_GAZEBO_NETWORK_SECONDARIES    Number of secondary participants "
  << " expected to join a distributed simulation environment. (Primary only)"
  << std::endl;
}

//////////////////////////////////////////////////
void version()
{
  std::cout << IGNITION_GAZEBO_VERSION_HEADER << std::endl;
}

//////////////////////////////////////////////////
static bool VerbosityValidator(const char */*_flagname*/, int _value)
{
  return _value >= 0 && _value <= 4;
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Register validators
  gflags::RegisterFlagValidator(&FLAGS_verbose, &VerbosityValidator);
  gflags::RegisterFlagValidator(&FLAGS_v, &VerbosityValidator);

  // Parse command line
  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineNonHelpFlags(&_argc, &_argv, true);

  // Hold info as we parse it
  gflags::CommandLineFlagInfo info;

  // Help
  // Parse out the help flag in such a way that the full help text
  // is suppressed: if --help* or -h is specified, override the default
  // help behavior and turn on --helpmatch, to only shows help for the
  // current executable (instead of showing a huge list of gflags built-ins).
  gflags::GetCommandLineFlagInfo("help", &info);
  bool showHelp = FLAGS_h || (info.current_value == "true");

  // Version
  gflags::GetCommandLineFlagInfo("version", &info);
  bool showVersion = (info.current_value == "true");

  // Verbosity
  gflags::GetCommandLineFlagInfo("verbose", &info);
  if (info.is_default)
  {
    gflags::GetCommandLineFlagInfo("v", &info);
    if (!info.is_default)
      FLAGS_verbose = FLAGS_v;
    else
      FLAGS_verbose = 1;
  }

  // If help message is requested, substitute in the override help function.
  if (showHelp)
  {
    gflags::SetCommandLineOption("help", "false");
    gflags::SetCommandLineOption("helpshort", "false");
    gflags::SetCommandLineOption("helpfull", "false");
    gflags::SetCommandLineOption("helpmatch", "");
    help();
    return 0;
  }

  // If version is requested, override with custom version print function.
  if (showVersion)
  {
    gflags::SetCommandLineOption("version", "false");
    version();
    return 0;
  }

  // Set verbosity
  ignition::common::Console::SetVerbosity(FLAGS_verbose);
  ignmsg << "Ignition Gazebo Server v" << IGNITION_GAZEBO_VERSION_FULL
         << std::endl;

  ignition::gazebo::ServerConfig serverConfig;
  if (!serverConfig.SetSdfFile(FLAGS_f))
  {
    ignerr << "Failed to set SDF file [" << FLAGS_f << "]" << std::endl;
    return -1;
  }

  // Set the update rate.
  if (FLAGS_z > 0.0)
    serverConfig.SetUpdateRate(FLAGS_z);

  if (FLAGS_levels)
  {
    ignmsg << "Using the level system\n";
    serverConfig.SetUseLevels(true);
  }

  if (FLAGS_distributed)
  {
    ignmsg << "Using the distributed simulation system\n";
    serverConfig.SetUseDistributedSimulation(true);
  }

  // Create the Gazebo server
  ignition::gazebo::Server server(serverConfig);

  // Run the server
  server.Run(true, FLAGS_iterations, !FLAGS_r);

  igndbg << "Shutting down ign-gazebo-server" << std::endl;
  return 0;
}
