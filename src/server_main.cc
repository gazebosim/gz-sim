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
DEFINE_string(network_role, "", "Participant role used in a distributed "
    " simulation environment. Role is one of [primary, secondary].");
DEFINE_int32(network_secondaries, 0, "Number of secondary participants "
    " expected to join a distributed simulation environment. (Primary only).");
DEFINE_bool(record, false, "Use logging system to record states");
DEFINE_string(record_path, "", "Custom path to put recorded files");
DEFINE_bool(record_resources, false, "Record meshes and material files");
DEFINE_string(playback, "", "Use logging system to play back states");
DEFINE_uint32(seed, 0, "Start with a given random number seed");

//////////////////////////////////////////////////
void help()
{
  std::cout
  << "DEPRECATED: Use the 'ign gazebo' command line tool."
  << std::endl
  << std::endl
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
  << std::endl
  << "  --levels               Use the level system."
  << std::endl
  << "                         The default is false, which loads all models."
  << std::endl
  << "                         It's always true with --network-role."
  << std::endl
  << "  --distributed          Use the distributed simulation system."
  << " The default is false, which disables all distributed simulation."
  << " This will be deprecated in ign-gazebo2. Please use --network-role "
  << " and/or --network-secondaries instead. It implies --levels."
  << std::endl
  << "  --network-role         Participant role used in a distributed "
  << " simulation environment. Role is one of [primary, secondary]."
  << " It implies --levels."
  << std::endl
  << "  --network-secondaries  Number of secondary participants "
  << " expected to join a distributed simulation environment. (Primary only)"
  << std::endl
  << std::endl
  << "  --record               Use logging system to record states."
  << std::endl
  << "  --record-path arg      Implicitly invokes --record, and specifies"
  << " custom path to put recorded files. Argument is path to recorded states."
  << std::endl
  << "  --record-resources     Implicitly invokes --record, and records"
  << " meshes and material files."
  << std::endl
  << "  --playback arg         Use logging system to play back states."
  << " Argument is path to recorded states."
  << std::endl
  << "  --seed arg             Start with a given random number seed."
  << " Arg is the random seed (unsigned int)."
  << std::endl
  << "Environment variables:" << std::endl
  << "  IGN_GAZEBO_RESOURCE_PATH    Colon separated paths used to locate "
  << " resources. Can be useful with the -f option to find an SDF file."
  << std::endl
  << "  IGN_GAZEBO_NETWORK_ROLE     Participant role used in a distributed "
  << " simulation environment. Role is one of [PRIMARY, SECONDARY]. This will"
  << " be deprecated in ign-gazebo2. Please use --network-role instead."
  << std::endl
  << "  IGN_GAZEBO_NETWORK_SECONDARIES    Number of secondary participants "
  << " expected to join a distributed simulation environment. (Primary only)"
  << " This will be deprecated in ign-gazebo2. Please use --network-role "
  << " instead."
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
  ignerr << "The ign-gazebo-server tool is deprecated, and is replaced by "
    << "`ign gazebo`\n";
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

  /// \todo(nkoenig) Deprecated the FLAGS_distributed in ign-gazebo2, and
  /// remove in ign-gazebo3. The FLAGS_network_role is used to indicate
  /// if distributed simulation is enabled.
  if (FLAGS_distributed)
  {
    ignwarn << "Distributed simulation configuration via --distributed"
      << " and environment variables is deprecated. Please use the"
      << " --network-role and --network-secondaries command line options"
      << " instead.\n";
    ignmsg << "Using the distributed simulation and levels systems\n";
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
    serverConfig.SetUseDistributedSimulation(true);
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
    serverConfig.SetUseLevels(true);
  }

  if (!FLAGS_network_role.empty())
  {
    // This if is here to prevent the ignmsg from being displayed twice
    // in the case when FLAGS_distributed is used with FLAGS_network_role
    if (!FLAGS_distributed)
      ignmsg << "Using the distributed simulation and levels systems\n";
    serverConfig.SetNetworkRole(FLAGS_network_role);
    serverConfig.SetNetworkSecondaries(FLAGS_network_secondaries);
    serverConfig.SetUseLevels(true);
  }

  if (!FLAGS_record_path.empty() || FLAGS_record || FLAGS_record_resources)
  {
    if (!FLAGS_playback.empty())
    {
      ignerr << "Both record and playback are specified. Only specify one.\n";
      return -1;
    }

    serverConfig.SetUseLogRecord(true);
    serverConfig.SetLogRecordResources(FLAGS_record_resources);

    if (!FLAGS_record_path.empty())
    {
      serverConfig.SetLogRecordPath(ignition::common::absPath(
        FLAGS_record_path));
    }
    else
    {
      ignmsg << "Recording states to default path\n";
    }
  }

  if (!FLAGS_playback.empty())
  {
    if (!FLAGS_f.empty())
    {
      ignerr << "Both an SDF file and playback flag are specified. "
        << "Only specify one.\n";
      return -1;
    }
    else
    {
      ignmsg << "Playing back states" << FLAGS_playback << std::endl;
      serverConfig.SetLogPlaybackPath(ignition::common::absPath(
        FLAGS_playback));
    }
  }

  if (FLAGS_seed)
    serverConfig.SetSeed(FLAGS_seed);

  // Create the Gazebo server
  ignition::gazebo::Server server(serverConfig);

  // Run the server
  server.Run(true, FLAGS_iterations, !FLAGS_r);

  igndbg << "Shutting down ign-gazebo-server" << std::endl;
  return 0;
}
