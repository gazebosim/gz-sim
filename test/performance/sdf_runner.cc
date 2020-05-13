/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <array>

#include <ignition/msgs.hh>
#include <ignition/math/Stopwatch.hh>
#include <ignition/common/Console.hh>

#include "ignition/transport/Node.hh"

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

using namespace ignition;
using namespace gazebo;

DEFINE_int32(verbose, 1, "");
DEFINE_int32(v, 1, "");
DEFINE_double(z, -1, "Update rate in Hertz.");
DEFINE_uint64(iterations, 10000, "Number of iterations to execute.");
DEFINE_string(f, "", "Load an SDF file on start.");

//////////////////////////////////////////////////
static bool VerbosityValidator(const char */*_flagname*/, int _value)
{
  return _value >= 0 && _value <= 4;
}

//////////////////////////////////////////////////
int main(int _argc, char** _argv)
{
  // Register validators
  gflags::RegisterFlagValidator(&FLAGS_verbose, &VerbosityValidator);
  gflags::RegisterFlagValidator(&FLAGS_v, &VerbosityValidator);

  // Parse command line
  gflags::AllowCommandLineReparsing();
  gflags::ParseCommandLineNonHelpFlags(&_argc, &_argv, true);

  // Hold info as we parse it
  gflags::CommandLineFlagInfo info;

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

  // Set verbosity
  ignition::common::Console::SetVerbosity(FLAGS_verbose);

  ignition::gazebo::ServerConfig serverConfig;
  if (!serverConfig.SetSdfFile(FLAGS_f))
  {
    ignerr << "Failed to set SDF file [" << FLAGS_f << "]" << std::endl;
    return -1;
  }

  // Set the update rate.
  if (FLAGS_z > 0.0)
    serverConfig.SetUpdateRate(FLAGS_z);

  // Create the Gazebo server
  ignition::gazebo::Server server(serverConfig);

  ignition::transport::Node node;

  std::vector<ignition::msgs::Clock> msgs;
  msgs.reserve(FLAGS_iterations);

  std::function<void(const ignition::msgs::Clock&)> cb =
    [&](const ignition::msgs::Clock &_msg)
    {
      msgs.push_back(_msg);
    };

  double progress = 0;

  std::function<void(const ignition::msgs::WorldStatistics &)> cb2 =
    [&](const ignition::msgs::WorldStatistics &_msg)
    {
      double nIters = _msg.iterations();
      nIters = nIters / FLAGS_iterations * 100;
      if (nIters >= progress)
      {
        std::cout << "Simulation Progress: " << nIters << "%" << std::endl;
        progress += 1.0;
      }
    };

  node.Subscribe("/clock", cb);
  node.Subscribe("/stats", cb2);

  // Run the server
  server.Run(true, FLAGS_iterations, false);

  std::ofstream ofs("data.csv", std::ofstream::out);

  ofs << "# Filename: " << FLAGS_f << std::endl;
  ofs << "# Iterations: " << FLAGS_iterations << std::endl;
  ofs << "# Rate: " << FLAGS_z << std::endl;
  ofs << "# Real s, Real ns, sim s, sim ns" << std::endl;

  for (auto &msg : msgs)
  {
    ofs << msg.real().sec() << ", " << msg.real().nsec() << ", "
        << msg.sim().sec() << ", " << msg.sim().nsec() << std::endl;
  }

  return 0;
}
