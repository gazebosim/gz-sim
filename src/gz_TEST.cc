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

#include <gtest/gtest.h>
#include <cstdio>
#include <cstdlib>

#include <fstream>
#include <string>
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "test_config.hh"  // NOLINT(build/include)

static const std::string kBinPath(PROJECT_BINARY_PATH);

static const std::string kGzCommand(
    std::string(BREW_RUBY) + std::string(GZ_PATH) + " sim -s ");
static const std::string kGzModelCommand(
    std::string(BREW_RUBY) + std::string(GZ_PATH) + " model ");

/////////////////////////////////////////////////
std::string customExecStr(std::string _cmd)
{
  // Augment the system plugin path.
  gz::common::setenv("GZ_SIM_SYSTEM_PLUGIN_PATH",
      gz::common::joinPaths(std::string(PROJECT_BINARY_PATH), "lib").c_str());

  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != nullptr)
    {
      result += buffer;
    }
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
TEST(CmdLine, Server)
{
  std::string cmd = kGzCommand + " -r -v 4 --iterations 5 " +
    std::string(PROJECT_SOURCE_PATH) + "/test/worlds/plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = customExecStr(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }

// Disable on WIN32 as on Windows it is not support to prepend
// a command with the env variable to set
#ifndef _WIN32
  // Use GZ_SIM_RESOURCE_PATH instead of specifying the complete path
  cmd = std::string("GZ_SIM_RESOURCE_PATH=") +
    PROJECT_SOURCE_PATH + "/test/worlds " + kGzCommand +
    " -r -v 4 --iterations 5 " + " plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  output = customExecStr(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }
#endif
}

/////////////////////////////////////////////////
TEST(CmdLine, CachedFuelWorld)
{
  std::string projectPath = std::string(PROJECT_SOURCE_PATH) + "/test/worlds";
  gz::common::setenv("GZ_FUEL_CACHE_PATH", projectPath.c_str());
  std::string cmd = kGzCommand + " -r -v 4 --iterations 5" +
    " https://fuel.gazebosim.org/1.0/openroboticstest/worlds/test%20world";
  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = customExecStr(cmd);
  EXPECT_NE(output.find("Cached world found."), std::string::npos)
      << output;
}

/////////////////////////////////////////////////
TEST(CmdLine, GZ_UTILS_TEST_DISABLED_ON_WIN32(RandomSeedValue))
{
  std::string cmd = kGzCommand + " -r -v 4 --seed 5 --iterations 5";
  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = customExecStr(cmd);
  EXPECT_NE(output.find("Setting seed value"), std::string::npos)
      << output;
}

/////////////////////////////////////////////////
TEST(CmdLine, GazeboServer)
{
  std::string cmd = kGzCommand + " -r -v 4 --iterations 5 " +
    std::string(PROJECT_SOURCE_PATH) + "/test/worlds/plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = customExecStr(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }
}

/////////////////////////////////////////////////
TEST(CmdLine, SimtimeArgument)
{
  std::string cmd =
    kGzCommand + " -r -v 4 --iterations 500 --initial-sim-time 1000.5 " +
    std::string(PROJECT_SOURCE_PATH) + "/test/worlds/plugins.sdf -z 1000";

  std::cout << "Running command [" << cmd << "]" << std::endl;
  int msgCount = 0;

  gz::transport::Node node;
  auto cb = [&](const gz::msgs::Clock &_msg) -> void
  {
    EXPECT_GE(_msg.sim().sec() + 1e-9 * _msg.sim().nsec(),
        1000.5);
    msgCount++;
  };

  auto cbFcn = std::function<void(const gz::msgs::Clock &)>(cb);
  EXPECT_TRUE(node.Subscribe(std::string("/clock"), cbFcn));

  std::string output = customExecStr(cmd);

  // Try waiting different amounts if no messages have been received
  if (!msgCount)
  {
    for (auto i : {1, 10, 100, 1000})
    {
      std::cout << "Sleeping for " << i << " ms";
      GZ_SLEEP_MS(i);
      std::cout << ", recevied " << msgCount << " messages." << std::endl;
      if (msgCount)
      {
        break;
      }
    }
  }
  EXPECT_GT(msgCount, 0) << output;
}

/////////////////////////////////////////////////
TEST(CmdLine, Gazebo)
{
  std::string cmd = kGzCommand + " -r -v 4 --iterations 5 " +
    std::string(PROJECT_SOURCE_PATH) + "/test/worlds/plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = customExecStr(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }
}

/////////////////////////////////////////////////
TEST(CmdLine, GZ_UTILS_TEST_DISABLED_ON_WIN32(ResourcePath))
{
  std::string cmd = kGzCommand + " -s -r -v 4 --iterations 1 plugins.sdf";

  // No path
  std::string output = customExecStr(cmd);
  EXPECT_NE(output.find("Unable to find or download file"), std::string::npos)
      << output;

  // Correct path
  auto path = std::string("GZ_SIM_RESOURCE_PATH=") +
              gz::common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds ");

  output = customExecStr(path + cmd);
  EXPECT_EQ(output.find("Unable to find file plugins.sdf"), std::string::npos)
      << output;

  path = std::string("GZ_SIM_RESOURCE_PATH=") +
         gz::common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds ");
  output = customExecStr(path + cmd);
  EXPECT_EQ(output.find("Unable to find file plugins.sdf"), std::string::npos)
      << output;

  // Several paths
  path = std::string("GZ_SIM_RESOURCE_PATH=banana:") +
    PROJECT_SOURCE_PATH + "/test/worlds:orange ";

  output = customExecStr(path + cmd);
  EXPECT_EQ(output.find("Unable to find file plugins.sdf"), std::string::npos)
      << output;

  // Test nested models
  // Use a direct path to the input file. Using a file name that has to be
  // resolved interacts with how resource environment variables are processed
  // and so will have different behavior than when a direct path is provided.
  cmd = kGzCommand + " -s -r -v 4 --iterations 1 " +
        gz::common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds",
                              "include_nested_models.sdf");
  output = customExecStr(cmd);
  EXPECT_NE(output.find("Unable to find"), std::string::npos) << output;

  std::string pathValue =
      gz::common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "models");

  output = customExecStr("GZ_SIM_RESOURCE_PATH=" + pathValue + " " + cmd);
  EXPECT_EQ(output.find("Unable to find"), std::string::npos) << output;
}

//////////////////////////////////////////////////
/// \brief Check --help message and bash completion script for consistent flags
TEST(CmdLine, GZ_UTILS_TEST_DISABLED_ON_WIN32(GazeboHelpVsCompletionFlags))
{
  // Flags in help message
  std::string helpOutput = customExecStr(kGzCommand + " sim --help");

  // Call the output function in the bash completion script
  std::string scriptPath = gz::common::joinPaths(
    std::string(PROJECT_SOURCE_PATH),
    "src", "cmd", "sim.bash_completion.sh");

  // Equivalent to:
  // sh -c "bash -c \". /path/to/sim.bash_completion.sh; _gz_sim_flags\""
  std::string cmd = "bash -c \". " + scriptPath + "; _gz_sim_flags\"";
  std::string scriptOutput = customExecStr(cmd);

  // Tokenize script output
  std::istringstream iss(scriptOutput);
  std::vector<std::string> flags((std::istream_iterator<std::string>(iss)),
    std::istream_iterator<std::string>());

  EXPECT_GT(flags.size(), 0u);

  // Match each flag in script output with help message
  for (const auto &flag : flags)
  {
    EXPECT_NE(std::string::npos, helpOutput.find(flag)) << flag;
  }
}

//////////////////////////////////////////////////
/// \brief Check --help message and bash completion script for consistent flags
TEST(CmdLine, GZ_UTILS_TEST_DISABLED_ON_WIN32(ModelHelpVsCompletionFlags))
{
  // Flags in help message
  std::string helpOutput = customExecStr(kGzModelCommand + " --help");

  // Call the output function in the bash completion script
  std::string scriptPath = gz::common::joinPaths(
    std::string(PROJECT_SOURCE_PATH),
    "src", "cmd", "model.bash_completion.sh");

  // Equivalent to:
  // sh -c "bash -c \". /path/to/model.bash_completion.sh; _gz_model_flags\""
  std::string cmd = "bash -c \". " + scriptPath + "; _gz_model_flags\"";
  std::string scriptOutput = customExecStr(cmd);

  // Tokenize script output
  std::istringstream iss(scriptOutput);
  std::vector<std::string> flags((std::istream_iterator<std::string>(iss)),
    std::istream_iterator<std::string>());

  EXPECT_GT(flags.size(), 0u);

  // Match each flag in script output with help message
  for (const auto &flag : flags)
  {
    EXPECT_NE(std::string::npos, helpOutput.find(flag)) << helpOutput;
  }
}
