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
#include <ignition/common/Util.hh>
#include <ignition/utilities/ExtraTestMacros.hh>

#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

static const std::string kBinPath(PROJECT_BINARY_PATH);

static const std::string kIgnCommand(
    std::string(BREW_RUBY) + std::string(IGN_PATH) + "/ign gazebo -s ");

/////////////////////////////////////////////////
std::string customExecStr(std::string _cmd)
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
  std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 " +
    std::string(PROJECT_SOURCE_PATH) + "/test/worlds/plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = customExecStr(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }

  // Use IGN_GAZEBO_RESOURCE_PATH instead of specifying the complete path
  cmd = std::string("IGN_GAZEBO_RESOURCE_PATH=") +
    PROJECT_SOURCE_PATH + "/test/worlds " + kIgnCommand +
    " -r -v 4 --iterations 5 " + " plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  output = customExecStr(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }
}

/////////////////////////////////////////////////
TEST(CmdLine, CachedFuelWorld)
{
  std::string projectPath = std::string(PROJECT_SOURCE_PATH) + "/test/worlds";
  ignition::common::setenv("IGN_FUEL_CACHE_PATH", projectPath.c_str());
  std::string cmd = kIgnCommand + " -r -v 4 --iterations 5" +
    " https://fuel.ignitionrobotics.org/1.0/OpenRobotics/worlds/Test%20world";
  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = customExecStr(cmd);
  EXPECT_NE(output.find("Cached world found."), std::string::npos)
      << output;
}

/////////////////////////////////////////////////
TEST(CmdLine, GazeboServer)
{
  std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 " +
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
TEST(CmdLine, Gazebo)
{
  std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 " +
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
TEST(CmdLine, ResourcePath)
{
  std::string cmd = kIgnCommand + " -s -r -v 4 --iterations 1 plugins.sdf";

  // No path
  std::string output = customExecStr(cmd);
  EXPECT_NE(output.find("Unable to find or download file"), std::string::npos)
      << output;

  // Correct path
  auto path = std::string("IGN_GAZEBO_RESOURCE_PATH=") +
    PROJECT_SOURCE_PATH + "/test/worlds ";

  output = customExecStr(path + cmd);
  EXPECT_EQ(output.find("Unable to find file plugins.sdf"), std::string::npos)
      << output;

  // Several paths
  path = std::string("IGN_GAZEBO_RESOURCE_PATH=banana:") +
    PROJECT_SOURCE_PATH + "/test/worlds:orange ";

  output = customExecStr(path + cmd);
  EXPECT_EQ(output.find("Unable to find file plugins.sdf"), std::string::npos)
      << output;
}

//////////////////////////////////////////////////
/// \brief Check --help message and bash completion script for consistent flags
TEST(CmdLine, HelpVsCompletionFlags)
{
  // Flags in help message
  std::string output = customExecStr(kIgnCommand + " gazebo --help");
  EXPECT_NE(std::string::npos, output.find("-g")) << output;
  EXPECT_NE(std::string::npos, output.find("--iterations")) << output;
  EXPECT_NE(std::string::npos, output.find("--levels")) << output;
  EXPECT_NE(std::string::npos, output.find("--network-role")) << output;
  EXPECT_NE(std::string::npos, output.find("--network-secondaries")) << output;
  EXPECT_NE(std::string::npos, output.find("--record")) << output;
  EXPECT_NE(std::string::npos, output.find("--record-path")) << output;
  EXPECT_NE(std::string::npos, output.find("--record-resources")) << output;
  EXPECT_NE(std::string::npos, output.find("--record-topic")) << output;
  EXPECT_NE(std::string::npos, output.find("--log-overwrite")) << output;
  EXPECT_NE(std::string::npos, output.find("--log-compress")) << output;
  EXPECT_NE(std::string::npos, output.find("--playback")) << output;
  EXPECT_NE(std::string::npos, output.find("-r")) << output;
  EXPECT_NE(std::string::npos, output.find("-s")) << output;
  EXPECT_NE(std::string::npos, output.find("--verbose")) << output;
  EXPECT_NE(std::string::npos, output.find("--gui-config")) << output;
  EXPECT_NE(std::string::npos, output.find("--physics-engine")) << output;
  EXPECT_NE(std::string::npos, output.find("--render-engine")) << output;
  EXPECT_NE(std::string::npos, output.find("--render-engine-gui")) << output;
  EXPECT_NE(std::string::npos, output.find("--render-engine-server")) << output;
  EXPECT_NE(std::string::npos, output.find("--version")) << output;
  EXPECT_NE(std::string::npos, output.find("-z")) << output;
  EXPECT_NE(std::string::npos, output.find("--help")) << output;
  EXPECT_NE(std::string::npos, output.find("--force-version")) << output;
  EXPECT_NE(std::string::npos, output.find("--versions")) << output;

  // Flags in bash completion
  std::ifstream scriptFile(std::string(PROJECT_SOURCE_PATH) +
    "/src/cmd/gazebo.bash_completion.sh");
  std::string script((std::istreambuf_iterator<char>(scriptFile)),
      std::istreambuf_iterator<char>());

  EXPECT_NE(std::string::npos, script.find("-g")) << script;
  EXPECT_NE(std::string::npos, script.find("--iterations")) << script;
  EXPECT_NE(std::string::npos, script.find("--levels")) << script;
  EXPECT_NE(std::string::npos, script.find("--network-role")) << script;
  EXPECT_NE(std::string::npos, script.find("--network-secondaries")) << script;
  EXPECT_NE(std::string::npos, script.find("--record")) << script;
  EXPECT_NE(std::string::npos, script.find("--record-path")) << script;
  EXPECT_NE(std::string::npos, script.find("--record-resources")) << script;
  EXPECT_NE(std::string::npos, script.find("--record-topic")) << script;
  EXPECT_NE(std::string::npos, script.find("--log-overwrite")) << script;
  EXPECT_NE(std::string::npos, script.find("--log-compress")) << script;
  EXPECT_NE(std::string::npos, script.find("--playback")) << script;
  EXPECT_NE(std::string::npos, script.find("-r")) << script;
  EXPECT_NE(std::string::npos, script.find("-s")) << script;
  EXPECT_NE(std::string::npos, script.find("--verbose")) << script;
  EXPECT_NE(std::string::npos, script.find("--gui-config")) << script;
  EXPECT_NE(std::string::npos, script.find("--physics-engine")) << script;
  EXPECT_NE(std::string::npos, script.find("--render-engine")) << script;
  EXPECT_NE(std::string::npos, script.find("--render-engine-gui")) << script;
  EXPECT_NE(std::string::npos, script.find("--render-engine-server")) << script;
  EXPECT_NE(std::string::npos, script.find("--version")) << script;
  EXPECT_NE(std::string::npos, script.find("-z")) << script;
  EXPECT_NE(std::string::npos, script.find("--help")) << script;
  EXPECT_NE(std::string::npos, script.find("--force-version")) << script;
  EXPECT_NE(std::string::npos, script.find("--versions")) << script;
}
