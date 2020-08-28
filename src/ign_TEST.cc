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

#include <string>

#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

static const std::string kBinPath(PROJECT_BINARY_PATH);

// Command line not working on OSX, see
// https://github.com/ignitionrobotics/ign-gazebo/issues/25/
#ifndef __APPLE__
static const std::string kIgnCommand(
  "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=" + kBinPath + "/lib LD_LIBRARY_PATH=" +
  kBinPath + "/lib:/usr/local/lib:${LD_LIBRARY_PATH} ign gazebo -s ");

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
  setenv("IGN_FUEL_CACHE_PATH", projectPath.c_str(), true);
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
  EXPECT_NE(output.find("Unable to find file plugins.sdf"), std::string::npos)
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
#endif

/////////////////////////////////////////////////
/// Main
int main(int _argc, char **_argv)
{
  // Set IGN_CONFIG_PATH to the directory where the .yaml configuration files
  // is located.
  setenv("IGN_CONFIG_PATH", IGN_CONFIG_PATH, 1);

  ::testing::InitGoogleTest(&_argc, _argv);
  return RUN_ALL_TESTS();
}
