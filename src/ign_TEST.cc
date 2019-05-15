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

#ifdef __APPLE__
static const std::string kldLibraryPath =  // NOLINT(runtime/string)
"DYLD_LIBRARY_PATH";
static const std::string kIgnTool =  // NOLINT(runtime/string)
"ign-gazebo-server";
static const std::string kSdfFileOpt =  // NOLINT(runtime/string)
"-f ";
#else
static const std::string kLdLibraryPath =  // NOLINT(runtime/string)
"LD_LIBRARY_PATH";
static const std::string kIgnTool =  // NOLINT(runtime/string)
"ign gazebo -s";
static const std::string kSdfFileOpt =  // NOLINT(runtime/string)
" ";
#endif
static const std::string kIgnCommand(
  "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=" + kBinPath + "/lib " +
  kLdLibraryPath + "=" + kBinPath + "/lib:/usr/local/lib:${" +
  kLdLibraryPath + "} " + kIgnTool);

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
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
TEST(CmdLine, Server)
{
  std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 " + kSdfFileOpt +
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
    " -r -v 4 --iterations 5 " + kSdfFileOpt + " plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  output = customExecStr(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }
}

/////////////////////////////////////////////////
TEST(CmdLine, GazeboServer)
{
  std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 " + kSdfFileOpt +
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
  std::string cmd = kIgnCommand + " -r -v 4 --iterations 5 " + kSdfFileOpt +
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
/// Main
int main(int argc, char **argv)
{
  // Set IGN_CONFIG_PATH to the directory where the .yaml configuration files
  // is located.
  setenv("IGN_CONFIG_PATH", IGN_CONFIG_PATH, 1);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
