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
#include <stdio.h>
#include <stdlib.h>

#include <string>

#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

static const std::string g_bin(PROJECT_BINARY_PATH);

static const std::string g_ignCommand(
  "IGN_GAZEBO_SYSTEM_PLUGIN_PATH=" + g_bin + "/lib " +
  "LD_LIBRARY_PATH=" + g_bin + "/lib:/usr/local/lib:${LD_LIBRARY_PATH} " +
  g_bin + "/bin/");

/////////////////////////////////////////////////
std::string custom_exec_str(std::string _cmd)
{
  _cmd += " 2>&1";
  FILE *pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
TEST(CmdLine, Server)
{
  std::string cmd = g_ignCommand +
    "ign-gazebo-server -r -v 4 --iterations 5 -f " +
    std::string(PROJECT_SOURCE_PATH) + "/test/worlds/plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = custom_exec_str(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }
}

/////////////////////////////////////////////////
TEST(CmdLine, GazeboServer)
{
  std::string cmd = g_ignCommand +
    "ign-gazebo -s -r -v 4 --iterations 5 -f " +
    std::string(PROJECT_SOURCE_PATH) + "/test/worlds/plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = custom_exec_str(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }
}

/////////////////////////////////////////////////
TEST(CmdLine, Gazebo)
{
  std::string cmd = g_ignCommand +
    "ign-gazebo -r -v 4 --iterations 5 -f " +
    std::string(PROJECT_SOURCE_PATH) + "/test/worlds/plugins.sdf";

  std::cout << "Running command [" << cmd << "]" << std::endl;

  std::string output = custom_exec_str(cmd);

  for (auto i : {1, 2, 3, 4, 5})
  {
    EXPECT_NE(output.find("iteration " + std::to_string(i)), std::string::npos)
        << output;
  }
}

