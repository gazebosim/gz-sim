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

#include <string>

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ignition/gazebo/test_config.hh"

#ifdef _WIN32
#include <direct.h>
#define ChangeDirectory _chdir
#else
#include <unistd.h>
#define ChangeDirectory chdir
#endif

using namespace ignition;
using namespace gazebo;

class LogSystemTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

/////////////////////////////////////////////////
// This test checks that a file is created by log recorder
TEST_F(LogSystemTest, CreateLogFile)
{
  // Configure to use binary path as cache
  ASSERT_EQ(0, ChangeDirectory(PROJECT_BINARY_PATH));
  common::removeAll("test_cache");
  common::createDirectories("test_cache");

  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "log_record_keyboard.sdf");
  serverConfig.SetSdfFile(sdfFile);
  serverConfig.SetResourceCache(common::cwd() + "/test_cache");

  // Start server
  Server server(serverConfig);
  server.Run(true, 1000, false);

  // Verify file is created
  // TODO: Change this to be in build/ directory
  EXPECT_TRUE(common::exists(common::joinPaths("/", "tmp", "log")));

  common::removeAll("test_cache");
}

