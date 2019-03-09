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

#include <string>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>

#include <sdf/Root.hh>
#include <sdf/World.hh>
#include <sdf/Element.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/ServerConfig.hh"
#include "ignition/gazebo/test_config.hh"

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
  std::string cacheDir = common::joinPaths(PROJECT_BINARY_PATH, "test",
    "test_cache");
  if (common::exists(cacheDir))
  {
    common::removeAll(cacheDir);
  }
  common::createDirectories(cacheDir);
  std::string logDest = common::joinPaths(cacheDir, "log");

  ServerConfig serverConfig;
  serverConfig.SetResourceCache(cacheDir);

  const auto sdfPath = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "log_record_keyboard.sdf");

  sdf::Root root;
  EXPECT_EQ(root.Load(sdfPath).size(), 0lu);
  EXPECT_GT(root.WorldCount(), 0lu);
  const sdf::World * sdfWorld = root.WorldByIndex(0);
  EXPECT_TRUE(sdfWorld->Element()->HasElement("plugin"));

  sdf::ElementPtr pluginElt = sdfWorld->Element()->GetElement("plugin");
  while (pluginElt != nullptr)
  {
    if (pluginElt->HasAttribute("name"))
    {
      // Change log path to build directory
      if (pluginElt->GetAttribute("name")->GetAsString().find("LogRecord")
        != std::string::npos)
      {
        if (pluginElt->HasElement("path"))
        {
          sdf::ElementPtr pathElt = pluginElt->GetElement("path");
          pathElt->Set(logDest);
        }
        else
        {
          sdf::ElementPtr pathElt = pluginElt->AddElement("path");
          pathElt->Set(logDest);
        }
      }
    }

    // Go to next plugin
    pluginElt = pluginElt->GetNextElement("plugin");
  }

  // Pass changed SDF to server
  serverConfig.SetSdfString(root.Element()->ToString(""));

  // Start server
  Server server(serverConfig);
  server.Run(true, 1000, false);

  // Verify file is created
  EXPECT_TRUE(common::exists(common::joinPaths(logDest, "state.tlog")));

  common::removeAll(cacheDir);
}

