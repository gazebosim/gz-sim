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
#include <ignition/msgs/gui.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test GuiBroadcaster system
class GuiBroadcasterTest : public ::testing::Test
{
  // Documentation inherited
  protected: virtual void SetUp()
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

/////////////////////////////////////////////////
TEST_F(GuiBroadcasterTest, GuiInfo)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(14u, *server.EntityCount());

  // Run server
  server.Run(true, 1, false);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  msgs::GUI res;

  EXPECT_TRUE(node.Request("/world/default/gui/info", timeout, res, result));
  EXPECT_TRUE(result);

  ASSERT_EQ(1, res.plugin_size());

  auto plugin = res.plugin(0);
  EXPECT_EQ("3D View", plugin.name());
  EXPECT_EQ("Scene3D", plugin.filename());
  EXPECT_NE(plugin.innerxml().find("<ignition-gui>"), std::string::npos);
  EXPECT_NE(plugin.innerxml().find("<ambient_light>"), std::string::npos);
  EXPECT_NE(plugin.innerxml().find("<pose_topic>"), std::string::npos);
}

