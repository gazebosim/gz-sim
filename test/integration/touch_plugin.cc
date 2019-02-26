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

#include <gtest/gtest.h>
#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test TouchPlugin system
class TouchPluginTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);

    ServerConfig serverConfig;
    serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
        "/test/worlds/touch_plugin.sdf");
    server = std::make_unique<Server>(serverConfig);
    using namespace std::chrono_literals;
    server->SetUpdatePeriod(0ns);

    EXPECT_FALSE(server->Running());
    EXPECT_FALSE(*server->Running(0));
  }

  public: std::unique_ptr<Server> server;
};

/////////////////////////////////////////////////
TEST_F(TouchPluginTest, OneLink)
{
  bool whiteTouched{false};
  auto whiteTouchCb =
      std::function([&](const msgs::Boolean &) { whiteTouched = true; });

  transport::Node node;
  node.Subscribe("/white_touches_only_green/touched", whiteTouchCb);

  // Check boxes haven't touched yet
  EXPECT_FALSE(whiteTouched);

  // Let white box fall on top of green box
  server->Run(true, 1000, false);

  // Check it hasn't touched for long enough
  EXPECT_FALSE(whiteTouched);

  // Give it time to touch for 3 seconds
  server->Run(true, 3100, false);

  // Check it has touched for long enough
  EXPECT_TRUE(whiteTouched);

  // Wait more and check it doesn't notify again
  whiteTouched = false;
  server->Run(true, 3100, false);
  EXPECT_FALSE(whiteTouched);

  // Enable plugin again
  msgs::Boolean req;
  req.set_data(true);
  bool executed = node.Request("/white_touches_only_green/enable", req);

  EXPECT_TRUE(executed);

  // Wait and see it notifies again
  whiteTouched = false;
  server->Run(true, 3100, false);
  EXPECT_TRUE(whiteTouched);
}

//////////////////////////////////////////////////
TEST_F(TouchPluginTest, MultiLink)
{
  bool redTouched{false};
  auto redTouchedCb =
      std::function([&](const msgs::Boolean &) { redTouched  = true; });

  transport::Node node;
  // Subscribe to plugin notifications
  node.Subscribe("/red_and_yellow_touch_only_green/touched", redTouchedCb);

  // Check boxes haven't touched yet
  EXPECT_FALSE(redTouched);

  // Let red and yellow boxes fall on top of green box
  server->Run(true, 1000, false);

  // Check it hasn't touched for long enough
  EXPECT_FALSE(redTouched);

  // Give it time to touch for 2 seconds
  server->Run(true, 2100, false);

  // Check it has touched for long enough
  EXPECT_TRUE(redTouched);
}

//////////////////////////////////////////////////
TEST_F(TouchPluginTest, StartDisabled)
{
  // Subscribe to plugin notifications
  bool blueTouched{false};
  auto blueTouchedCb =
      std::function([&](const msgs::Boolean &) { blueTouched  = true; });

  transport::Node node;
  // Subscribe to plugin notifications
  node.Subscribe("/blue_touches_only_green/touched", blueTouchedCb);

  // Check boxes haven't touched yet
  EXPECT_FALSE(blueTouched);

  // Let the box fall on top of green box and touch for a while
  server->Run(true, 3000, false);

  // Verify we don't get a notification
  EXPECT_FALSE(blueTouched);

  // Enable plugin
  msgs::Boolean req;
  req.set_data(true);
  bool executed = node.Request("/blue_touches_only_green/enable", req);

  EXPECT_TRUE(executed);

  // Wait and see it notifies now
  blueTouched = false;
  server->Run(true, 3000, false);
  EXPECT_TRUE(blueTouched);
}

//////////////////////////////////////////////////
TEST_F(TouchPluginTest, RemovalOfParentModel)
{
  bool redTouched{false};
  auto redTouchedCb =
      std::function([&](const msgs::Boolean &) { redTouched  = true; });

  transport::Node node;
  // Subscribe to plugin notifications
  node.Subscribe("/red_and_yellow_touch_only_green/touched", redTouchedCb);

  // Check boxes haven't touched yet
  EXPECT_FALSE(redTouched);

  // Let red and yellow boxes fall on top of green box
  server->Run(true, 1000, false);

  // Check it hasn't touched for long enough
  EXPECT_FALSE(redTouched);

  // Remove the model containing the touch plugin
  server->RequestRemoveEntity("red_yellow_box");

  // Give it time to touch for 2 seconds
  server->Run(true, 2100, false);

  // Check that we don't get a touched event
  EXPECT_FALSE(redTouched);
}
