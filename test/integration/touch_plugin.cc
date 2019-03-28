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
  }

  public: void StartServer(const std::string &_sdfFile)
  {
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) + _sdfFile);
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
  this->StartServer("/test/worlds/touch_plugin.sdf");

  bool whiteTouched{false};
  auto whiteTouchCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        whiteTouched = true;
      });

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

  // TODO(anyone) Not fair to only wait for "true"
  for (int sleep = 0; sleep < 50 && !whiteTouched; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  EXPECT_TRUE(whiteTouched);
}

//////////////////////////////////////////////////
TEST_F(TouchPluginTest, MultiLink)
{
  this->StartServer("/test/worlds/touch_plugin.sdf");

  bool redTouched{false};
  auto redTouchedCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        redTouched = true;
      });

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
  this->StartServer("/test/worlds/touch_plugin.sdf");

  // Subscribe to plugin notifications
  bool blueTouched{false};
  auto blueTouchedCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        blueTouched = true;
      });

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

  for (int sleep = 0; sleep < 50 && !blueTouched; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  EXPECT_TRUE(blueTouched);
}

//////////////////////////////////////////////////
TEST_F(TouchPluginTest, RemovalOfParentModel)
{
  this->StartServer("/test/worlds/touch_plugin.sdf");

  bool redTouched{false};
  auto redTouchedCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        redTouched = true;
      });

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


//////////////////////////////////////////////////
/// Tests whether the plugin works when it is spawned after other entities have
/// already been created and vice versa
/// This test uses depends on the user_commands system
TEST_F(TouchPluginTest, SpawnedEntities)
{
  std::string whiteBox = R"EOF(
  <?xml version="1.0" ?>
  <sdf version="1.6">
      <model name="white_box">
        <pose>0 0 4 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>0.5 0.5 0.5</size>
              </box>
            </geometry>
          </collision>
          <sensor name="white_box_sensor" type="contact">
            <contact>
              <collision>collision</collision>
            </contact>
          </sensor>
        </link>
        <plugin
          filename="libignition-gazebo-touchplugin-system.so"
          name="ignition::gazebo::systems::TouchPlugin">
          <target>green_box_for_white</target>
          <time>0.2</time>
          <namespace>white_touches_only_green</namespace>
          <enabled>true</enabled>
        </plugin>
      </model>
  </sdf>)EOF";

  std::string greenBox = R"EOF(
  <?xml version="1.0" ?>
  <sdf version="1.6">
      <model name="green_box_for_white">
        <static>1</static>
        <pose>0 0 0.5 0 0 0</pose>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>1 1 1</size>
              </box>
            </geometry>
          </collision>
        </link>
      </model>
  </sdf>)EOF";

  transport::Node node;

  bool whiteTouched{false};
  auto whiteTouchCb = std::function<void(const msgs::Boolean &)>(
      [&](const msgs::Boolean &)
      {
        whiteTouched = true;
      });

  node.Subscribe("/white_touches_only_green/touched", whiteTouchCb);

  // Request entity spawn
  msgs::EntityFactory req;
  unsigned int timeout = 5000;
  std::string service{"/world/empty/create"};

  msgs::Boolean res;
  bool result;

  auto testFunc = [&](const std::string &_box1, const std::string &_box2)
  {
    this->server.reset();
    this->StartServer("/examples/worlds/empty.sdf");

    whiteTouched = false;
    req.set_sdf(_box1);

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());

    // Run the server to actually create the entities
    server->Run(true, 100, false);

    req.set_sdf(_box2);

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());
    server->Run(true, 100, false);

    // Check boxes haven't touched yet
    EXPECT_FALSE(whiteTouched);

    // Let white box fall on top of green box
    server->Run(true, 500, false);

    // Check it hasn't touched for long enough
    EXPECT_FALSE(whiteTouched);

    // Give it time to touch for at least 0.2 seconds
    server->Run(true, 500, false);

    // TODO(anyone) Not fair to only wait for "true"
    for (int sleep = 0; sleep < 50 && !whiteTouched; ++sleep)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    EXPECT_TRUE(whiteTouched);
  };

  // Test different spawn orders
  testFunc(whiteBox, greenBox);
  testFunc(greenBox, whiteBox);
}
