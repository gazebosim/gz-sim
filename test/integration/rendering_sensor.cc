/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/transport/Node.hh>
#include <ignition/utilities/ExtraTestMacros.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace std::chrono_literals;

std::mutex mutex;
int leftMsgCount = 0;
int rightMsgCount = 0;

//////////////////////////////////////////////////
class CustomRenderingSensorTest :
  public InternalFixture<InternalFixture<::testing::Test>>
{
};

/////////////////////////////////////////////////
void leftImgCb(const msgs::Image & _msg)
{
  ASSERT_EQ(msgs::PixelFormatType::RGB_INT8,
      _msg.pixel_format_type());

  std::lock_guard<std::mutex> lock(mutex);
  leftMsgCount++;
}

/////////////////////////////////////////////////
void rightImgCb(const msgs::Image & _msg)
{
  ASSERT_EQ(msgs::PixelFormatType::RGB_INT8,
      _msg.pixel_format_type());

  std::lock_guard<std::mutex> lock(mutex);
  rightMsgCount++;
}

/////////////////////////////////////////////////
// Test whether custom rendering sensor is publishing messages
TEST_F(CustomRenderingSensorTest,
    IGN_UTILS_TEST_DISABLED_ON_MAC(TestMessages))
{
  // Start server
  gazebo::ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "custom_rendering_sensor_world.sdf");
  serverConfig.SetSdfFile(sdfFile);

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // subscribe to the sensors topics
  transport::Node node;
  leftMsgCount = 0;
  rightMsgCount = 0;
  node.Subscribe("/camera_left", &leftImgCb);
  node.Subscribe("/camera_right", &rightImgCb);

  // Run server and verify that we are receiving messages
  server.Run(true, 100, false);

  int i = 0;
  while (i < 100 && leftMsgCount <= 0 && rightMsgCount <= 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i++;
  }

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_GE(leftMsgCount, 1);
  EXPECT_GE(rightMsgCount, 1);
}
