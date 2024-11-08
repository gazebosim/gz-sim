/*
 * Copyright (C) 2024 Open Source Robotics Foundation
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

#include <gz/msgs/image.pb.h>

#include <string>

#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Util.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace std::chrono_literals;

std::mutex mutex;
int cbValue = 0;
int giEnabled = false;

//////////////////////////////////////////////////
/// Note: This test is almost identical to the test in
/// camera_sensor_scene_background.cc, and the `cameraCb` could have been
/// reused, but loading the world twice in a single processes causes errors with
/// Ogre.
class CameraSensorGlobalIlluminationTest :
  public InternalFixture<InternalFixture<::testing::Test>>
{
};

/////////////////////////////////////////////////
void cameraCb(const msgs::Image & _msg)
{
  ASSERT_EQ(msgs::PixelFormatType::RGB_INT8,
      _msg.pixel_format_type());

  for (unsigned int y = 0; y < _msg.height(); ++y)
  {
    for (unsigned int x = 0; x < _msg.width(); ++x)
    {
      unsigned char r = _msg.data()[y * _msg.step() + x*3];
      unsigned char g = _msg.data()[y * _msg.step() + x*3+1];
      unsigned char b = _msg.data()[y * _msg.step() + x*3+2];

      if (!giEnabled)
      {
        ASSERT_LT(static_cast<int>(r), 2);
        ASSERT_LT(static_cast<int>(g), 2);
        ASSERT_LT(static_cast<int>(b), 2);
      }
      else
      {
        ASSERT_GT(static_cast<int>(r), 50);
        ASSERT_LT(static_cast<int>(g), 25);
        ASSERT_LT(static_cast<int>(g), 25);
      }
    }
  }
  std::lock_guard<std::mutex> lock(mutex);
  if (!giEnabled)
    cbValue = 1;
  else
    cbValue = 2;
}

/////////////////////////////////////////////////
// Check that sensor reads a very dark value when GI is not enabled
TEST_F(CameraSensorGlobalIlluminationTest,
       GlobalIlluminationNotEnabled)
{
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "camera_sensor_gi_enabled_false.sdf");
  // Start server
  sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfFile);

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // subscribe to the camera topic
  transport::Node node;
  cbValue = 0;
  giEnabled = false;
  node.Subscribe("/camera", &cameraCb);

  // Run server and verify that we are receiving a message
  // from the depth camera
  server.Run(true, 100, false);

  int i = 0;
  while (i < 100 && cbValue == 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i++;
  }

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(cbValue, 1);
}

/////////////////////////////////////////////////
// Check that sensor reads less dark value when GI is enabled
TEST_F(CameraSensorGlobalIlluminationTest,
       GZ_UTILS_TEST_DISABLED_ON_MAC(GlobalIlluminationEnabled))
{
  // \todo(anyone) test fails on github action but pass on other
  // ubuntu jenkins CI. Need to investigate further.
  // Github action sets the MESA_GL_VERSION_OVERRIDE variable
  // so check for this variable and disable test if it is set.
#ifdef __linux__
  std::string value;
  bool result = common::env("MESA_GL_VERSION_OVERRIDE", value, true);
  if (result && value == "3.3")
  {
    GTEST_SKIP() << "Test is run on machine with software rendering or mesa "
                 << "driver. Skipping test. " << std::endl;
  }
#endif

  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "camera_sensor_gi_enabled_true.sdf");
  // Start server
  sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(sdfFile);

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // subscribe to the camera topic
  transport::Node node;
  cbValue = 0;
  giEnabled = true;
  node.Subscribe("/camera", &cameraCb);

  // Run server and verify that we are receiving a message
  // from the depth camera
  server.Run(true, 100, false);

  int i = 0;
  while (i < 100 && cbValue == 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i++;
  }

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(cbValue, 2);
}
