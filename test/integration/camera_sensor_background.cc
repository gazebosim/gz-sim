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

#include <gz/msgs/image.pb.h>

#include <string>

#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace std::chrono_literals;

std::mutex mutex;
int cbCount = 0;

//////////////////////////////////////////////////
class CameraSensorBackgroundFixture :
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
      // The "/test/worlds/camera_sensor_empty_scene.sdf" world has set a
      // background color of 1,0,0,1. So, all the pixels returned by the
      // camera should be red.
      unsigned char r = _msg.data()[y * _msg.step() + x*3];
      EXPECT_EQ(255, static_cast<int>(r));

      unsigned char g = _msg.data()[y * _msg.step() + x*3+1];
      EXPECT_EQ(0, static_cast<int>(g));

      unsigned char b = _msg.data()[y * _msg.step() + x*3+2];
      EXPECT_EQ(0, static_cast<int>(b));
    }
  }
  std::lock_guard<std::mutex> lock(mutex);
  cbCount++;
}

/////////////////////////////////////////////////
// Test the ability to set the background color using the sensor system
// plugin.
TEST_F(CameraSensorBackgroundFixture,
    GZ_UTILS_TEST_DISABLED_ON_MAC(RedBackground))
{
  // Start server
  sim::ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "camera_sensor_empty_scene.sdf");
  serverConfig.SetSdfFile(sdfFile);

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // subscribe to the camera topic
  transport::Node node;
  cbCount = 0;
  node.Subscribe("/camera", &cameraCb);

  // Run server and verify that we are receiving a message
  // from the depth camera
  server.Run(true, 100, false);

  int i = 0;
  while (i < 100 && cbCount <= 0)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    i++;
  }

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_GE(cbCount, 1);
}
