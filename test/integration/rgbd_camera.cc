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

#include <ignition/msgs/image.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

#define DEPTH_TOL 1e-4

using namespace ignition;
using namespace gazebo;

/// \brief Test RgbdCameraTest system
class RgbdCameraTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

std::mutex mutex;
msgs::Image depthMsg;
float *depthBuffer = nullptr;

/////////////////////////////////////////////////
void depthCb(const msgs::Image &_msg)
{
  mutex.lock();
  unsigned int depthSamples = _msg.width() * _msg.height();
  unsigned int depthBufferSize = depthSamples * sizeof(float);

  if (!depthBuffer)
    depthBuffer = new float[depthSamples];
  memcpy(depthBuffer, _msg.data().c_str(), depthBufferSize);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the Rgbd Camera readings when it faces a box
TEST_F(RgbdCameraTest, RgbdCameraBox)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/rgbd_camera_sensor.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // subscribe to the depth image topic
  transport::Node node;
  node.Subscribe("/rgbd_camera/depth_image", &depthCb);

  // Run server and verify that we are receiving a message
  // from the depth camera
  size_t iters100 = 100u;
  server.Run(true, iters100, false);

  ignition::common::Time waitTime = ignition::common::Time(0.001);
  int i = 0;
  while (nullptr == depthBuffer && i < 500)
  {
    ignition::common::Time::Sleep(waitTime);
    i++;
  }
  ASSERT_NE(depthBuffer, nullptr);

  // Take into account box of 1 m on each side and 0.05 cm sensor offset
  double expectedRangeAtMidPointBox1 = 2.45;
  unsigned int height = 256;
  unsigned int width = 256;

  // Sensor should see TestBox1
  int left = height/2 * width;
  int mid = height/2 * width + width/2 - 1;
  int right = height/2 * width  + width - 1;

  // Lock access to buffer and don't release it
  mutex.lock();
  EXPECT_DOUBLE_EQ(depthBuffer[left], ignition::math::INF_D);
  EXPECT_NEAR(depthBuffer[mid], expectedRangeAtMidPointBox1, DEPTH_TOL);
  EXPECT_DOUBLE_EQ(depthBuffer[right], ignition::math::INF_D);

  delete[] depthBuffer;
}
