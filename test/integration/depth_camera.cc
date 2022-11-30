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

#include <gz/msgs/image.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

#define DEPTH_TOL 1e-4

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test DepthCameraTest system
class DepthCameraTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
msgs::Image depthMsg;
float * depthBuffer = nullptr;

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
// The test checks the Depth Camera readings when it faces a box
TEST_F(DepthCameraTest, GZ_UTILS_TEST_DISABLED_ON_MAC(DepthCameraBox))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/depth_camera_sensor.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // subscribe to the depth camera topic
  transport::Node node;
  node.Subscribe("/depth_camera", &depthCb);

  // Run server and verify that we are receiving a message
  // from the depth camera
  size_t iters100 = 100u;
  server.Run(true, iters100, false);

  int sleep{0};
  int maxSleep{30};
  while (depthBuffer == nullptr && sleep < maxSleep)
  {
    std::this_thread::sleep_for(100ms);
    sleep++;
  }
  EXPECT_LT(sleep, maxSleep);
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
  EXPECT_DOUBLE_EQ(depthBuffer[left], math::INF_D);
  EXPECT_NEAR(depthBuffer[mid], expectedRangeAtMidPointBox1, DEPTH_TOL);
  EXPECT_DOUBLE_EQ(depthBuffer[right], math::INF_D);

  delete[] depthBuffer;
}
