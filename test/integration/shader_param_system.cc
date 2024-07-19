/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test ShaderParamTest system
class ShaderParamTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
msgs::Image imageMsg;
unsigned char *imageBuffer = nullptr;

/////////////////////////////////////////////////
void imageCb(const msgs::Image &_msg)
{
  mutex.lock();
  unsigned int channels = 3u;
  unsigned int imageSamples = _msg.width() * _msg.height();
  unsigned int imageBufferSize = imageSamples * sizeof(unsigned char)
      * channels;

  if (!imageBuffer)
    imageBuffer = new unsigned char[imageSamples * channels];
  memcpy(imageBuffer, _msg.data().c_str(), imageBufferSize);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks camera image data to verify that the sphere is using
// custom material shaders
TEST_F(ShaderParamTest, GZ_UTILS_TEST_DISABLED_ON_MAC(ShaderParam))
{
  // This test fails on Github Actions. Skip it for now.
  // Note: The GITHUB_ACTIONS environment variable is automatically set when
  // running on Github Actions.
  std::string githubAction;
  if (common::env("GITHUB_ACTIONS", githubAction))
  {
    GTEST_SKIP();
  }

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "examples", "worlds", "shader_param.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // subscribe to the image camera topic
  transport::Node node;
  node.Subscribe("/camera", &imageCb);

  // Run server and verify that we are receiving a message
  // from the image camera
  size_t iters100 = 100u;
  server.Run(true, iters100, false);

  int sleep{0};
  int maxSleep{30};
  while (imageBuffer == nullptr && sleep < maxSleep)
  {
    std::this_thread::sleep_for(100ms);
    sleep++;
  }
  EXPECT_LT(sleep, maxSleep);
  ASSERT_NE(imageBuffer, nullptr);

  // shaders set the sphere color to red
  unsigned int height = 320;
  unsigned int width = 240;

  int mid = (height / 2 * width * 3u) + (width / 2 - 1) * 3u;

  // Lock access to buffer and don't release it
  mutex.lock();
  int r = static_cast<int>(imageBuffer[mid]);
  int g = static_cast<int>(imageBuffer[mid+1]);
  int b = static_cast<int>(imageBuffer[mid+2]);
  EXPECT_GT(r, g);
  EXPECT_GT(r, b);
  EXPECT_EQ(g, b);

  delete[] imageBuffer;
}
