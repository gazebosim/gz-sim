/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <gz/msgs/image.pb.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include "gz/sim/Server.hh"
#include "test_config.hh"
#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test LensFlare system
class LensFlareTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
msgs::Image imageMsg;
unsigned char *imageBuffer = nullptr;

/////////////////////////////////////////////////
void imageCb(const msgs::Image &_msg)
{
  ASSERT_EQ(msgs::PixelFormatType::RGB_INT8,
      _msg.pixel_format_type());

  // Add the frame to image buffer
  mutex.lock();
  unsigned int imageSamples = _msg.width() * _msg.height() * 3;

  if (!imageBuffer)
    imageBuffer = new unsigned char[imageSamples];
  memcpy(imageBuffer, _msg.data().c_str(), imageSamples);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the Camera with Lens Flare
TEST_F(LensFlareTest,
    GZ_UTILS_TEST_DISABLED_ON_MAC(LensFlareTest))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(std::string(PROJECT_SOURCE_PATH),
    "test", "worlds", "camera_lens_flare.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Subscribe to the image topic
  transport::Node node;
  node.Subscribe("/camera_lens_flare", &imageCb);

  // Run server and verify that we are receiving a message
  // from the camera with lens flare
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

  delete[] imageBuffer;
}
