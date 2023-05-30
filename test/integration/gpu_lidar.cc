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

#include <gz/msgs/laserscan.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

#define LASER_TOL 1e-4

using namespace gz;
using namespace sim;

/// \brief Test GpuLidarTest system
class GpuLidarTest : public InternalFixture<::testing::Test>
{
};

std::mutex mutex;
std::vector<msgs::LaserScan> laserMsgs;

/////////////////////////////////////////////////
void laserCb(const msgs::LaserScan &_msg)
{
  mutex.lock();
  laserMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the Gpu Lidar readings when it faces a box
TEST_F(GpuLidarTest, GZ_UTILS_TEST_DISABLED_ON_MAC(GpuLidarBox))
{
  const int horzSamples = 640;

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/gpu_lidar_sensor.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // subscribe to lidar topic
  transport::Node node;
  node.Subscribe("/lidar", &laserCb);

  // Run server and verify that we are receiving a message
  // from the lidar
  size_t iters100 = 100u;
  server.Run(true, iters100, false);

  // Wait for a message to be received
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mutex.lock();
    bool received = !laserMsgs.empty();
    mutex.unlock();

    if (received)
      break;
  }

  mutex.lock();
  EXPECT_GT(laserMsgs.size(), 0u);
  auto lastMsg = laserMsgs.back();
  mutex.unlock();

  int mid = horzSamples / 2;
  int last = (horzSamples - 1);
  // Take into account box of 1 m on each side and 0.05 cm sensor offset
  double expectedRangeAtMidPointBox1 = 0.45;

  // Sensor 1 should see TestBox1
  EXPECT_DOUBLE_EQ(lastMsg.ranges(0), math::INF_D);
  EXPECT_NEAR(lastMsg.ranges(mid), expectedRangeAtMidPointBox1,
              LASER_TOL);
  EXPECT_DOUBLE_EQ(lastMsg.ranges(last), math::INF_D);
  EXPECT_EQ("gpu_lidar::gpu_lidar_link::gpu_lidar", lastMsg.frame());
}
