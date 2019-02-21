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

#include "ignition/gazebo/components/DepthCamera.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

#define LASER_TOL 1e-4
#define DOUBLE_TOL 1e-6

using namespace ignition;
using namespace gazebo;

/// \brief Test DepthCameraTest system
class DepthCameraTest : public ::testing::Test
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
std::vector<msgs::Image> depthMsgs;

/////////////////////////////////////////////////
void depthCb(const msgs::Image &_msg)
{
  mutex.lock();
  depthMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the Depth Camera readings when it faces a box
TEST_F(DepthCameraTest, DepthCameraBox)
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
  mutex.lock();
  EXPECT_GT(depthMsgs.size(), 0u);
  mutex.unlock();

  ignition::common::Time waitTime = ignition::common::Time(0.01);
  int i = 0;
  while (i < 300)
  {
    ignition::common::Time::Sleep(waitTime);
    i++;
  }
  mutex.lock();
  EXPECT_GT(depthMsgs.size(), 0u);
  mutex.unlock();

  // const int horzSamples = 320;
  // int mid = horzSamples / 2;
  // int last = (horzSamples - 1);
  // // Take into account box of 1 m on each side and 0.05 cm sensor offset
  // double expectedRangeAtMidPointBox1 = 0.45;

  // // Sensor 1 should see TestBox1
  // mutex.lock();
  // EXPECT_DOUBLE_EQ(depthMsgs.back().ranges(0), ignition::math::INF_D);
  // EXPECT_NEAR(depthMsgs.back().ranges(mid), expectedRangeAtMidPointBox1,
  //    LASER_TOL);
  // EXPECT_DOUBLE_EQ(depthMsgs.back().ranges(last), ignition::math::INF_D);
  // EXPECT_EQ(depthMsgs.back().frame(), "depth_camera::depth_camera_link");
  // mutex.unlock();
}
