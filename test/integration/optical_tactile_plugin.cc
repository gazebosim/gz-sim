/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <gz/msgs/contacts.pb.h>

#include <thread>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/msgs/Utility.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/// \brief Test OpticalTactilePlugin system
class OpticalTactilePluginTest : public InternalFixture<::testing::Test>
{
  public: void StartServer(const std::string &_sdfFile)
  {
    ServerConfig serverConfig;
    serverConfig.SetSdfFile(_sdfFile);
    server = std::make_unique<Server>(serverConfig);
    using namespace std::chrono_literals;
    server->SetUpdatePeriod(0ns);

    EXPECT_FALSE(server->Running());
    EXPECT_FALSE(*server->Running(0));
  }

  public: gz::math::Vector3f MapPointCloudData(
    const uint64_t &_i,
    const uint64_t &_j)
  {
    // Initialize return variable
    gz::math::Vector3f measuredPoint(0, 0, 0);

    std::string data = this->normalForces.data();
    char *msgBuffer = data.data();

    // Number of bytes from the beginning of the pointer (image coordinates at
    // 0,0) to the desired (i,j) position
    uint32_t msgBufferIndex =
      _j * this->normalForces.step() + _i * 3 * sizeof(float);

    measuredPoint.X() = *reinterpret_cast<float*>(
      &msgBuffer[msgBufferIndex]);

    measuredPoint.Y() = *reinterpret_cast<float*>(
      &msgBuffer[msgBufferIndex + sizeof(float)]);

    measuredPoint.Z() = *reinterpret_cast<float*>(
      &msgBuffer[msgBufferIndex + 2*sizeof(float)]);

    return measuredPoint;
  }

  public: msgs::Image normalForces;

  public: std::unique_ptr<Server> server;
};

/////////////////////////////////////////////////
// The test checks the normal forces on the corners of the box-shaped sensor
// Fails to load Ogre plugin on macOS
TEST_F(OpticalTactilePluginTest,
    GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(ForcesOnPlane))
{
  // World with moving entities
  const auto sdfPath = common::joinPaths(
    PROJECT_SOURCE_PATH, "test", "worlds", "optical_tactile_plugin.sdf");
  this->StartServer(sdfPath);

  bool receivedMsg{false};
  math::Vector3f upperLeftNormalForce(0, 0, 0);
  math::Vector3f upperRightNormalForce(0, 0, 0);
  math::Vector3f lowerLeftNormalForce(0, 0, 0);
  math::Vector3f lowerRightNormalForce(0, 0, 0);
  auto normalForcesCb = std::function<void(const msgs::Image &)>(
    [&](const msgs::Image &_image)
    {
      this->normalForces = _image;

      upperRightNormalForce = this->MapPointCloudData(1, 1);
      upperLeftNormalForce = this->MapPointCloudData((_image.width() - 2), 1);
      lowerLeftNormalForce = this->MapPointCloudData(1, (_image.height() - 2));
      lowerRightNormalForce = this->MapPointCloudData(
        _image.width() - 2, _image.height() - 2);
      receivedMsg = true;
    });

  transport::Node node;
  node.Subscribe("/optical_tactile_sensor/normal_forces", normalForcesCb);

  // Check that there are no contacts nor forces yet
  EXPECT_EQ(math::Vector3f(0, 0, 0), upperRightNormalForce);
  EXPECT_EQ(math::Vector3f(0, 0, 0), upperLeftNormalForce);
  EXPECT_EQ(math::Vector3f(0, 0, 0), lowerLeftNormalForce);
  EXPECT_EQ(math::Vector3f(0, 0, 0), lowerRightNormalForce);

  // Let the depth camera generate data
  server->Run(true, 1000, false);

  // Give some time for messages to propagate
  int sleep{0};
  int maxSleep{10};
  while (!receivedMsg && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sleep++;
  }
  EXPECT_EQ(maxSleep, sleep);
  EXPECT_FALSE(receivedMsg);
  receivedMsg = false;

  // Check that there are no forces before the plugin is enabled
  EXPECT_EQ(math::Vector3f(0, 0, 0), upperRightNormalForce);
  EXPECT_EQ(math::Vector3f(0, 0, 0), upperLeftNormalForce);
  EXPECT_EQ(math::Vector3f(0, 0, 0), lowerLeftNormalForce);
  EXPECT_EQ(math::Vector3f(0, 0, 0), lowerRightNormalForce);

  // Enable the plugin
  msgs::Boolean req;
  req.set_data(true);
  bool executed = node.Request("/optical_tactile_sensor/enable", req);
  EXPECT_TRUE(executed);

  // Let the plugin generate data again
  server->Run(true, 2000, false);

  // Give some time for messages to propagate
  sleep = 0;
  // cppcheck-suppress knownConditionTrueFalse
  while (!receivedMsg && sleep < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sleep++;
  }
  EXPECT_NE(maxSleep, sleep);
  EXPECT_TRUE(receivedMsg);
  receivedMsg = false;

  // Check the values of the forces
  EXPECT_EQ(math::Vector3f(-1, 0, 0), upperRightNormalForce);
  EXPECT_EQ(math::Vector3f(-1, 0, 0), upperLeftNormalForce);
  EXPECT_EQ(math::Vector3f(-1, 0, 0), lowerLeftNormalForce);
  EXPECT_EQ(math::Vector3f(-1, 0, 0), lowerRightNormalForce);
}
