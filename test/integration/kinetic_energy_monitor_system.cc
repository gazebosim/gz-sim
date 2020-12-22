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

#include <ignition/msgs/double.pb.h>
#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test Kinetic Energy Monitor system
class KineticEnergyMonitorTest : public ::testing::Test
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
std::vector<msgs::Double> dblMsgs;

/////////////////////////////////////////////////
void cb(const msgs::Double &_msg)
{
  mutex.lock();
  dblMsgs.push_back(_msg);
  mutex.unlock();
}

/////////////////////////////////////////////////
// The test checks the world pose and sensor readings of a falling altimeter
TEST_F(KineticEnergyMonitorTest, ModelFalling)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/altimeter.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::string sensorName = "altimeter_sensor";

  // subscribe to kinetic energy topic
  std::string topic = "/model/altimeter_model/kinetic_energy";
  transport::Node node;
  node.Subscribe(topic, &cb);

  // Run server
  size_t iters = 1000u;
  server.Run(true, iters, false);

  // Wait for messages to be received
  for (int sleep = 0; sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mutex.lock();
    bool received = dblMsgs.size() > 0;
    mutex.unlock();

    if (received)
      break;
  }

  mutex.lock();
  EXPECT_EQ(1u, dblMsgs.size());
  auto firstMsg = dblMsgs.front();
  mutex.unlock();
  EXPECT_GT(firstMsg.data(), 2);
}
