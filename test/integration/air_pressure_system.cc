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

#include <ignition/msgs/fluid_pressure.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Test AirPressureTest system
class AirPressureTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

/////////////////////////////////////////////////
TEST_F(AirPressureTest, AirPressure)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/air_pressure.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Subscribe to air_pressure topic
  bool received{false};
  msgs::FluidPressure msg;
  msg.Clear();
  std::function<void(const msgs::FluidPressure &)>  cb =
      [&received, &msg](const msgs::FluidPressure &_msg)
  {
    // Only need one message
    if (received)
      return;

    msg = _msg;
    received = true;
  };

  transport::Node node;
  node.Subscribe(std::string("/world/air_pressure_sensor/model/") +
      "air_pressure_model/link/link/sensor/air_pressure_sensor/air_pressure",
      cb);

  // Run server
  server.Run(true, 100, false);

  // Wait for message to be received
  for (int sleep = 0; !received && sleep < 30; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_TRUE(received);

  // check air pressure
  EXPECT_TRUE(msg.has_header());
  EXPECT_TRUE(msg.header().has_stamp());
  EXPECT_EQ(0, msg.header().stamp().sec());
  EXPECT_LT(0, msg.header().stamp().nsec());
  EXPECT_DOUBLE_EQ(101325.0, msg.pressure());
  EXPECT_DOUBLE_EQ(0.0, msg.variance());
}
