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

#include <gz/common/Filesystem.hh>

#include "gz/sim/components/Environment.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/TestFixture.hh"

#include <gz/msgs/double.pb.h>

#include "gz/transport/Node.hh"
#include "test_config.hh"
#include "../helpers/EnvTestFixture.hh"
#include "../helpers/Relay.hh"

using namespace gz;
using namespace sim;

/// \brief Test EnvironmentPreload system
class EnvironmentSensorTest : public InternalFixture<::testing::Test>
{
    /////////////////////////////////////////////////
    public: EnvironmentSensorTest()
    {
        node.Subscribe("/world/environmental_sensor_test/"
            "model/model_with_sensor/link/link/sensor/custom_sensor/"
            "environmental_sensor/humidity",
            &EnvironmentSensorTest::OnRecieveMsg,
            this);
    }

    /////////////////////////////////////////////////
    public: void OnRecieveMsg(const gz::msgs::Double& msg)
    {
        // Data set is such that sensor value == time for the first second.
        double nsec = msg.header().stamp().nsec();
        double sec = msg.header().stamp().sec();
        auto time = nsec * 1e-9 + sec;
        EXPECT_NEAR(time, msg.data(), 1e-9);
        recieved_data = true;
    }

    /////////////////////////////////////////////////
    public: std::atomic<bool> recieved_data{false};

    /////////////////////////////////////////////////
    public: transport::Node node;

};

/////////////////////////////////////////////////
TEST_F(EnvironmentSensorTest, CanPreload)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
      std::string(PROJECT_BINARY_PATH), "test",
      "worlds", "environmental_sensor.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  // Run server
  server.Run(true, 1000, false);
  EXPECT_TRUE(this->recieved_data.load());
}