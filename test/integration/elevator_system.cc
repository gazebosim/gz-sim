/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

/*
 * \author Nick Lamprianidis <nlamprian@gmail.com>
 * \date January 2021
 */

#include <gtest/gtest.h>

#include <gz/msgs/int32.pb.h>

#include <gz/transport/Node.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

/// \brief Test fixture for Elevator system
class ElevatorTestFixture : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("GZ_SIM_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }

  /// \brief Callback for the state messages
  public: void OnStateMsg(const msgs::Int32 &msg)
  {
    if (this->states.empty() || msg.data() != this->states.back())
      this->states.push_back(msg.data());
  }

  /// \brief Communication node
  protected: transport::Node node;

  /// \brief Accummulates the states published by the system
  protected: std::vector<int32_t> states;
};

/////////////////////////////////////////////////
// Tests that the elevator accepts commands and the state is updated correctly
TEST_F(ElevatorTestFixture, Interface)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                          "/test/worlds/elevator.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  server.SetUpdatePeriod(0ns);

  // Make sure the elevator state topic is advertised
  const std::size_t initIters = 100;
  server.Run(true, initIters, false);

  // Subscribe to the elevator state topic
  std::string stateTopic = "/model/elevator/state";
  EXPECT_TRUE(this->node.Subscribe(stateTopic, &ElevatorTestFixture::OnStateMsg,
                                   static_cast<ElevatorTestFixture *>(this)));

  // Check that the state is initialized correctly
  server.Run(true, initIters, false);
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(this->states, (std::vector<int32_t>{0}));

  // Advertise elevator command topic
  auto cmdPub = this->node.Advertise<msgs::Int32>("/model/elevator/cmd");

  // Publish command to move the elevator up
  msgs::Int32 msg;
  msg.set_data(2);
  cmdPub.Publish(msg);
  std::this_thread::sleep_for(100ms);

  // Check that the state has progressed
  const std::size_t runIters = 25000;
  server.Run(true, runIters, false);
  EXPECT_EQ(this->states, (std::vector<int32_t>{0, 1, 2}));

  // Publish command to move the elevator down
  msg.set_data(1);
  cmdPub.Publish(msg);
  std::this_thread::sleep_for(100ms);

  // Check that the state has progressed
  server.Run(true, runIters, false);
  EXPECT_EQ(this->states, (std::vector<int32_t>{0, 1, 2, 1}));
}
