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

#include <gtest/gtest-param-test.h>
#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <mutex>

#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>
#include "gz/sim/Server.hh"
#include "test_config.hh"  // NOLINT(build/include)
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

/////////////////////////////////////////////////
class AcousticCommsTestDefinition
{
public:
  std::string worldFile;
  std::string srcAddr;
  std::string dstAddr;
  int numMsgsSent;
  int numMsgsReceived;

  AcousticCommsTestDefinition(
    std::string worldFile_, std::string srcAddr_,
    std::string dstAddr_, int numMsgsSent_,
    int numMsgsReceived_):
    worldFile(worldFile_), srcAddr(srcAddr_),
    dstAddr(dstAddr_), numMsgsSent(numMsgsSent_),
    numMsgsReceived(numMsgsReceived_) {}
};

/////////////////////////////////////////////////
class AcousticCommsTestFixture :
  public ::testing::TestWithParam<AcousticCommsTestDefinition>
{
};

TEST_P(AcousticCommsTestFixture,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(AcousticCommsTestTemplate))
{
  auto param = GetParam();

  // Start server
  ServerConfig serverConfig;
  const auto sdfFile =
    gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "examples", "worlds", param.worldFile);
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Run server
  size_t iters = 1000;
  server.Run(true, iters, false);

  int msgCounter = 0;
  std::mutex mutex;
  auto cb = [&](const msgs::Dataframe &_msg) -> void
  {
    // Verify msg content
    std::lock_guard<std::mutex> lock(mutex);
    std::string expected = std::to_string(msgCounter);

    gz::msgs::StringMsg receivedMsg;
    receivedMsg.ParseFromString(_msg.data());
    EXPECT_EQ(expected, receivedMsg.data());
    msgCounter++;
  };

  // Create subscriber.
  gz::transport::Node node;
  std::string addr  = param.dstAddr;
  std::string subscriptionTopic = param.dstAddr + "/rx";

  // Subscribe to a topic by registering a callback.
  auto cbFunc = std::function<void(const msgs::Dataframe &)>(cb);
  EXPECT_TRUE(node.Subscribe(subscriptionTopic, cbFunc))
      << "Error subscribing to topic [" << subscriptionTopic << "]";

  // Create publisher.
  std::string publicationTopic = "/broker/msgs";
  auto pub = node.Advertise<gz::msgs::Dataframe>(publicationTopic);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // Prepare the message.
  gz::msgs::Dataframe msg;
  msg.set_src_address(param.srcAddr);
  msg.set_dst_address(addr);

  // Publish some messages.
  gz::msgs::StringMsg payload;
  int pubCount = param.numMsgsSent;
  for (int i = 0; i < pubCount; ++i)
  {
    // Prepare the payload.
    payload.set_data(std::to_string(i));
    std::string serializedData;
    EXPECT_TRUE(payload.SerializeToString(&serializedData))
        << payload.DebugString();
    msg.set_data(serializedData);
    EXPECT_TRUE(pub.Publish(msg));
    server.Run(true, 100, false);
  }

  // Verify subscriber received all msgs.
  bool done = false;
  for (int sleep = 0; !done && sleep < 3; ++sleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    {
      std::lock_guard<std::mutex> lock(mutex);
      done = (msgCounter == pubCount) && (pubCount != 0);
    }
  }
  if (param.numMsgsSent != 0 && param.numMsgsSent == param.numMsgsReceived)
  {
    EXPECT_TRUE(done);
  }
  EXPECT_EQ(param.numMsgsReceived, msgCounter);
}

INSTANTIATE_TEST_SUITE_P(
  AcousticCommsTests,
  AcousticCommsTestFixture,
  ::testing::Values(
    AcousticCommsTestDefinition(
      "acoustic_comms.sdf", "addr2", "addr1", 3, 3),
    // 3 msgs will be sent, but 0 will be received as
    // the distance between endpoints exceeds the max range set in the plugin.
    AcousticCommsTestDefinition(
      "acoustic_comms.sdf", "addr4", "addr3", 3, 0),
    // The source is moving and the destination is stationary.
    AcousticCommsTestDefinition(
      "acoustic_comms_moving_targets.sdf", "addr2", "addr1", 3, 3),
    // The source is stationary and the destnation is moving.
    AcousticCommsTestDefinition(
      "acoustic_comms_moving_targets.sdf", "addr4", "addr3", 3, 3),
    // Message packets will be dropped as they are sent too fast
    // compared to "collision_time_interval".
    AcousticCommsTestDefinition(
      "acoustic_comms_packet_collision.sdf", "addr2", "addr1", 3, 1),
    // Source power is decreased and noise bumped up to make the packets
    // drop.
    AcousticCommsTestDefinition(
      "acoustic_comms_propagation.sdf", "addr2", "addr1", 3, 0)
    )
);
