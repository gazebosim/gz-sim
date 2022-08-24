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
class AcousticCommsTest : public InternalFixture<::testing::Test>
{
  public:
    void CommsTestFixture(
        std::string worldFile,
        std::string srcAddr, std::string dstAddr,
        int numMsgs = 3)
    {
      // Start server
      ServerConfig serverConfig;
      const auto sdfFile =
        gz::common::joinPaths(std::string(PROJECT_SOURCE_PATH),
          "examples", "worlds", worldFile);
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
        std::string expected = "hello world " + std::to_string(msgCounter);

        gz::msgs::StringMsg receivedMsg;
        receivedMsg.ParseFromString(_msg.data());
        EXPECT_EQ(expected, receivedMsg.data());
        msgCounter++;
      };

      // Create subscriber.
      gz::transport::Node node;
      std::string addr  = dstAddr;
      std::string subscriptionTopic = dstAddr + "/rx";

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
      msg.set_src_address(srcAddr);
      msg.set_dst_address(addr);

      // Publish some messages.
      gz::msgs::StringMsg payload;
      int pubCount = numMsgs;
      for (int i = 0; i < pubCount; ++i)
      {
        // Prepare the payload.
        payload.set_data("hello world " + std::to_string(i));
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
      if (numMsgs != 0) 
      {
        EXPECT_TRUE(done);
      }
      EXPECT_EQ(pubCount, msgCounter);
    }
};

/////////////////////////////////////////////////
TEST_F(AcousticCommsTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(AcousticCommsInRange))
{
  this->CommsTestFixture("acoustic_comms.sdf", "addr2", "addr1");
}

/////////////////////////////////////////////////
TEST_F(AcousticCommsTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(AcousticCommsMovingSource))
{
  // The source is moving and the destination is stationary.
  this->CommsTestFixture(
      "acoustic_comms_moving_targets.sdf",
      "addr2", "addr1");
}

/////////////////////////////////////////////////
TEST_F(AcousticCommsTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(AcousticCommsMovingDst))
{
  // The source is stationary and the destnation is moving.
  this->CommsTestFixture(
      "acoustic_comms_moving_targets.sdf",
      "addr4", "addr3");
}

/////////////////////////////////////////////////
TEST_F(AcousticCommsTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(AcousticCommsOutOfRange))
{
  // Source and destination are outside the maximum allowed range
  // and hence should not receive any msgs.
  this->CommsTestFixture("acoustic_comms.sdf", "addr4", "addr3", 0);
}
