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

#include <ignition/msgs/dataframe.pb.h>
#include <ignition/msgs/stringmsg_v.pb.h>
#include "ignition/gazebo/comms/Broker.hh"
#include "ignition/gazebo/comms/MsgManager.hh"
#include "helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Tests for Broker class
class BrokerTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(BrokerTest, Broker)
{
  comms::Broker broker;

  // Test locking / unlocking and accessing data.
  EXPECT_NO_THROW(broker.Start());
  EXPECT_NO_THROW(broker.Lock());
  auto &allData = broker.DataManager().Data();
  EXPECT_TRUE(allData.empty());
  EXPECT_NO_THROW(broker.Unlock());

  // Test manually binding address and topic.
  msgs::StringMsg_V reqBind;
  reqBind.add_data("addr1");
  reqBind.add_data("model1");
  reqBind.add_data("topic");
  broker.OnBind(reqBind);
  allData = broker.DataManager().Data();
  EXPECT_EQ(1u, allData.size());
  EXPECT_EQ(1u, allData["addr1"].subscriptions.size());
  EXPECT_NE(allData["addr1"].subscriptions.end(),
      allData["addr1"].subscriptions.find("topic"));

  // Test manually adding a msg.
  EXPECT_TRUE(allData["addr1"].outboundMsgs.empty());
  msgs::Dataframe msg;
  msg.set_src_address("addr1");
  broker.OnMsg(msg);
  EXPECT_EQ(1u, allData["addr1"].outboundMsgs.size());
  EXPECT_EQ("addr1", allData["addr1"].outboundMsgs[0u]->src_address());

  // Test manually unbinding address and topic.
  msgs::StringMsg_V reqUnbind;
  reqUnbind.add_data("addr1");
  reqUnbind.add_data("topic");
  broker.OnUnbind(reqUnbind);
  EXPECT_EQ(1u, allData.size());
  EXPECT_TRUE(allData["addr1"].subscriptions.empty());

  // Test msg delivery.
  auto msgIn = std::make_shared<msgs::Dataframe>();
  allData["addr2"].inboundMsgs.push_back(msgIn);
  EXPECT_EQ(1u, allData["addr2"].inboundMsgs.size());
  broker.DeliverMsgs();
  EXPECT_TRUE(allData["addr2"].inboundMsgs.empty());
}
