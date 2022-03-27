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
#include <ignition/common/Console.hh>
#include <ignition/msgs/dataframe.pb.h>
#include "ignition/gazebo/comms/MsgManager.hh"
#include "helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

/// \brief Tests for MsgManager class
class MsgManagerTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(MsgManagerTest, MsgManager)
{
  comms::MsgManager msgManager;

  EXPECT_TRUE(msgManager.Data().empty());
  EXPECT_TRUE(msgManager.Copy().empty());

  // test subscriber
  msgManager.AddSubscriber("addr1", "model1", "topic1");
  EXPECT_EQ(1u, msgManager.Data().size());
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr1"));
  EXPECT_NE(msgManager.Data()["addr1"].subscriptions.end(),
            msgManager.Data()["addr1"].subscriptions.find("topic1"));

  // test inbound
  auto msgIn = std::make_shared<msgs::Dataframe>();
  EXPECT_EQ(msgManager.Data().end(), msgManager.Data().find("addr2"));
  msgManager.AddInbound("addr2", msgIn);
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr2"));
  EXPECT_FALSE(msgManager.Data()["addr2"].inboundMsgs.empty());
  EXPECT_EQ(msgIn, msgManager.Data()["addr2"].inboundMsgs[0]);

  // test outbound
  auto msgOut = std::make_shared<msgs::Dataframe>();
  EXPECT_EQ(msgManager.Data().end(), msgManager.Data().find("addr3"));
  msgManager.AddOutbound("addr3", msgOut);
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr3"));
  EXPECT_FALSE(msgManager.Data()["addr3"].outboundMsgs.empty());
  EXPECT_EQ(msgOut, msgManager.Data()["addr3"].outboundMsgs[0]);

  // test msg removal
  msgManager.RemoveInbound("addr2", msgIn);
  EXPECT_TRUE(msgManager.Data()["addr2"].inboundMsgs.empty());
  msgManager.RemoveOutbound("addr3", msgOut);
  EXPECT_TRUE(msgManager.Data()["addr3"].outboundMsgs.empty());

  // test msg delivery
  msgManager.AddInbound("addr4", msgIn);
  EXPECT_FALSE(msgManager.Data()["addr4"].inboundMsgs.empty());
  msgManager.DeliverMsgs();
  EXPECT_TRUE(msgManager.Data()["addr4"].inboundMsgs.empty());

  // test subscriber removal
  msgManager.RemoveSubscriber("addr1", "topic1");
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr1"));
  EXPECT_TRUE(msgManager.Data()["addr1"].subscriptions.empty());

  // test setting msg content
  auto msgIn2 = std::make_shared<msgs::Dataframe>();
  auto msgOut2 = std::make_shared<msgs::Dataframe>();
  std::map<std::string, comms::AddressContent> content;
  content["addr6"].inboundMsgs.push_back(msgIn2);
  content["addr6"].outboundMsgs.push_back(msgOut2);
  msgManager.Set(content);
  EXPECT_TRUE(msgManager.Data()["addr6"].subscriptions.empty());
  EXPECT_EQ(1u, msgManager.Data()["addr6"].inboundMsgs.size());
  EXPECT_EQ(msgIn2, msgManager.Data()["addr6"].inboundMsgs[0u]);
  EXPECT_EQ(1u, msgManager.Data()["addr6"].outboundMsgs.size());
  EXPECT_EQ(msgOut2, msgManager.Data()["addr6"].outboundMsgs[0u]);

  // test copy
  EXPECT_TRUE(msgManager.Copy()["addr6"].subscriptions.empty());
  EXPECT_EQ(1u, msgManager.Copy()["addr6"].inboundMsgs.size());
  EXPECT_EQ(msgIn2, msgManager.Copy()["addr6"].inboundMsgs[0u]);
  EXPECT_EQ(1u, msgManager.Copy()["addr6"].outboundMsgs.size());
  EXPECT_EQ(msgOut2, msgManager.Copy()["addr6"].outboundMsgs[0u]);
}
