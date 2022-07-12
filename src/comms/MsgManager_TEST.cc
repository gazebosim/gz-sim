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
#include <gz/msgs/dataframe.pb.h>

#include <unordered_map>

#include "gz/sim/comms/MsgManager.hh"
#include "helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

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

  // Test subscriber.
  EXPECT_TRUE(msgManager.AddSubscriber("addr1", "model1", "topic1"));
  EXPECT_EQ(1u, msgManager.Data().size());
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr1"));
  EXPECT_EQ(1u, msgManager.Data()["addr1"].subscriptions.size());
  EXPECT_NE(msgManager.Data()["addr1"].subscriptions.end(),
            msgManager.Data()["addr1"].subscriptions.find("topic1"));
  EXPECT_EQ("model1", msgManager.Data()["addr1"].modelName);

  // Try to bind to an address attached to another model.
  EXPECT_FALSE(msgManager.AddSubscriber("addr1", "model2", "topic2"));
  EXPECT_EQ(1u, msgManager.Data().size());
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr1"));
  EXPECT_EQ(1u, msgManager.Data()["addr1"].subscriptions.size());
  EXPECT_NE(msgManager.Data()["addr1"].subscriptions.end(),
            msgManager.Data()["addr1"].subscriptions.find("topic1"));
  EXPECT_EQ("model1", msgManager.Data()["addr1"].modelName);

  // Add an additional topic.
  EXPECT_TRUE(msgManager.AddSubscriber("addr1", "model1", "topic2"));
  EXPECT_EQ(1u, msgManager.Data().size());
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr1"));
  EXPECT_EQ(2u, msgManager.Data()["addr1"].subscriptions.size());
  EXPECT_NE(msgManager.Data()["addr1"].subscriptions.end(),
            msgManager.Data()["addr1"].subscriptions.find("topic1"));
  EXPECT_NE(msgManager.Data()["addr1"].subscriptions.end(),
            msgManager.Data()["addr1"].subscriptions.find("topic2"));
  EXPECT_EQ("model1", msgManager.Data()["addr1"].modelName);

  // Test inbound.
  auto msgIn = std::make_shared<msgs::Dataframe>();
  EXPECT_EQ(msgManager.Data().end(), msgManager.Data().find("addr2"));
  msgManager.AddInbound("addr2", msgIn);
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr2"));
  EXPECT_FALSE(msgManager.Data()["addr2"].inboundMsgs.empty());
  EXPECT_EQ(msgIn, msgManager.Data()["addr2"].inboundMsgs[0]);

  // Test outbound.
  auto msgOut = std::make_shared<msgs::Dataframe>();
  EXPECT_EQ(msgManager.Data().end(), msgManager.Data().find("addr3"));
  msgManager.AddOutbound("addr3", msgOut);
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr3"));
  EXPECT_FALSE(msgManager.Data()["addr3"].outboundMsgs.empty());
  EXPECT_EQ(msgOut, msgManager.Data()["addr3"].outboundMsgs[0]);

  // Test msg removal.
  EXPECT_FALSE(msgManager.RemoveInbound("not_found", msgIn));
  EXPECT_TRUE(msgManager.RemoveInbound("addr2", msgIn));
  EXPECT_TRUE(msgManager.Data()["addr2"].inboundMsgs.empty());
  EXPECT_FALSE(msgManager.RemoveOutbound("not_found", msgOut));
  EXPECT_TRUE(msgManager.RemoveOutbound("addr3", msgOut));
  EXPECT_TRUE(msgManager.Data()["addr3"].outboundMsgs.empty());

  // Test msg delivery.
  msgManager.AddInbound("addr4", msgIn);
  EXPECT_FALSE(msgManager.Data()["addr4"].inboundMsgs.empty());
  msgManager.DeliverMsgs();
  EXPECT_TRUE(msgManager.Data()["addr4"].inboundMsgs.empty());

  // Test subscriber removal.
  msgManager.RemoveSubscriber("addr1", "topic1");
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr1"));
  EXPECT_EQ(1u, msgManager.Data()["addr1"].subscriptions.size());
  EXPECT_NE(msgManager.Data()["addr1"].subscriptions.end(),
            msgManager.Data()["addr1"].subscriptions.find("topic2"));
  msgManager.RemoveSubscriber("addr1", "topic2");
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr1"));
  EXPECT_EQ(0u, msgManager.Data()["addr1"].subscriptions.size());
  EXPECT_TRUE(msgManager.Data()["addr1"].modelName.empty());

  // Without subscribers, we should be able to recycle the address for a
  // different model.
  EXPECT_TRUE(msgManager.AddSubscriber("addr1", "model2", "topic2"));
  EXPECT_EQ(4u, msgManager.Data().size());
  EXPECT_NE(msgManager.Data().end(), msgManager.Data().find("addr1"));
  EXPECT_EQ(1u, msgManager.Data()["addr1"].subscriptions.size());
  EXPECT_NE(msgManager.Data()["addr1"].subscriptions.end(),
            msgManager.Data()["addr1"].subscriptions.find("topic2"));
  EXPECT_EQ("model2", msgManager.Data()["addr1"].modelName);

  // Test setting msg content.
  auto msgIn2 = std::make_shared<msgs::Dataframe>();
  auto msgOut2 = std::make_shared<msgs::Dataframe>();
  std::unordered_map<std::string, comms::AddressContent> content;
  content["addr6"].inboundMsgs.push_back(msgIn2);
  content["addr6"].outboundMsgs.push_back(msgOut2);
  msgManager.Set(content);
  EXPECT_TRUE(msgManager.Data()["addr6"].subscriptions.empty());
  EXPECT_EQ(1u, msgManager.Data()["addr6"].inboundMsgs.size());
  EXPECT_EQ(msgIn2, msgManager.Data()["addr6"].inboundMsgs[0u]);
  EXPECT_EQ(1u, msgManager.Data()["addr6"].outboundMsgs.size());
  EXPECT_EQ(msgOut2, msgManager.Data()["addr6"].outboundMsgs[0u]);

  // Test copy.
  EXPECT_TRUE(msgManager.Copy()["addr6"].subscriptions.empty());
  EXPECT_EQ(1u, msgManager.Copy()["addr6"].inboundMsgs.size());
  EXPECT_EQ(msgIn2, msgManager.Copy()["addr6"].inboundMsgs[0u]);
  EXPECT_EQ(1u, msgManager.Copy()["addr6"].outboundMsgs.size());
  EXPECT_EQ(msgOut2, msgManager.Copy()["addr6"].outboundMsgs[0u]);

  // Test DataConst.
  auto it = msgManager.DataConst().find("addr6");
  EXPECT_TRUE(it->second.subscriptions.empty());

}
