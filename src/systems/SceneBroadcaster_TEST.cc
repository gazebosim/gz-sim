/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#include <google/protobuf/util/message_differencer.h>

#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"

using namespace ignition;

/// \brief Test SceneBroadcaster system
class SceneBroadcasterTest : public ::testing::TestWithParam<int>
{
  // Documentation inherited
  protected: virtual void SetUp() override
  {
    common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

/////////////////////////////////////////////////
TEST_P(SceneBroadcasterTest, PoseInfo)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(14u, *server.EntityCount());

  // Create pose subscriber
  transport::Node node;

  bool received{false};
  std::function<void(const msgs::Pose_V &)> cb = [&](const msgs::Pose_V &_msg)
  {
    EXPECT_EQ(10, _msg.pose_size());

    std::map<int, std::string> entityMap;
    for (auto p = 0; p < _msg.pose_size(); ++p)
    {
      entityMap.insert(std::make_pair(_msg.pose(p).id(), _msg.pose(p).name()));
    }

    EXPECT_EQ(10u, entityMap.size());

    received = true;
  };
  EXPECT_TRUE(node.Subscribe("/world/default/pose/info", cb));

  // Run server
  server.Run(true, 1, false);

  unsigned int sleep{0u};
  unsigned int maxSleep{10u};
  while (!received && sleep++ < maxSleep)
    IGN_SLEEP_MS(100);

  EXPECT_TRUE(received);
}

/////////////////////////////////////////////////
TEST_P(SceneBroadcasterTest, SceneInfo)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(14u, *server.EntityCount());

  // Run server
  server.Run(true, 1, false);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  ignition::msgs::Scene res;

  EXPECT_TRUE(node.Request("/world/default/scene/info", timeout, res, result));
  EXPECT_TRUE(result);

  EXPECT_EQ(3, res.model_size());

  for (auto m = 0; m < res.model_size(); ++m)
  {
    ASSERT_EQ(1, res.model(m).link_size());
    EXPECT_EQ(res.model(m).name() + "_link", res.model(m).link(0).name());

    ASSERT_EQ(1, res.model(m).link(0).visual_size());
    EXPECT_EQ(res.model(m).name() + "_visual",
        res.model(m).link(0).visual(0).name());
  }

  // Repeat the request to make sure the same information is returned
  ignition::msgs::Scene res2;
  EXPECT_TRUE(node.Request("/world/default/scene/info", timeout, res2, result));
  EXPECT_TRUE(result);

  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(res, res2));
}

/////////////////////////////////////////////////
TEST_P(SceneBroadcasterTest, SceneGraph)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(14u, *server.EntityCount());

  // Run server
  server.Run(true, 1, false);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  ignition::msgs::StringMsg res;

  EXPECT_TRUE(node.Request("/world/default/scene/graph", timeout, res, result));
  EXPECT_TRUE(result);

  EXPECT_FALSE(res.data().empty());
  EXPECT_NE(res.data().find("default (0)"), std::string::npos);
  EXPECT_NE(res.data().find("box (1)"), std::string::npos);
  EXPECT_NE(res.data().find("box_link (2)"), std::string::npos);
  EXPECT_NE(res.data().find("box_visual (3)"), std::string::npos);
  EXPECT_NE(res.data().find("cylinder (5)"), std::string::npos);
  EXPECT_NE(res.data().find("cylinder_link (6)"), std::string::npos);
  EXPECT_NE(res.data().find("cylinder_visual (7)"), std::string::npos);
  EXPECT_NE(res.data().find("sphere (9)"), std::string::npos);
  EXPECT_NE(res.data().find("sphere_link (10)"), std::string::npos);
  EXPECT_NE(res.data().find("sphere_visual (11)"), std::string::npos);
}

/////////////////////////////////////////////////
/// Test whether the scene topic is published only when new entities are added
TEST_P(SceneBroadcasterTest, SceneTopic)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                          "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(14u, *server.EntityCount());

  // Create requester
  transport::Node node;

  std::vector<msgs::Scene> sceneMsgs;
  std::function<void(const msgs::Scene &)> collectMsgs =
      [&sceneMsgs](const msgs::Scene &_msg)
      {
        sceneMsgs.push_back(_msg);
      };

  node.Subscribe("/world/default/scene/info", collectMsgs);

  // Run server
  server.Run(true, 10, false);

  // Should only have one scene even though the simulation ran multiple times
  ASSERT_EQ(1u, sceneMsgs.size());

  // Compare this scene with one from a service request
  msgs::Scene &scene = sceneMsgs.front();

  bool result{false};
  unsigned int timeout{5000};
  ignition::msgs::Scene msg;

  EXPECT_TRUE(node.Request("/world/default/scene/info", timeout, msg, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(msg, scene));
}

/////////////////////////////////////////////////
/// Test whether the scene topic is published only when new entities are added
TEST_P(SceneBroadcasterTest, DeletedTopic)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                          "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::size_t initEntityCount = 14;
  EXPECT_EQ(initEntityCount, *server.EntityCount());

  // Subscribe to deletions
  transport::Node node;

  std::vector<msgs::UInt32_V> deletionMsgs;
  std::function<void(const msgs::UInt32_V &)> collectMsgs =
      [&deletionMsgs](const msgs::UInt32_V &_msg)
      {
        deletionMsgs.push_back(_msg);
      };

  node.Subscribe("/world/default/scene/deletion", collectMsgs);

  auto cylinderModelId = server.EntityByName("cylinder");
  auto cylinderLinkId = server.EntityByName("cylinder_link");
  ASSERT_TRUE(cylinderModelId.has_value());
  ASSERT_TRUE(cylinderLinkId.has_value());

  EXPECT_EQ(0u, deletionMsgs.size());

  // Run server
  server.Run(true, 1, false);
  EXPECT_EQ(0u, deletionMsgs.size());

  // Delete the cylinder. Deleting the model and the link to avoid physics
  // warnings
  server.RequestEraseEntity(cylinderModelId.value());
  server.RequestEraseEntity(cylinderLinkId.value());
  server.Run(true, 10, false);

  EXPECT_EQ(initEntityCount - 2, server.EntityCount());

  ASSERT_EQ(1u, deletionMsgs.size());

  auto delMsg = deletionMsgs.front();

  // The id of the deleted entity should have been published
  // Note: Only model entities are currently supported for deletion
  EXPECT_TRUE(std::find_if(delMsg.data().cbegin(), delMsg.data().cend(),
      [&cylinderModelId](const auto &_val)
      {
        return _val == cylinderModelId;
      }));
}
// Run multiple times
INSTANTIATE_TEST_CASE_P(ServerRepeat, SceneBroadcasterTest,
    ::testing::Range(1, 2));
