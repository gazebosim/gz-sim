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

#include <thread>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/EnvTestFixture.hh"

using namespace ignition;

/// \brief Test SceneBroadcaster system
class SceneBroadcasterTest
  : public InternalFixture<::testing::TestWithParam<int>>
{
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
  EXPECT_EQ(24u, *server.EntityCount());

  // Create pose subscriber
  transport::Node node;

  bool received{false};
  std::function<void(const msgs::Pose_V &)> cb = [&](const msgs::Pose_V &_msg)
  {
    ASSERT_TRUE(_msg.has_header());
    ASSERT_TRUE(_msg.header().has_stamp());
    EXPECT_LT(0, _msg.header().stamp().sec() +  _msg.header().stamp().nsec());

    EXPECT_EQ(16, _msg.pose_size());

    std::map<int, std::string> entityMap;
    for (auto p = 0; p < _msg.pose_size(); ++p)
    {
      entityMap.insert(std::make_pair(_msg.pose(p).id(), _msg.pose(p).name()));
    }

    EXPECT_EQ(16u, entityMap.size());

    received = true;
  };
  EXPECT_TRUE(node.Subscribe("/world/default/pose/info", cb));

  // Run server
  server.Run(true, 1, false);

  unsigned int sleep{0u};
  unsigned int maxSleep{30u};
  // cppcheck-suppress unmatchedSuppression
  // cppcheck-suppress knownConditionTrueFalse
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
  EXPECT_EQ(24u, *server.EntityCount());

  // Run server
  server.Run(true, 1, false);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  ignition::msgs::Scene res;

  EXPECT_TRUE(node.Request("/world/default/scene/info", timeout, res, result));
  EXPECT_TRUE(result);

  EXPECT_EQ(5, res.model_size());

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
  EXPECT_EQ(24u, *server.EntityCount());

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
  EXPECT_NE(res.data().find("default (1)"), std::string::npos);
  EXPECT_NE(res.data().find("box (4)"), std::string::npos);
  EXPECT_NE(res.data().find("box_link (5)"), std::string::npos);
  EXPECT_NE(res.data().find("box_visual (6)"), std::string::npos);
  EXPECT_NE(res.data().find("cylinder (8)"), std::string::npos);
  EXPECT_NE(res.data().find("cylinder_link (9)"), std::string::npos);
  EXPECT_NE(res.data().find("cylinder_visual (10)"), std::string::npos);
  EXPECT_NE(res.data().find("sphere (12)"), std::string::npos);
  EXPECT_NE(res.data().find("sphere_link (13)"), std::string::npos);
  EXPECT_NE(res.data().find("sphere_visual (14)"), std::string::npos);
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
  EXPECT_EQ(24u, *server.EntityCount());

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
TEST_P(SceneBroadcasterTest, SceneTopicSensors)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                          "/test/worlds/altimeter_with_pose.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(12u, *server.EntityCount());

  // Create requester
  transport::Node node;

  std::vector<msgs::Scene> sceneMsgs;
  std::function<void(const msgs::Scene &)> collectMsgs =
      [&sceneMsgs](const msgs::Scene &_msg)
      {
        sceneMsgs.push_back(_msg);
      };

  node.Subscribe("/world/altimeter_sensor/scene/info", collectMsgs);

  // Run server
  server.Run(true, 10, false);

  // Should only have one scene even though the simulation ran multiple times
  ASSERT_EQ(1u, sceneMsgs.size());

  // Compare this scene with one from a service request
  msgs::Scene &scene = sceneMsgs.front();

  bool result{false};
  unsigned int timeout{5000};
  ignition::msgs::Scene msg;

  EXPECT_TRUE(node.Request("/world/altimeter_sensor/scene/info",
        timeout, msg, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(msg, scene));

  EXPECT_EQ(1, msg.model(1).link(0).sensor_size());
  EXPECT_EQ("altimeter_sensor", msg.model(1).link(0).sensor(0).name());
  EXPECT_DOUBLE_EQ(0.1, msg.model(1).link(0).sensor(0).pose().position().x());
  EXPECT_DOUBLE_EQ(0.2, msg.model(1).link(0).sensor(0).pose().position().y());
  EXPECT_DOUBLE_EQ(0.3, msg.model(1).link(0).sensor(0).pose().position().z());
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

  const std::size_t initEntityCount = 24;
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
  server.RequestRemoveEntity(cylinderModelId.value(), false);
  server.RequestRemoveEntity(cylinderLinkId.value(), false);
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

/////////////////////////////////////////////////
/// Test whether the scene is updated when a model is spawned.
TEST_P(SceneBroadcasterTest, SpawnedModel)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                          "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  const std::size_t initEntityCount = 24;
  EXPECT_EQ(initEntityCount, *server.EntityCount());

  server.Run(true, 1, false);

  transport::Node node;

  // Spawn a model
  {
    auto modelStr = R"(
<?xml version="1.0" ?>
<sdf version='1.6'>
  <model name='spawned_model'>
    <link name='link'>
      <visual name='visual'>
        <geometry><sphere><radius>1.0</radius></sphere></geometry>
      </visual>
    </link>
  </model>
</sdf>)";

    msgs::EntityFactory req;
    msgs::Boolean res;
    bool result;
    unsigned int timeout = 5000;
    req.set_sdf(modelStr);
    EXPECT_TRUE(node.Request("/world/default/create",
          req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());
  }

  // Iterate once so that the model can get spawned.
  server.Run(true, 1, false);


  // Check that the model is in the scene/infor response
  {
    ignition::msgs::Empty req;
    ignition::msgs::Scene rep;
    bool result;
    unsigned int timeout = 2000;
    EXPECT_TRUE(node.Request("/world/default/scene/info", req, timeout,
          rep, result));
    EXPECT_TRUE(result);

    bool found = false;
    for (int i = 0; i < rep.model_size(); ++i)
    {
      found = rep.model(i).name() == "spawned_model";
      if (found)
        break;
    }
    EXPECT_TRUE(found);
  }
  EXPECT_EQ(initEntityCount + 3, *server.EntityCount());
}

/////////////////////////////////////////////////
TEST_P(SceneBroadcasterTest, State)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/shapes.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(24u, *server.EntityCount());
  transport::Node node;

  // Run server
  server.Run(true, 1, false);

  bool received{false};
  auto checkMsg = [&](const msgs::SerializedStepMap &_msg, int _count)
  {
    // Stats
    ASSERT_TRUE(_msg.has_stats());
    EXPECT_FALSE(_msg.stats().paused());
    EXPECT_LT(1u, _msg.stats().iterations());
    EXPECT_EQ(0, _msg.stats().step_size().sec());
    EXPECT_EQ(1000000, _msg.stats().step_size().nsec());
    EXPECT_LT(0, _msg.stats().sim_time().nsec());
    EXPECT_LT(0, _msg.stats().real_time().nsec());

    // State
    ASSERT_TRUE(_msg.has_state());

    EXPECT_EQ(_count, _msg.state().entities_size());

    received = true;
  };

  std::function<void(const msgs::SerializedStepMap &, const bool)> cb =
      [&](const msgs::SerializedStepMap &_res, const bool _success)
  {
    EXPECT_TRUE(_success);
    checkMsg(_res, 24);
  };
  std::function<void(const msgs::SerializedStepMap &)> cb2 =
      [&](const msgs::SerializedStepMap &_res)
  {
    checkMsg(_res, 5);
  };

  // async state request with full state response
  std::function<void(const msgs::SerializedStepMap &)> cbAsync =
      [&](const msgs::SerializedStepMap &_res)
  {
    checkMsg(_res, 24);
  };

  // The request is blocking even though it's meant to be async, so we spin a
  // thread
  auto request = [&]()
  {
    EXPECT_TRUE(node.Request("/world/default/state", cb));
  };
  auto requestThread = std::thread(request);

  // Run server
  unsigned int sleep{0u};
  unsigned int maxSleep{30u};
  while (!received && sleep++ < maxSleep)
  {
    IGN_SLEEP_MS(100);
    server.Run(true, 1, false);
  }

  EXPECT_TRUE(received);
  requestThread.join();

  received = false;
  EXPECT_TRUE(node.Subscribe("/world/default/state", cb2));

  // Run server
  server.Run(true, 1, false);

  sleep = 0;
  // cppcheck-suppress unmatchedSuppression
  // cppcheck-suppress knownConditionTrueFalse
  while (!received && sleep++ < maxSleep)
    IGN_SLEEP_MS(100);
  EXPECT_TRUE(received);
  EXPECT_TRUE(node.Unsubscribe("/world/default/state"));

  // test async state request
  received = false;
  std::string reqSrv = "/state_async_callback_test";
  node.Advertise(reqSrv, cbAsync);

  ignition::msgs::StringMsg req;
  req.set_data(reqSrv);
  node.Request("/world/default/state_async", req);

  sleep = 0;
  // cppcheck-suppress unmatchedSuppression
  // cppcheck-suppress knownConditionTrueFalse
  while (!received && sleep++ < maxSleep)
  {
    // Run server
    server.Run(true, 1, false);
    IGN_SLEEP_MS(100);
  }
  EXPECT_TRUE(received);
}

/////////////////////////////////////////////////
TEST_P(SceneBroadcasterTest, StateStatic)
{
  // Start server
  ignition::gazebo::ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/examples/worlds/empty.sdf");

  gazebo::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(8u, *server.EntityCount());
  transport::Node node;

  // Run server
  server.Run(true, 1, false);

  bool received{false};
  msgs::SerializedStepMap stepMsg;
  auto checkMsg = [&](const msgs::SerializedStepMap &_msg)
  {
    // Stats
    ASSERT_TRUE(_msg.has_stats());
    EXPECT_FALSE(_msg.stats().paused());
    EXPECT_LT(1u, _msg.stats().iterations());
    EXPECT_EQ(0, _msg.stats().step_size().sec());
    EXPECT_EQ(1000000, _msg.stats().step_size().nsec());
    EXPECT_LT(0, _msg.stats().sim_time().nsec());
    EXPECT_LT(0, _msg.stats().real_time().nsec());

    // State
    ASSERT_TRUE(_msg.has_state());

    stepMsg = _msg;
    received = true;
  };

  auto checkNewMsg = [&](const msgs::SerializedStepMap &_msg)
  {
    // Stats
    ASSERT_TRUE(_msg.has_stats());
    EXPECT_FALSE(_msg.stats().paused());
    EXPECT_LT(stepMsg.stats().iterations(), _msg.stats().iterations());
    EXPECT_EQ(0, _msg.stats().step_size().sec());
    EXPECT_EQ(1000000, _msg.stats().step_size().nsec());
    EXPECT_LT(stepMsg.stats().sim_time().nsec(),
        _msg.stats().sim_time().nsec());
    EXPECT_LT(stepMsg.stats().real_time().nsec(),
        _msg.stats().real_time().nsec());

    // State
    ASSERT_TRUE(_msg.has_state());

    received = true;
  };


  std::function<void(const msgs::SerializedStepMap &, const bool)> cb =
      [&](const msgs::SerializedStepMap &_res, const bool _success)
  {
    EXPECT_TRUE(_success);
    checkMsg(_res);
  };
  std::function<void(const msgs::SerializedStepMap &)> cb2 =
      [&](const msgs::SerializedStepMap &_res)
  {
    checkNewMsg(_res);
  };

  // The request is blocking even though it's meant to be async, so we spin a
  // thread
  auto request = [&]()
  {
    EXPECT_TRUE(node.Request("/world/empty/state", cb));
  };
  auto requestThread = std::thread(request);

  // Run server
  unsigned int sleep{0u};
  unsigned int maxSleep{30u};
  while (!received && sleep++ < maxSleep)
  {
    IGN_SLEEP_MS(100);
    server.Run(true, 1, false);
  }

  EXPECT_TRUE(received);
  requestThread.join();

  received = false;
  EXPECT_TRUE(node.Subscribe("/world/empty/state", cb2));

  // Run server
  server.Run(true, 1, false);

  sleep = 0;
  // cppcheck-suppress unmatchedSuppression
  // cppcheck-suppress knownConditionTrueFalse
  while (!received && sleep++ < maxSleep)
    IGN_SLEEP_MS(100);
  EXPECT_TRUE(received);
}

// Run multiple times
INSTANTIATE_TEST_SUITE_P(ServerRepeat, SceneBroadcasterTest,
    ::testing::Range(1, 2));
