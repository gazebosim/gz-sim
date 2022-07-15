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

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "test_config.hh"

#include "../helpers/EnvTestFixture.hh"
#include "../helpers/MessageReceiver.hh"
#include "../helpers/Relay.hh"

using namespace gz;

/// \brief Test SceneBroadcaster system
class SceneBroadcasterTest
  : public InternalFixture<::testing::TestWithParam<int>>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(SceneBroadcasterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(PoseInfo))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);
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
    GZ_SLEEP_MS(100);

  EXPECT_TRUE(received);
}

/////////////////////////////////////////////////
TEST_F(SceneBroadcasterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SceneInfo))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(24u, *server.EntityCount());

  // Run server
  server.Run(true, 1, false);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  gz::msgs::Scene res;

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
  gz::msgs::Scene res2;
  EXPECT_TRUE(node.Request("/world/default/scene/info", timeout, res2, result));
  EXPECT_TRUE(result);

  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(res, res2));
}

/////////////////////////////////////////////////
TEST_F(SceneBroadcasterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SceneGraph))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));
  EXPECT_EQ(24u, *server.EntityCount());

  // Run server
  server.Run(true, 1, false);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  gz::msgs::StringMsg res;

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
TEST_F(SceneBroadcasterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SceneTopic))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);
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
  gz::msgs::Scene msg;

  EXPECT_TRUE(node.Request("/world/default/scene/info", timeout, msg, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(msg, scene));
}

/////////////////////////////////////////////////
/// Test whether the scene topic is published only when new entities are added
TEST_F(SceneBroadcasterTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(SceneTopicSensors))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "altimeter_with_pose.sdf"));

  sim::Server server(serverConfig);
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
  gz::msgs::Scene msg;

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
TEST_F(SceneBroadcasterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(DeletedTopic))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);
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
TEST_F(SceneBroadcasterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SpawnedModel))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);
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
    gz::msgs::Empty req;
    gz::msgs::Scene rep;
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
TEST_F(SceneBroadcasterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(State))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes.sdf"));

  sim::Server server(serverConfig);
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
    GZ_SLEEP_MS(100);
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
    GZ_SLEEP_MS(100);
  EXPECT_TRUE(received);
  EXPECT_TRUE(node.Unsubscribe("/world/default/state"));

  // test async state request
  received = false;
  std::string reqSrv = "/state_async_callback_test";
  node.Advertise(reqSrv, cbAsync);

  gz::msgs::StringMsg req;
  req.set_data(reqSrv);
  node.Request("/world/default/state_async", req);

  sleep = 0;
  // cppcheck-suppress unmatchedSuppression
  // cppcheck-suppress knownConditionTrueFalse
  while (!received && sleep++ < maxSleep)
  {
    // Run server
    server.Run(true, 1, false);
    GZ_SLEEP_MS(100);
  }
  EXPECT_TRUE(received);
}

/////////////////////////////////////////////////
TEST_F(SceneBroadcasterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(StateStatic))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "empty.sdf"));

  sim::Server server(serverConfig);
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
    GZ_SLEEP_MS(100);
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
    GZ_SLEEP_MS(100);
  EXPECT_TRUE(received);
}

enum class State
{
  kNoop,
  kRemoveComponent,
  kAddComponent,
  kRemoveEntity,
  kAddEntity,
  kOneTimeChange,
  kPeriodicChange,
};

/////////////////////////////////////////////////
struct AddRemoveEntitiesComponentsPlugin:
  public gz::sim::System,
  public gz::sim::ISystemUpdate
{

State state = State::kNoop;
    
void Update(const gz::sim::UpdateInfo &,
            gz::sim::EntityComponentManager &_ecm) override
{
  switch(this->state)
  {
    case State::kRemoveComponent:
      this->RemoveComponentFromEntity(_ecm);
      break;
    case State::kAddComponent:
      this->AddComponentToEntity(_ecm);
      break;
    case State::kRemoveEntity:
      this->RemoveEntity(_ecm);
      break;
    case State::kAddEntity:
      this->AddEntity(_ecm);
      break;
    case State::kOneTimeChange:
      this->OneTimeChange(_ecm);
      break;
    case State::kPeriodicChange:
      this->PeriodicChange(_ecm);
      break;
    case State::kNoop:
    default:
      break;
  }
  this->state = State::kNoop;

}

void RemoveComponentFromEntity(gz::sim::EntityComponentManager &_ecm)
{
  igndbg << "RemoveComponentFromEntity" << std::endl;
  _ecm.Each<gz::sim::components::Model,
            gz::sim::components::Name,
            gz::sim::components::Pose>(
    [&](const gz::sim::Entity &_entity,
        const gz::sim::components::Model *,
        const gz::sim::components::Name *_name,
        const gz::sim::components::Pose *)->bool
    {
      if (_name->Data() == "box")
      {
        _ecm.RemoveComponent<gz::sim::components::Pose>(_entity);
      }
      return true;
    });
}

void AddComponentToEntity(gz::sim::EntityComponentManager &_ecm)
{
  igndbg << "AddComponentToEntity" << std::endl;
  auto boxEntity = _ecm.EntityByComponents(
      sim::components::Name("box"), sim::components::Model());
  ASSERT_NE(sim::kNullEntity, boxEntity);
  EXPECT_FALSE(_ecm.EntityHasComponentType(boxEntity,
        gz::sim::components::Pose::typeId));
  _ecm.CreateComponent<gz::sim::components::Pose>(boxEntity,
      gz::sim::components::Pose({1, 2, 3, 4, 5, 6}));
  EXPECT_TRUE(_ecm.EntityHasComponentType(boxEntity,
        gz::sim::components::Pose::typeId));
}

void RemoveEntity(gz::sim::EntityComponentManager &_ecm)
{
  igndbg << "RemoveEntity" << std::endl;
  auto boxEntity = _ecm.EntityByComponents(
      sim::components::Name("box"), sim::components::Model());
  ASSERT_NE(sim::kNullEntity, boxEntity);
  _ecm.RequestRemoveEntity(boxEntity);
}

void AddEntity(gz::sim::EntityComponentManager &_ecm)
{
  igndbg << "AddEntity" << std::endl;
  EXPECT_EQ(sim::kNullEntity, _ecm.EntityByComponents(
        sim::components::Name("newEntity"),
        sim::components::Model()));
  auto newEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(newEntity, sim::components::Name("newEntity"));
  _ecm.CreateComponent(newEntity, sim::components::Model());
  EXPECT_NE(sim::kNullEntity, _ecm.EntityByComponents(
        sim::components::Name("newEntity"),
        sim::components::Model()));
}

void OneTimeChange(gz::sim::EntityComponentManager &_ecm)
{
  igndbg << "OneTimeChange" << std::endl;
  auto entity = _ecm.EntityByComponents(
      sim::components::Name("newEntity"),
      sim::components::Model());
  ASSERT_NE(sim::kNullEntity, entity);
  EXPECT_TRUE(_ecm.SetComponentData<sim::components::Name>(entity,
      "newEntity1"));
  _ecm.SetChanged(entity, sim::components::Name::typeId,
      sim::ComponentState::OneTimeChange);
}

void PeriodicChange(gz::sim::EntityComponentManager &_ecm)
{
    igndbg << "PeriodicChange" << std::endl;
    auto entity = _ecm.EntityByComponents(
        sim::components::Name("newEntity1"),
        sim::components::Model());
    ASSERT_NE(sim::kNullEntity, entity);
    EXPECT_TRUE(_ecm.SetComponentData<sim::components::Name>(entity,
        "newEntity2"));
    _ecm.SetChanged(entity, sim::components::Name::typeId,
        sim::ComponentState::PeriodicChange);
}
};


/////////////////////////////////////////////////
/// Test whether the scene topic is published when entities and components are
/// removed/added
TEST_F(SceneBroadcasterTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(AddRemoveEntitiesComponents))
{
  // Start server
  gz::sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "shapes_scene_broadcaster_only.sdf"));

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  auto testSystem = std::make_shared<AddRemoveEntitiesComponentsPlugin>();
  server.AddSystem(testSystem);

  gz::sim::EntityComponentManager localEcm;

  auto stepReceiver = gz::sim::test::MessageReceiver<msgs::SerializedStepMap>();
  EXPECT_TRUE(stepReceiver.Start("/world/default/state"));

  // Run server once. The first time should send the state message
  server.RunOnce();
  ASSERT_TRUE(stepReceiver.Spin());
  ASSERT_TRUE(stepReceiver.Last().has_state());
  localEcm.SetState(stepReceiver.Last().state());
  stepReceiver.Clear();

  // Run server again. The second time shouldn't have state info. The
  // message can still arrive due the passage of time (see `itsPubTime` in
  // SceneBroadcaster::PostUpdate.
  // Sleep for more than 1/30 of a second (paused update rate) to guarantee
  std::this_thread::sleep_for(std::chrono::milliseconds(33));
  server.RunOnce();
  ASSERT_TRUE(stepReceiver.Spin());
  ASSERT_FALSE(stepReceiver.Last().has_state());
  stepReceiver.Clear();

  // Run server again. The third time should send the state message because
  // the test system removed a component.
  testSystem->state = State::kRemoveComponent;
  server.RunOnce();
  ASSERT_TRUE(stepReceiver.Spin());
  ASSERT_TRUE(stepReceiver.Last().has_state());
  localEcm.SetState(stepReceiver.Last().state());
  stepReceiver.Clear();

  // Run server again. The fourth time should send the state message because
  // the test system added a component.
  testSystem->state = State::kAddComponent;
  server.RunOnce();
  ASSERT_TRUE(stepReceiver.Spin());
  ASSERT_TRUE(stepReceiver.Last().has_state());
  localEcm.SetState(stepReceiver.Last().state());
  stepReceiver.Clear();

  // Run server again. The fifth time should send the state message because
  // the test system requested to remove an entity.
  testSystem->state = State::kRemoveEntity;
  server.RunOnce();
  ASSERT_TRUE(stepReceiver.Spin());
  ASSERT_TRUE(stepReceiver.Last().has_state());
  localEcm.SetState(stepReceiver.Last().state());
  stepReceiver.Clear();

  // Run server again. The sixth time should send the state message because
  // the test system created an entity.
  testSystem->state = State::kAddEntity;
  server.RunOnce();
  ASSERT_TRUE(stepReceiver.Spin());
  ASSERT_TRUE(stepReceiver.Last().has_state());
  localEcm.SetState(stepReceiver.Last().state());
  stepReceiver.Clear();

  // Run server again. The seventh time should send the state message because
  // the test system modified a component and marked it as a OneTimeChange.
  testSystem->state = State::kOneTimeChange;
  server.RunOnce();
  ASSERT_TRUE(stepReceiver.Spin());
  ASSERT_TRUE(stepReceiver.Last().has_state());
  localEcm.SetState(stepReceiver.Last().state());
  stepReceiver.Clear();

  // Run server for a few iterations to make sure that the periodic change
  // made by the test system is received.
  testSystem->state = State::kPeriodicChange;
  server.Run(true, 20, false);
  ASSERT_TRUE(stepReceiver.Spin());
}

/////////////////////////////////////////////////
// Tests https://github.com/gazebosim/gz-sim/issues/1414
TEST_F(SceneBroadcasterTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(DecimalStateHertz))
{
  // Start server
  std::string sdfStr = R"(
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="world with spaces">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
      <state_hertz>0.4</state_hertz>
    </plugin>
    <scene>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
    </scene>
  </world>
</sdf>)";
  sim::ServerConfig serverConfig;
  serverConfig.SetSdfString(sdfStr);

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Run server
  server.Run(true, 1, false);
}

/////////////////////////////////////////////////
TEST_F(SceneBroadcasterTest,
    GZ_UTILS_TEST_DISABLED_ON_WIN32(SceneInfoHasSceneSdf))
{
  // Start server
  sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(
      PROJECT_SOURCE_PATH, "/", "test", "worlds", "conveyor.sdf"));

  sim::Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Run server
  server.Run(true, 1, false);

  // Create requester
  transport::Node node;

  bool result{false};
  unsigned int timeout{5000};
  msgs::Scene res;

  EXPECT_TRUE(node.Request("/world/default/scene/info", timeout, res, result));
  EXPECT_TRUE(result);

  ASSERT_TRUE(res.has_ambient());
  EXPECT_EQ(math::Color(1.0, 1.0, 1.0, 1.0), msgs::Convert(res.ambient()));

  ASSERT_TRUE(res.has_background());
  EXPECT_EQ(math::Color(0.8, 0.8, 0.8, 1.0), msgs::Convert(res.background()));

  EXPECT_TRUE(res.shadows());
  EXPECT_FALSE(res.grid());
  EXPECT_FALSE(res.has_fog());
  EXPECT_FALSE(res.has_sky());
}
