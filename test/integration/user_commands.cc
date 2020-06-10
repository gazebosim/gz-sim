/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include <ignition/msgs/entity_factory.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
class UserCommandsTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, Create)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/examples/worlds/empty.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system just to get the ECM
  // TODO(louise) It would be much more convenient if the Server just returned
  // the ECM for us. This would save all the trouble which is causing us to
  // create `Relay` systems in the first place. Consider keeping the ECM in a
  // shared pointer owned by the SimulationRunner.
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const gazebo::UpdateInfo &,
                             gazebo::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  auto entityCount = ecm->EntityCount();

  // SDF strings
  auto modelStr = std::string("<?xml version=\"1.0\" ?>") +
      "<sdf version='1.6'>" +
      "<model name='spawned_model'>" +
      "<link name='link'>" +
      "<visual name='visual'>" +
      "<geometry><sphere><radius>1.0</radius></sphere></geometry>" +
      "</visual>" +
      "<collision name='visual'>" +
      "<geometry><sphere><radius>1.0</radius></sphere></geometry>" +
      "</collision>" +
      "</link>" +
      "</model>" +
      "</sdf>";

  auto lightStr = std::string("<?xml version='1.0' ?>") +
      "<sdf version='1.6'>" +
      "<light name='spawned_light' type='directional'>" +
      "</light>" +
      "</sdf>";

  auto lightsStr = std::string("<?xml version='1.0' ?>") +
      "<sdf version='1.6'>" +
      "<light name='accepted_light' type='directional'>" +
      "</light>" +
      "<light name='ignored_light' type='directional'>" +
      "</light>" +
      "</sdf>";

  auto badStr = std::string("<?xml version='1.0' ?>") +
      "<sdf version='1.6'>" +
      "</sdfo>";

  // Request entity spawn
  msgs::EntityFactory req;
  req.set_sdf(modelStr);

  auto pose = req.mutable_pose();
  auto pos = pose->mutable_position();
  pos->set_z(10);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/empty/create"};

  transport::Node node;
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Check entity has not been created yet
  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("spawned_model")));

  // Run an iteration and check it was created
  server.Run(true, 1, false);
  EXPECT_EQ(entityCount + 4, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  auto model = ecm->EntityByComponents(components::Model(),
      components::Name("spawned_model"));
  EXPECT_NE(kNullEntity, model);

  auto poseComp = ecm->Component<components::Pose>(model);
  EXPECT_NE(nullptr, poseComp);

  EXPECT_EQ(math::Pose3d(0, 0, 10, 0, 0, 0), poseComp->Data());

  // Request to spawn same model and check if fails due to repeated name
  req.Clear();
  req.set_sdf(modelStr);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was not created
  server.Run(true, 1, false);

  EXPECT_EQ(entityCount, ecm->EntityCount());

  // Enable renaming and check it is spawned with new name
  req.Clear();
  req.set_sdf(modelStr);
  req.set_allow_renaming(true);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was created with a new name
  server.Run(true, 1, false);

  EXPECT_EQ(entityCount + 4, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  model = ecm->EntityByComponents(components::Model(),
      components::Name("spawned_model_0"));
  EXPECT_NE(kNullEntity, model);

  // Spawn with a different name
  req.Clear();
  req.set_sdf(modelStr);
  req.set_name("banana");

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was created with given name
  server.Run(true, 1, false);

  EXPECT_EQ(entityCount + 4, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  model = ecm->EntityByComponents(components::Model(),
      components::Name("banana"));
  EXPECT_NE(kNullEntity, model);

  // Spawn a light
  req.Clear();
  req.set_sdf(lightStr);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was created
  server.Run(true, 1, false);

  EXPECT_EQ(entityCount + 1, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  auto light = ecm->EntityByComponents(components::Name("spawned_light"));
  EXPECT_NE(kNullEntity, light);

  EXPECT_NE(nullptr, ecm->Component<components::Light>(light));

  // Queue commands and check they're all executed in the same iteration
  req.Clear();
  req.set_sdf(modelStr);
  req.set_name("acerola");

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  req.Clear();
  req.set_sdf(modelStr);
  req.set_name("coconut");

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Check neither exists yet
  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("acerola")));
  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("coconut")));
  EXPECT_EQ(entityCount, ecm->EntityCount());

  // Run an iteration and check both models were created
  server.Run(true, 1, false);

  EXPECT_EQ(entityCount + 8, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("acerola")));
  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("coconut")));

  // Try to spawn 2 entities at once
  req.Clear();
  req.set_sdf(lightsStr);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check only the 1st was created
  server.Run(true, 1, false);

  EXPECT_EQ(entityCount + 1, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(
      components::Name("ignored_light")));

  light = ecm->EntityByComponents(components::Name("accepted_light"));
  EXPECT_NE(kNullEntity, light);

  EXPECT_NE(nullptr, ecm->Component<components::Light>(light));

  // Try to spawn a malformed SDF
  req.Clear();
  req.set_sdf(badStr);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check nothing was created
  server.Run(true, 1, false);

  EXPECT_EQ(entityCount, ecm->EntityCount());

  // Spawn from file
  auto testModel = common::joinPaths(PROJECT_SOURCE_PATH, "test", "media",
      "test_model.sdf");
  req.Clear();
  req.set_sdf_filename(testModel);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was created
  server.Run(true, 1, false);
  EXPECT_EQ(entityCount + 4, ecm->EntityCount());

  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("test_model")));
}

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, Remove)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/shapes.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system just to get the ECM
  // TODO(louise) It would be much more convenient if the Server just returned
  // the ECM for us. This would save all the trouble which is causing us to
  // create `Relay` systems in the first place. Consider keeping the ECM in a
  // shared pointer owned by the SimulationRunner.
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const gazebo::UpdateInfo &,
                             gazebo::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  // Check entities
  // 1 x world + 1 x (default) level + 1 x wind + 3 x model + 3 x link + 3 x
  // collision + 3 x visual + 1 x light
  EXPECT_EQ(16u, ecm->EntityCount());

  // Entity remove by name
  msgs::Entity req;
  req.set_name("box");
  req.set_type(msgs::Entity::MODEL);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/default/remove"};

  transport::Node node;
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Check entity has not been removed yet
  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("box")));

  // Run an iteration and check it was removed
  server.Run(true, 1, false);
  EXPECT_EQ(12u, ecm->EntityCount());

  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("box")));

  // Entity remove by ID
  auto sphereId = ecm->EntityByComponents(components::Model(),
      components::Name("sphere"));
  EXPECT_NE(kNullEntity, sphereId);

  req.Clear();
  req.set_id(sphereId);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Check entity has not been removed yet
  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("sphere")));

  // Run an iteration and check it was removed
  server.Run(true, 1, false);
  EXPECT_EQ(8u, ecm->EntityCount());

  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("sphere")));

  // Can't remove a link
  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Link(),
      components::Name("cylinder_link")));

  req.Clear();
  req.set_name("cylinder_link");
  req.set_type(msgs::Entity::LINK);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was not removed
  server.Run(true, 1, false);
  EXPECT_EQ(8u, ecm->EntityCount());

  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Link(),
      components::Name("cylinder_link")));

  // All fields present - ID is used
  auto cylinderId = ecm->EntityByComponents(components::Model(),
      components::Name("cylinder"));
  EXPECT_NE(kNullEntity, cylinderId);

  req.Clear();
  req.set_id(cylinderId);
  req.set_name("sun");
  req.set_type(msgs::Entity::LIGHT);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check cylinder was removed and light wasn't
  server.Run(true, 1, false);
  EXPECT_EQ(4u, ecm->EntityCount());

  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Model(),
      components::Name("cylinder")));
  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Name("sun")));

  // Only name - fails to remove
  auto lightId = ecm->EntityByComponents(components::Name("sun"));
  EXPECT_NE(kNullEntity, lightId);

  req.Clear();
  req.set_name("sun");

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check nothing was removed
  server.Run(true, 1, false);
  EXPECT_EQ(4u, ecm->EntityCount());

  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Name("sun")));

  // Inexistent entity - fails to remove
  req.Clear();
  req.set_id(9999);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check nothing was removed
  server.Run(true, 1, false);
  EXPECT_EQ(4u, ecm->EntityCount());

  // Unsupported type - fails to remove
  req.Clear();
  req.set_name("sun");
  req.set_type(msgs::Entity::LINK);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check nothing was removed
  server.Run(true, 1, false);
  EXPECT_EQ(4u, ecm->EntityCount());

  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Name("sun")));

  // Remove light
  req.Clear();
  req.set_name("sun");
  req.set_type(msgs::Entity::LIGHT);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was removed
  server.Run(true, 1, false);
  EXPECT_EQ(3u, ecm->EntityCount());

  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Name("sun")));
}

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, Pose)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/shapes.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system just to get the ECM
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const gazebo::UpdateInfo &,
                             gazebo::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  // Entity move by name
  msgs::Pose req;
  req.set_name("box");
  req.mutable_position()->set_y(123.0);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/default/set_pose"};

  transport::Node node;
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Box entity
  auto boxEntity = ecm->EntityByComponents(components::Name("box"));
  EXPECT_NE(kNullEntity, boxEntity);

  // Check entity has not been moved yet
  auto poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_EQ(math::Pose3d(1, 2, 3, 0, 0, 1), poseComp->Data());

  // Run an iteration and check it was moved
  server.Run(true, 1, false);

  poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(123.0, poseComp->Data().Pos().Y(), 0.2);

  // Entity move by ID
  req.Clear();
  req.set_id(boxEntity);
  req.mutable_position()->set_y(321.0);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Check entity has not been moved yet
  poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(123.0, poseComp->Data().Pos().Y(), 0.2);

  // Run an iteration and check it was moved
  server.Run(true, 1, false);

  poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(321.0, poseComp->Data().Pos().Y(), 0.2);

  // Link entity
  auto linkEntity = ecm->EntityByComponents(components::Name("box_link"));
  EXPECT_NE(kNullEntity, linkEntity);

  // Can't move a link
  req.Clear();
  req.set_id(linkEntity);
  req.mutable_position()->set_y(123.0);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was not moved
  server.Run(true, 1, false);

  poseComp = ecm->Component<components::Pose>(linkEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_EQ(math::Pose3d(0.1, 0.1, 0.1, 0, 0, 0), poseComp->Data());

  poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(321.0, poseComp->Data().Pos().Y(), 0.2);

  // All fields present - ID is used
  req.Clear();
  req.set_id(boxEntity);
  req.set_name("sphere");
  req.mutable_position()->set_y(456.0);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check box was moved and sphere wasn't
  server.Run(true, 1, false);

  auto sphereEntity = ecm->EntityByComponents(components::Name("sphere"));
  EXPECT_NE(kNullEntity, sphereEntity);

  poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(456.0, poseComp->Data().Pos().Y(), 0.2);

  poseComp = ecm->Component<components::Pose>(sphereEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(0.0, poseComp->Data().Pos().Y(), 0.2);

  // Inexistent entity - fails to move
  req.Clear();
  req.set_id(9999);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 1, false);

  poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(456.0, poseComp->Data().Pos().Y(), 0.2);

  poseComp = ecm->Component<components::Pose>(sphereEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(0.0, poseComp->Data().Pos().Y(), 0.2);

  // Entities move even when paused
  req.Clear();
  req.set_id(boxEntity);
  req.mutable_position()->set_y(500.0);
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Check entity has not been moved yet
  poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(456.0, poseComp->Data().Pos().Y(), 0.2);

  // Run an iteration while in the paused state and check it was moved
  // Note: server.Run(true, 1, true) does not return so we have to use the async
  // Run function
  server.Run(false, 1, true);

  // Sleep for a small duration to allow Run thread to start
  IGN_SLEEP_MS(10);

  poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(500.0, poseComp->Data().Pos().Y(), 0.2);
}
