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

#include <string>

#include <gtest/gtest.h>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/msgs/material_color.pb.h>
#include <gz/msgs/physics.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/msgs/wheel_slip_parameters_cmd.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Material.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Physics.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/VisualCmd.hh"
#include "gz/sim/components/WheelSlipCmd.hh"
#include "gz/sim/components/World.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;

//////////////////////////////////////////////////
class UserCommandsTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(UserCommandsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Create))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/empty.sdf";
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
  testSystem.OnPreUpdate([&](const UpdateInfo &,
                             EntityComponentManager &_ecm)
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

  EXPECT_EQ(entityCount + 2, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  auto light = ecm->EntityByComponents(components::Name("spawned_light"));
  EXPECT_NE(kNullEntity, light);

  EXPECT_NE(nullptr, ecm->Component<components::Light>(light));

  // Request entity spawn
  req.Clear();
  req.mutable_light()->set_name("light_test");
  req.mutable_light()->set_parent_id(1);
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was created
  server.Run(true, 1, false);

  EXPECT_EQ(entityCount + 2, ecm->EntityCount());
  entityCount = ecm->EntityCount();

  light = ecm->EntityByComponents(components::Name("light_test"));
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

  EXPECT_EQ(entityCount + 2, ecm->EntityCount());
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
TEST_F(UserCommandsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Remove))
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
  testSystem.OnPreUpdate([&](const UpdateInfo &,
                             EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  // Check entities
  // 1 x world + 1 x (default) level + 1 x wind + 5 x model + 5 x link + 5 x
  // collision + 5 x visual + 1 x light (light + visual)
  EXPECT_EQ(25u, ecm->EntityCount());

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
  EXPECT_EQ(21u, ecm->EntityCount());

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
  EXPECT_EQ(17u, ecm->EntityCount());

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
  EXPECT_EQ(17u, ecm->EntityCount());

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
  EXPECT_EQ(13u, ecm->EntityCount());

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
  EXPECT_EQ(13u, ecm->EntityCount());

  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Name("sun")));

  // Inexistent entity - fails to remove
  req.Clear();
  req.set_id(9999);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check nothing was removed
  server.Run(true, 1, false);
  EXPECT_EQ(13u, ecm->EntityCount());

  // Unsupported type - fails to remove
  req.Clear();
  req.set_name("sun");
  req.set_type(msgs::Entity::LINK);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check nothing was removed
  server.Run(true, 1, false);
  EXPECT_EQ(13u, ecm->EntityCount());

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
  EXPECT_EQ(11u, ecm->EntityCount());

  EXPECT_EQ(kNullEntity, ecm->EntityByComponents(components::Name("sun")));
}

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Pose))
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
  testSystem.OnPreUpdate([&](const UpdateInfo &,
                             EntityComponentManager &_ecm)
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
  GZ_SLEEP_MS(10);

  poseComp = ecm->Component<components::Pose>(boxEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(500.0, poseComp->Data().Pos().Y(), 0.2);
}


/////////////////////////////////////////////////
TEST_F(UserCommandsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(PoseVector))
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
  testSystem.OnPreUpdate([&](const UpdateInfo &,
                             EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  // Entity move by name
  msgs::Pose_V req;

  auto poseBoxMsg = req.add_pose();
  poseBoxMsg->set_name("box");
  poseBoxMsg->mutable_position()->set_y(123.0);

  auto poseSphereMsg = req.add_pose();
  poseSphereMsg->set_name("sphere");
  poseSphereMsg->mutable_position()->set_y(456.0);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/default/set_pose_vector"};

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

  auto sphereEntity = ecm->EntityByComponents(components::Name("sphere"));
  EXPECT_NE(kNullEntity, sphereEntity);

  poseComp = ecm->Component<components::Pose>(sphereEntity);
  ASSERT_NE(nullptr, poseComp);
  EXPECT_NEAR(456, poseComp->Data().Pos().Y(), 0.2);
}

/////////////////////////////////////////////////
// https://github.com/gazebosim/gz-sim/issues/634
TEST_F(UserCommandsTest, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(Light))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = gz::common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds", "lights_render.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system just to get the ECM
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const sim::UpdateInfo &,
                             sim::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  msgs::Light req;
  msgs::Boolean res;
  transport::Node node;
  bool result;
  unsigned int timeout = 1000;
  std::string service{"/world/lights_command/light_config"};

  // Point light
  auto pointLightEntity = ecm->EntityByComponents(components::Name("point"));
  EXPECT_NE(kNullEntity, pointLightEntity);

  // Check point light entity has not been edited yet - Initial values
  auto pointLightComp = ecm->Component<components::Light>(pointLightEntity);
  ASSERT_NE(nullptr, pointLightComp);
  EXPECT_EQ(
    math::Pose3d(0, -1.5, 3, 0, 0, 0), pointLightComp->Data().RawPose());
  EXPECT_EQ(math::Color(1.0f, 0.0f, 0.0f, 1.0f),
      pointLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.1f, 0.1f, 0.1f, 1.0f),
      pointLightComp->Data().Specular());
  EXPECT_NEAR(4.0, pointLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(0.5, pointLightComp->Data().LinearAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.2, pointLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.01, pointLightComp->Data().QuadraticAttenuationFactor(), 0.1);
  EXPECT_FALSE(pointLightComp->Data().CastShadows());
  EXPECT_TRUE(pointLightComp->Data().LightOn());
  EXPECT_TRUE(pointLightComp->Data().Visualize());

  req.Clear();
  gz::msgs::Set(req.mutable_diffuse(),
    gz::math::Color(0.0f, 1.0f, 1.0f, 0.0f));
  gz::msgs::Set(req.mutable_specular(),
    gz::math::Color(0.2f, 0.2f, 0.2f, 0.2f));
  req.set_range(2.6f);
  req.set_name("point");
  req.set_type(gz::msgs::Light::POINT);
  req.set_attenuation_linear(0.7f);
  req.set_attenuation_constant(0.6f);
  req.set_attenuation_quadratic(0.001f);
  req.set_cast_shadows(true);
  req.set_is_light_off(false);
  req.set_visualize_visual(false);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  // Check point light entity has been edited using the service
  pointLightComp = ecm->Component<components::Light>(pointLightEntity);
  ASSERT_NE(nullptr, pointLightComp);

  EXPECT_EQ(math::Color(0.0f, 1.0f, 1.0f, 0.0f),
      pointLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.2f, 0.2f, 0.2f, 0.2f),
      pointLightComp->Data().Specular());
  EXPECT_NEAR(2.6, pointLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(0.7, pointLightComp->Data().LinearAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.6, pointLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.001, pointLightComp->Data().QuadraticAttenuationFactor(), 0.1);
  EXPECT_TRUE(pointLightComp->Data().CastShadows());
  EXPECT_TRUE(pointLightComp->Data().LightOn());
  EXPECT_FALSE(pointLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::POINT, pointLightComp->Data().Type());

  // Check directional light entity has not been edited yet - Initial values
  auto directionalLightEntity = ecm->EntityByComponents(
      components::Name("directional"));
  EXPECT_NE(kNullEntity, directionalLightEntity);

  auto directionalLightComp =
    ecm->Component<components::Light>(directionalLightEntity);
  ASSERT_NE(nullptr, directionalLightComp);

  EXPECT_EQ(
    math::Pose3d(0, 0, 10, 0, 0, 0), directionalLightComp->Data().RawPose());
  EXPECT_EQ(math::Color(0.8f, 0.8f, 0.8f, 1.0f),
    directionalLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.2f, 0.2f, 0.2f, 1.0f),
    directionalLightComp->Data().Specular());
  EXPECT_NEAR(100, directionalLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(
    0.01, directionalLightComp->Data().LinearAttenuationFactor(), 0.01);
  EXPECT_NEAR(
    0.9, directionalLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(
    0.001, directionalLightComp->Data().QuadraticAttenuationFactor(), 0.001);
  EXPECT_EQ(
    math::Vector3d(0.5, 0.2, -0.9), directionalLightComp->Data().Direction());
  EXPECT_TRUE(directionalLightComp->Data().CastShadows());
  EXPECT_TRUE(directionalLightComp->Data().LightOn());
  EXPECT_TRUE(directionalLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::POINT, pointLightComp->Data().Type());

  req.Clear();
  gz::msgs::Set(req.mutable_diffuse(),
    gz::math::Color(0.0f, 1.0f, 1.0f, 0.0f));
  gz::msgs::Set(req.mutable_specular(),
    gz::math::Color(0.3f, 0.3f, 0.3f, 0.3f));
  req.set_range(2.6f);
  req.set_name("directional");
  req.set_type(gz::msgs::Light::DIRECTIONAL);
  req.set_attenuation_linear(0.7f);
  req.set_attenuation_constant(0.6f);
  req.set_attenuation_quadratic(1.0f);
  req.set_cast_shadows(false);
  req.set_is_light_off(false);
  req.set_visualize_visual(false);
  gz::msgs::Set(req.mutable_direction(),
    gz::math::Vector3d(1, 2, 3));
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  // Check directional light entity has been edited using the service
  directionalLightComp =
    ecm->Component<components::Light>(directionalLightEntity);
  ASSERT_NE(nullptr, directionalLightComp);

  EXPECT_EQ(math::Color(0.0f, 1.0f, 1.0f, 0.0f),
    directionalLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.3f, 0.3f, 0.3f, 0.3f),
    directionalLightComp->Data().Specular());
  EXPECT_NEAR(2.6, directionalLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(
    0.7, directionalLightComp->Data().LinearAttenuationFactor(), 0.1);
  EXPECT_NEAR(
    0.6, directionalLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(
    1, directionalLightComp->Data().QuadraticAttenuationFactor(), 0.1);
  EXPECT_EQ(math::Vector3d(1, 2, 3), directionalLightComp->Data().Direction());
  EXPECT_FALSE(directionalLightComp->Data().CastShadows());
  EXPECT_TRUE(directionalLightComp->Data().LightOn());
  EXPECT_FALSE(directionalLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::DIRECTIONAL,
    directionalLightComp->Data().Type());

  // spot light
  auto spotLightEntity = ecm->EntityByComponents(
      components::Name("spot"));
  EXPECT_NE(kNullEntity, spotLightEntity);

  // Check spot light entity has not been edited yet - Initial values
  auto spotLightComp =
    ecm->Component<components::Light>(spotLightEntity);
  ASSERT_NE(nullptr, spotLightComp);

  EXPECT_EQ(math::Pose3d(0, 1.5, 3, 0, 0, 0), spotLightComp->Data().RawPose());
  EXPECT_EQ(math::Color(0.0f, 1.0f, 0.0f, 1.0f),
    spotLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.2f, 0.2f, 0.2f, 1.0f),
    spotLightComp->Data().Specular());
  EXPECT_NEAR(5, spotLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(0.4, spotLightComp->Data().LinearAttenuationFactor(), 0.01);
  EXPECT_NEAR(0.3, spotLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(
    0.001, spotLightComp->Data().QuadraticAttenuationFactor(), 0.001);
  EXPECT_EQ(math::Vector3d(0, 0, -1), spotLightComp->Data().Direction());
  EXPECT_FALSE(spotLightComp->Data().CastShadows());
  EXPECT_TRUE(spotLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::SPOT, spotLightComp->Data().Type());
  EXPECT_NEAR(0.1, spotLightComp->Data().SpotInnerAngle().Radian(), 0.1);
  EXPECT_NEAR(0.5, spotLightComp->Data().SpotOuterAngle().Radian(), 0.1);
  EXPECT_NEAR(0.8, spotLightComp->Data().SpotFalloff(), 0.1);

  req.Clear();
  gz::msgs::Set(req.mutable_diffuse(),
    gz::math::Color(1.0f, 0.0f, 1.0f, 0.0f));
  gz::msgs::Set(req.mutable_specular(),
    gz::math::Color(0.3f, 0.3f, 0.3f, 0.3f));
  req.set_range(2.6f);
  req.set_name("spot");
  req.set_type(gz::msgs::Light::SPOT);
  req.set_attenuation_linear(0.7f);
  req.set_attenuation_constant(0.6f);
  req.set_attenuation_quadratic(1.0f);
  req.set_cast_shadows(true);
  req.set_is_light_off(true);
  req.set_visualize_visual(true);
  gz::msgs::Set(req.mutable_direction(),
    gz::math::Vector3d(1, 2, 3));
  req.set_spot_inner_angle(1.5f);
  req.set_spot_outer_angle(0.3f);
  req.set_spot_falloff(0.9f);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  // Check spot light entity has been edited using the service
  spotLightComp = ecm->Component<components::Light>(spotLightEntity);
  ASSERT_NE(nullptr, spotLightComp);

  EXPECT_EQ(math::Color(1.0f, 0.0f, 1.0f, 0.0f),
    spotLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.3f, 0.3f, 0.3f, 0.3f),
    spotLightComp->Data().Specular());
  EXPECT_NEAR(2.6, spotLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(0.7, spotLightComp->Data().LinearAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.6, spotLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(1, spotLightComp->Data().QuadraticAttenuationFactor(), 0.1);
  EXPECT_EQ(math::Vector3d(1, 2, 3), spotLightComp->Data().Direction());
  EXPECT_TRUE(spotLightComp->Data().CastShadows());
  EXPECT_FALSE(spotLightComp->Data().LightOn());
  EXPECT_TRUE(spotLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::SPOT, spotLightComp->Data().Type());
  EXPECT_NEAR(1.5, spotLightComp->Data().SpotInnerAngle().Radian(), 0.1);
  EXPECT_NEAR(0.3, spotLightComp->Data().SpotOuterAngle().Radian(), 0.1);
  EXPECT_NEAR(0.9, spotLightComp->Data().SpotFalloff(), 0.1);

  // Test light_config topic
  const std::string lightTopic = "/world/lights_command/light_config";

  msgs::Light lightMsg;
  gz::msgs::Set(lightMsg.mutable_diffuse(),
    gz::math::Color(1.0f, 1.0f, 1.0f, 1.0f));
  gz::msgs::Set(lightMsg.mutable_pose()->mutable_position(),
    gz::math::Vector3d(1.0f, 0.0f, 0.0f));

  // Publish light config without name
  auto pub = node.Advertise<msgs::Light>(lightTopic);
  pub.Publish(lightMsg);

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  // add name
  lightMsg.set_name("spot");

  // Publish light config
  pub.Publish(lightMsg);

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  EXPECT_EQ(math::Color(1.0f, 1.0f, 1.0f, 1.0f),
    spotLightComp->Data().Diffuse());
}

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(LightAll))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = gz::common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds",
      "lights_render_all.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system just to get the ECM
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const sim::UpdateInfo &,
                             sim::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  msgs::Light req;
  msgs::Boolean res;
  transport::Node node;
  bool result;
  unsigned int timeout = 1000;
  std::string service{"/world/lights_command/light_config"};

  // Point light
  auto pointLightEntity = ecm->EntityByComponents(components::Name("point"));
  EXPECT_NE(kNullEntity, pointLightEntity);

  // Check point light entity has not been edited yet - Initial values
  auto pointLightComp = ecm->Component<components::Light>(pointLightEntity);
  ASSERT_NE(nullptr, pointLightComp);
  EXPECT_EQ(
    math::Pose3d(0, -1.5, 3, 0, 0, 0), pointLightComp->Data().RawPose());
  EXPECT_EQ(math::Color(1.0f, 0.0f, 0.0f, 1.0f),
      pointLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.1f, 0.1f, 0.1f, 1.0f),
      pointLightComp->Data().Specular());
  EXPECT_NEAR(4.0, pointLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(0.5, pointLightComp->Data().LinearAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.2, pointLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.01, pointLightComp->Data().QuadraticAttenuationFactor(), 0.1);
  EXPECT_FALSE(pointLightComp->Data().CastShadows());
  EXPECT_TRUE(pointLightComp->Data().LightOn());
  EXPECT_TRUE(pointLightComp->Data().Visualize());

  req.Clear();
  gz::msgs::Set(req.mutable_diffuse(),
    gz::math::Color(0.0f, 1.0f, 1.0f, 0.0f));
  gz::msgs::Set(req.mutable_specular(),
    gz::math::Color(0.2f, 0.2f, 0.2f, 0.2f));
  req.set_range(2.6f);
  req.set_name("point");
  req.set_type(gz::msgs::Light::POINT);
  req.set_attenuation_linear(0.7f);
  req.set_attenuation_constant(0.6f);
  req.set_attenuation_quadratic(0.001f);
  req.set_cast_shadows(true);
  req.set_is_light_off(false);
  req.set_visualize_visual(false);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  // Check point light entity has been edited using the service
  pointLightComp = ecm->Component<components::Light>(pointLightEntity);
  ASSERT_NE(nullptr, pointLightComp);

  EXPECT_EQ(math::Color(0.0f, 1.0f, 1.0f, 0.0f),
      pointLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.2f, 0.2f, 0.2f, 0.2f),
      pointLightComp->Data().Specular());
  EXPECT_NEAR(2.6, pointLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(0.7, pointLightComp->Data().LinearAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.6, pointLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.001, pointLightComp->Data().QuadraticAttenuationFactor(), 0.1);
  EXPECT_TRUE(pointLightComp->Data().CastShadows());
  EXPECT_TRUE(pointLightComp->Data().LightOn());
  EXPECT_FALSE(pointLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::POINT, pointLightComp->Data().Type());

  // Check directional light entity has not been edited yet - Initial values
  auto directionalLightEntity = ecm->EntityByComponents(
      components::Name("directional"));
  EXPECT_NE(kNullEntity, directionalLightEntity);

  auto directionalLightComp =
    ecm->Component<components::Light>(directionalLightEntity);
  ASSERT_NE(nullptr, directionalLightComp);

  EXPECT_EQ(
    math::Pose3d(0, 0, 10, 0, 0, 0), directionalLightComp->Data().RawPose());
  EXPECT_EQ(math::Color(0.8f, 0.8f, 0.8f, 1.0f),
    directionalLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.2f, 0.2f, 0.2f, 1.0f),
    directionalLightComp->Data().Specular());
  EXPECT_NEAR(100, directionalLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(
    0.01, directionalLightComp->Data().LinearAttenuationFactor(), 0.01);
  EXPECT_NEAR(
    0.9, directionalLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(
    0.001, directionalLightComp->Data().QuadraticAttenuationFactor(), 0.001);
  EXPECT_EQ(
    math::Vector3d(0.5, 0.2, -0.9), directionalLightComp->Data().Direction());
  EXPECT_TRUE(directionalLightComp->Data().CastShadows());
  EXPECT_TRUE(directionalLightComp->Data().LightOn());
  EXPECT_TRUE(directionalLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::POINT, pointLightComp->Data().Type());

  req.Clear();
  gz::msgs::Set(req.mutable_diffuse(),
    gz::math::Color(0.0f, 1.0f, 1.0f, 0.0f));
  gz::msgs::Set(req.mutable_specular(),
    gz::math::Color(0.3f, 0.3f, 0.3f, 0.3f));
  req.set_range(2.6f);
  req.set_name("directional");
  req.set_type(gz::msgs::Light::DIRECTIONAL);
  req.set_attenuation_linear(0.7f);
  req.set_attenuation_constant(0.6f);
  req.set_attenuation_quadratic(1.0f);
  req.set_cast_shadows(false);
  req.set_is_light_off(false);
  req.set_visualize_visual(false);
  gz::msgs::Set(req.mutable_direction(),
    gz::math::Vector3d(1, 2, 3));
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  // Check directional light entity has been edited using the service
  directionalLightComp =
    ecm->Component<components::Light>(directionalLightEntity);
  ASSERT_NE(nullptr, directionalLightComp);

  EXPECT_EQ(math::Color(0.0f, 1.0f, 1.0f, 0.0f),
    directionalLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.3f, 0.3f, 0.3f, 0.3f),
    directionalLightComp->Data().Specular());
  EXPECT_NEAR(2.6, directionalLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(
    0.7, directionalLightComp->Data().LinearAttenuationFactor(), 0.1);
  EXPECT_NEAR(
    0.6, directionalLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(
    1, directionalLightComp->Data().QuadraticAttenuationFactor(), 0.1);
  EXPECT_EQ(math::Vector3d(1, 2, 3), directionalLightComp->Data().Direction());
  EXPECT_FALSE(directionalLightComp->Data().CastShadows());
  EXPECT_TRUE(directionalLightComp->Data().LightOn());
  EXPECT_FALSE(directionalLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::DIRECTIONAL,
    directionalLightComp->Data().Type());

  // spot light
  auto spotLightEntity = ecm->EntityByComponents(
      components::Name("spot"));
  EXPECT_NE(kNullEntity, spotLightEntity);

  // Check spot light entity has not been edited yet - Initial values
  auto spotLightComp =
    ecm->Component<components::Light>(spotLightEntity);
  ASSERT_NE(nullptr, spotLightComp);

  EXPECT_EQ(math::Pose3d(0, 1.5, 3, 0, 0, 0), spotLightComp->Data().RawPose());
  EXPECT_EQ(math::Color(0.0f, 1.0f, 0.0f, 1.0f),
    spotLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.2f, 0.2f, 0.2f, 1.0f),
    spotLightComp->Data().Specular());
  EXPECT_NEAR(5, spotLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(0.4, spotLightComp->Data().LinearAttenuationFactor(), 0.01);
  EXPECT_NEAR(0.3, spotLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(
    0.001, spotLightComp->Data().QuadraticAttenuationFactor(), 0.001);
  EXPECT_EQ(math::Vector3d(0, 0, -1), spotLightComp->Data().Direction());
  EXPECT_FALSE(spotLightComp->Data().CastShadows());
  EXPECT_TRUE(spotLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::SPOT, spotLightComp->Data().Type());
  EXPECT_NEAR(0.1, spotLightComp->Data().SpotInnerAngle().Radian(), 0.1);
  EXPECT_NEAR(0.5, spotLightComp->Data().SpotOuterAngle().Radian(), 0.1);
  EXPECT_NEAR(0.8, spotLightComp->Data().SpotFalloff(), 0.1);

  req.Clear();
  gz::msgs::Set(req.mutable_diffuse(),
    gz::math::Color(1.0f, 0.0f, 1.0f, 0.0f));
  gz::msgs::Set(req.mutable_specular(),
    gz::math::Color(0.3f, 0.3f, 0.3f, 0.3f));
  req.set_range(2.6f);
  req.set_name("spot");
  req.set_type(gz::msgs::Light::SPOT);
  req.set_attenuation_linear(0.7f);
  req.set_attenuation_constant(0.6f);
  req.set_attenuation_quadratic(1.0f);
  req.set_cast_shadows(true);
  req.set_is_light_off(true);
  req.set_visualize_visual(true);
  gz::msgs::Set(req.mutable_direction(),
    gz::math::Vector3d(1, 2, 3));
  req.set_spot_inner_angle(1.5f);
  req.set_spot_outer_angle(0.3f);
  req.set_spot_falloff(0.9f);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  // Check spot light entity has been edited using the service
  spotLightComp = ecm->Component<components::Light>(spotLightEntity);
  ASSERT_NE(nullptr, spotLightComp);

  EXPECT_EQ(math::Color(1.0f, 0.0f, 1.0f, 0.0f),
    spotLightComp->Data().Diffuse());
  EXPECT_EQ(math::Color(0.3f, 0.3f, 0.3f, 0.3f),
    spotLightComp->Data().Specular());
  EXPECT_NEAR(2.6, spotLightComp->Data().AttenuationRange(), 0.1);
  EXPECT_NEAR(0.7, spotLightComp->Data().LinearAttenuationFactor(), 0.1);
  EXPECT_NEAR(0.6, spotLightComp->Data().ConstantAttenuationFactor(), 0.1);
  EXPECT_NEAR(1, spotLightComp->Data().QuadraticAttenuationFactor(), 0.1);
  EXPECT_EQ(math::Vector3d(1, 2, 3), spotLightComp->Data().Direction());
  EXPECT_TRUE(spotLightComp->Data().CastShadows());
  EXPECT_FALSE(spotLightComp->Data().LightOn());
  EXPECT_TRUE(spotLightComp->Data().Visualize());
  EXPECT_EQ(sdf::LightType::SPOT, spotLightComp->Data().Type());
  EXPECT_NEAR(1.5, spotLightComp->Data().SpotInnerAngle().Radian(), 0.1);
  EXPECT_NEAR(0.3, spotLightComp->Data().SpotOuterAngle().Radian(), 0.1);
  EXPECT_NEAR(0.9, spotLightComp->Data().SpotFalloff(), 0.1);

  // sphere
  auto sphereLightEntity =
    ecm->EntityByComponents(components::Name("sphere_light"));
  ASSERT_NE(kNullEntity, sphereLightEntity);

  // check sphere light initial values
  auto sphereLightComp =
    ecm->Component<components::Light>(sphereLightEntity);
  ASSERT_NE(nullptr, sphereLightComp);
  EXPECT_EQ(math::Color(1.0f, 1.0f, 1.0f, 1.0f),
            sphereLightComp->Data().Diffuse());

  // Test light_config topic
  const std::string lightTopic = "/world/lights_command/light_config";

  msgs::Light lightMsg;
  gz::msgs::Set(lightMsg.mutable_diffuse(),
    gz::math::Color(1.0f, 0.0f, 0.0f, 1.0f));
  gz::msgs::Set(lightMsg.mutable_pose()->mutable_position(),
    gz::math::Vector3d(1.0f, 0.0f, 0.0f));

  // Publish light config without name
  auto pub = node.Advertise<msgs::Light>(lightTopic);
  pub.Publish(lightMsg);

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  // add name
  lightMsg.set_name("sphere_light");

  // Publish light config
  pub.Publish(lightMsg);

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(10);

  Entity sphereEntity0 =
    ecm->EntityByComponents(components::Name("sphere_0"));
  Entity sphereEntity1 =
    ecm->EntityByComponents(components::Name("sphere_1"));
  auto sphereLinkEntity0 =
    ecm->ChildrenByComponents(sphereEntity0,
      components::Name("sphere_link_0"))[0];
  auto sphereLinkEntity1 =
    ecm->ChildrenByComponents(sphereEntity1,
      components::Name("sphere_link_1"))[0];
  auto sphereLightEntity0 =
    ecm->ChildrenByComponents(sphereLinkEntity0,
      components::Name("sphere_light"))[0];
  auto sphereLightEntity1 =
    ecm->ChildrenByComponents(sphereLinkEntity1,
      components::Name("sphere_light"))[0];
  auto updatedLight0 =
    ecm->Component<components::Light>(sphereLightEntity0);
  auto updatedLight1 =
    ecm->Component<components::Light>(sphereLightEntity1);

  EXPECT_EQ(math::Color(1.0f, 0.0f, 0.0f, 1.0f),
            updatedLight0->Data().Diffuse());
  EXPECT_EQ(math::Color(1.0f, 0.0f, 0.0f, 1.0f),
            updatedLight1->Data().Diffuse());
}

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, GZ_UTILS_TEST_ENABLED_ONLY_ON_LINUX(MaterialColor))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = gz::common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds", "material_color.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system just to get the ECM
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const sim::UpdateInfo &,
                             sim::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  transport::Node node;

  // box
  auto sphereVisualEntity =
    ecm->EntityByComponents(components::Name("sphere_visual"));
  ASSERT_NE(kNullEntity, sphereVisualEntity);

  // check box visual's initial values
  auto sphereVisualComp =
    ecm->Component<components::Material>(sphereVisualEntity);
  ASSERT_NE(nullptr, sphereVisualComp);
  EXPECT_EQ(math::Color(0.3f, 0.3f, 0.3f, 1.0f),
            sphereVisualComp->Data().Diffuse());

  // Test material_color topic
  const std::string materialColorTopic =
    "/world/material_color/material_color";

  // Test first return logic (no direct compare as returns unordered set)
  msgs::MaterialColor materialColorMsgFirst;
  materialColorMsgFirst.mutable_entity()->set_name("sphere_visual");
  materialColorMsgFirst.set_entity_match(
    gz::msgs::MaterialColor::EntityMatch::MaterialColor_EntityMatch_FIRST);
  gz::msgs::Set(materialColorMsgFirst.mutable_diffuse(),
    gz::math::Color(0.0f, 0.0f, 0.0f, 1.0f));

  // Publish material color
  auto pub = node.Advertise<msgs::MaterialColor>(materialColorTopic);
  pub.Publish(materialColorMsgFirst);
  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(100);

  msgs::MaterialColor materialColorMsg;
  materialColorMsg.mutable_entity()->set_name("sphere_visual");
  materialColorMsg.set_entity_match(
    gz::msgs::MaterialColor::EntityMatch::MaterialColor_EntityMatch_ALL);
  gz::msgs::Set(materialColorMsg.mutable_diffuse(),
    gz::math::Color(1.0f, 1.0f, 1.0f, 1.0f));

  Entity sphereEntity0 =
    ecm->EntityByComponents(components::Name("sphere_0"));
  Entity sphereEntity1 =
    ecm->EntityByComponents(components::Name("sphere_1"));
  auto sphereLinkEntity0 =
    ecm->ChildrenByComponents(sphereEntity0,
      components::Name("sphere_link_0"))[0];
  auto sphereLinkEntity1 =
    ecm->ChildrenByComponents(sphereEntity1,
      components::Name("sphere_link_1"))[0];
  auto sphereVisualEntity0 =
    ecm->ChildrenByComponents(sphereLinkEntity0,
      components::Name("sphere_visual"))[0];
  auto sphereVisualEntity1 =
    ecm->ChildrenByComponents(sphereLinkEntity1,
      components::Name("sphere_visual"))[0];
  auto updatedVisual0 =
    ecm->Component<components::Material>(sphereVisualEntity0);
  auto updatedVisual1 =
    ecm->Component<components::Material>(sphereVisualEntity1);
  EXPECT_TRUE((math::Color(0.0f, 0.0f, 0.0f, 1.0f) ==
              updatedVisual0->Data().Diffuse()) ||
              (math::Color(0.0f, 0.0f, 0.0f, 1.0f) ==
              updatedVisual1->Data().Diffuse()));

  // Publish material color
  pub.Publish(materialColorMsg);
  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  GZ_SLEEP_MS(100);

  EXPECT_EQ(math::Color(1.0f, 1.0f, 1.0f, 1.0f),
            updatedVisual0->Data().Diffuse());
  EXPECT_EQ(math::Color(1.0f, 1.0f, 1.0f, 1.0f),
            updatedVisual1->Data().Diffuse());
}

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Physics))
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
  testSystem.OnPreUpdate([&](const UpdateInfo &,
                             EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  EXPECT_NE(nullptr, ecm);

  // Check that the physics properties are the ones specified in the sdf
  auto worldEntity = ecm->EntityByComponents(components::World());
  EXPECT_NE(kNullEntity, worldEntity);
  auto physicsComp = ecm->Component<components::Physics>(worldEntity);
  ASSERT_NE(nullptr, physicsComp);
  EXPECT_DOUBLE_EQ(0.001, physicsComp->Data().MaxStepSize());
  EXPECT_DOUBLE_EQ(0.0, physicsComp->Data().RealTimeFactor());

  // Set physics properties
  msgs::Physics req;
  req.set_max_step_size(0.123);
  req.set_real_time_factor(4.567);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/default/set_physics"};

  transport::Node node;
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run two iterations, in the first one the PhysicsCmd component is created
  // in the second one it is processed
  server.Run(true, 2, false);

  // Check updated physics properties
  physicsComp = ecm->Component<components::Physics>(worldEntity);
  EXPECT_DOUBLE_EQ(0.123, physicsComp->Data().MaxStepSize());
  EXPECT_DOUBLE_EQ(4.567, physicsComp->Data().RealTimeFactor());

  // Send invalid values (not > 0) and make sure they are not updated
  req.set_max_step_size(0.0);
  req.set_real_time_factor(0.0);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run two iterations, in the first one the PhysicsCmd component is created
  // in the second one it is processed
  server.Run(true, 2, false);

  // Check updated physics properties
  physicsComp = ecm->Component<components::Physics>(worldEntity);
  EXPECT_DOUBLE_EQ(0.123, physicsComp->Data().MaxStepSize());
  EXPECT_DOUBLE_EQ(4.567, physicsComp->Data().RealTimeFactor());
}

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(WheelSlip))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/trisphere_cycle_wheel_slip.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system just to get the ECM
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const sim::UpdateInfo &,
                             sim::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  ASSERT_NE(nullptr, ecm);

  // Check that the physics properties are the ones specified in the sdf
  auto worldEntity = ecm->EntityByComponents(components::World());
  ASSERT_NE(kNullEntity, worldEntity);
  Entity tc0 = ecm->EntityByComponents(
    components::Name("trisphere_cycle0"));
  ASSERT_NE(kNullEntity, tc0);
  Entity tc1 = ecm->EntityByComponents(
    components::Name("trisphere_cycle1"));
  ASSERT_NE(kNullEntity, tc1);

  Model tcModel0{tc0};
  Model tcModel1{tc1};
  Entity wf0 = tcModel0.LinkByName(*ecm, "wheel_front");
  ASSERT_NE(kNullEntity, wf0);
  Entity wrl0 = tcModel0.LinkByName(*ecm, "wheel_rear_left");
  ASSERT_NE(kNullEntity, wrl0);
  Entity wrf0 = tcModel0.LinkByName(*ecm, "wheel_rear_right");
  ASSERT_NE(kNullEntity, wrf0);
  Entity wf1 = tcModel1.LinkByName(*ecm, "wheel_front");
  ASSERT_NE(kNullEntity, wf1);
  Entity wrl1 = tcModel1.LinkByName(*ecm, "wheel_rear_left");
  ASSERT_NE(kNullEntity, wrl1);
  Entity wrf1 = tcModel1.LinkByName(*ecm, "wheel_rear_right");
  ASSERT_NE(kNullEntity, wrf1);

  Entity links[] = {wf0, wrl0, wrf0, wf1, wrl1, wrf1};
  for (auto link : links) {
    EXPECT_EQ(nullptr, ecm->Component<components::WheelSlipCmd>(link));
  }

  // modify wheel slip parameters of one link of model 0
  msgs::WheelSlipParametersCmd req;
  auto * entityMsg = req.mutable_entity();
  entityMsg->set_name("trisphere_cycle0::wheel_front");
  entityMsg->set_type(msgs::Entity::LINK);
  req.set_slip_compliance_lateral(1);
  req.set_slip_compliance_longitudinal(1);

  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/wheel_slip/wheel_slip"};

  transport::Node node;
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run two iterations, in the first one the WheelSlipCmd component is created
  // and processed.
  // The second one is just to check everything went fine.
  server.Run(true, 2, false);

  // modify wheel slip parameters of one link of model 1
  entityMsg->set_name("trisphere_cycle1");
  entityMsg->set_type(msgs::Entity::MODEL);
  req.set_slip_compliance_lateral(2);
  req.set_slip_compliance_longitudinal(1);

  result = false;
  res = msgs::Boolean{};

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run two iterations, in the first one the WheelSlipCmd component is created
  // and processed.
  // The second one is just to check everything went fine.
  server.Run(true, 3, false);
}

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(Visual))
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds", "shapes.sdf");
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system just to get the ECM
  EntityComponentManager *ecm{nullptr};
  test::Relay testSystem;
  testSystem.OnPreUpdate([&](const sim::UpdateInfo &,
                             sim::EntityComponentManager &_ecm)
      {
        ecm = &_ecm;
      });

  server.AddSystem(testSystem.systemPtr);

  // Run server and check we have the ECM
  EXPECT_EQ(nullptr, ecm);
  server.Run(true, 1, false);
  ASSERT_NE(nullptr, ecm);

  msgs::Visual req;
  msgs::Boolean res;
  transport::Node node;
  bool result;
  unsigned int timeout = 100;
  std::string service{"/world/default/visual_config"};

  auto boxVisualEntity =
    ecm->EntityByComponents(components::Name("box_visual"));
  ASSERT_NE(kNullEntity, boxVisualEntity);

  // check box visual's initial values
  auto boxVisualComp = ecm->Component<components::Material>(boxVisualEntity);
  ASSERT_NE(nullptr, boxVisualComp);
  EXPECT_EQ(math::Color(1.0f, 0.0f, 0.0f, 1.0f),
            boxVisualComp->Data().Diffuse());

  msgs::Set(req.mutable_material()->mutable_diffuse(),
            math::Color(0.0f, 1.0f, 0.0f, 1.0f));

  // This will fail to find the entity in VisualCommand::Execute()
  // since no id was provided
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  server.Run(true, 1, false);
  // check that the VisualCmd component was not created
  auto boxVisCmdComp = ecm->Component<components::VisualCmd>(boxVisualEntity);
  EXPECT_EQ(nullptr, boxVisCmdComp);

  // add id to msg and resend request
  req.set_id(boxVisualEntity);
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  server.Run(true, 1, false);
  // check the VisualCmd was created and check the values
  boxVisCmdComp = ecm->Component<components::VisualCmd>(boxVisualEntity);
  ASSERT_NE(nullptr, boxVisualComp);
  EXPECT_FLOAT_EQ(0.0f, boxVisCmdComp->Data().material().diffuse().r());
  EXPECT_FLOAT_EQ(1.0f, boxVisCmdComp->Data().material().diffuse().g());
  EXPECT_FLOAT_EQ(0.0f, boxVisCmdComp->Data().material().diffuse().b());
  EXPECT_FLOAT_EQ(1.0f, boxVisCmdComp->Data().material().diffuse().a());

  // update component using visual name and parent name
  req.Clear();
  req.set_name("box_visual");
  req.set_parent_name("box_link");
  msgs::Set(req.mutable_material()->mutable_diffuse(),
            math::Color(0.0f, 0.0f, 1.0f, 1.0f));
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  server.Run(true, 1, false);
  // check the values
  boxVisCmdComp = ecm->Component<components::VisualCmd>(boxVisualEntity);
  ASSERT_NE(nullptr, boxVisualComp);
  EXPECT_FLOAT_EQ(0.0f, boxVisCmdComp->Data().material().diffuse().r());
  EXPECT_FLOAT_EQ(0.0f, boxVisCmdComp->Data().material().diffuse().g());
  EXPECT_FLOAT_EQ(1.0f, boxVisCmdComp->Data().material().diffuse().b());
  EXPECT_FLOAT_EQ(1.0f, boxVisCmdComp->Data().material().diffuse().a());
}
