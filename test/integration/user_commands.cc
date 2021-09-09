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
#include <ignition/msgs/light.pb.h>
#include <ignition/msgs/physics.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Link.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Physics.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/World.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

//////////////////////////////////////////////////
class UserCommandsTest : public InternalFixture<::testing::Test>
{
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

  // Request entity spawn
  req.Clear();
  req.mutable_light()->set_name("light_test");
  req.mutable_light()->set_parent_id(1);
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check it was created
  server.Run(true, 1, false);

  EXPECT_EQ(entityCount + 1, ecm->EntityCount());
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
  // 1 x world + 1 x (default) level + 1 x wind + 5 x model + 5 x link + 5 x
  // collision + 5 x visual + 1 x light
  EXPECT_EQ(24u, ecm->EntityCount());

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
  EXPECT_EQ(20u, ecm->EntityCount());

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
  EXPECT_EQ(16u, ecm->EntityCount());

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
  EXPECT_EQ(16u, ecm->EntityCount());

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
  EXPECT_EQ(12u, ecm->EntityCount());

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
  EXPECT_EQ(12u, ecm->EntityCount());

  EXPECT_NE(kNullEntity, ecm->EntityByComponents(components::Name("sun")));

  // Inexistent entity - fails to remove
  req.Clear();
  req.set_id(9999);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check nothing was removed
  server.Run(true, 1, false);
  EXPECT_EQ(12u, ecm->EntityCount());

  // Unsupported type - fails to remove
  req.Clear();
  req.set_name("sun");
  req.set_type(msgs::Entity::LINK);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  // Run an iteration and check nothing was removed
  server.Run(true, 1, false);
  EXPECT_EQ(12u, ecm->EntityCount());

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

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, Light)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = ignition::common::joinPaths(
    std::string(PROJECT_SOURCE_PATH), "test", "worlds", "lights_render.sdf");
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

  req.Clear();
  ignition::msgs::Set(req.mutable_diffuse(),
    ignition::math::Color(0.0f, 1.0f, 1.0f, 0.0f));
  ignition::msgs::Set(req.mutable_specular(),
    ignition::math::Color(0.2f, 0.2f, 0.2f, 0.2f));
  req.set_range(2.6f);
  req.set_name("point");
  req.set_type(ignition::msgs::Light::POINT);
  req.set_attenuation_linear(0.7f);
  req.set_attenuation_constant(0.6f);
  req.set_attenuation_quadratic(0.001f);
  req.set_cast_shadows(true);
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  IGN_SLEEP_MS(10);

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
  EXPECT_EQ(sdf::LightType::POINT, pointLightComp->Data().Type());

  req.Clear();
  ignition::msgs::Set(req.mutable_diffuse(),
    ignition::math::Color(0.0f, 1.0f, 1.0f, 0.0f));
  ignition::msgs::Set(req.mutable_specular(),
    ignition::math::Color(0.3f, 0.3f, 0.3f, 0.3f));
  req.set_range(2.6f);
  req.set_name("directional");
  req.set_type(ignition::msgs::Light::DIRECTIONAL);
  req.set_attenuation_linear(0.7f);
  req.set_attenuation_constant(0.6f);
  req.set_attenuation_quadratic(1.0f);
  req.set_cast_shadows(false);
  ignition::msgs::Set(req.mutable_direction(),
    ignition::math::Vector3d(1, 2, 3));
  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  IGN_SLEEP_MS(10);

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
  EXPECT_EQ(sdf::LightType::SPOT, spotLightComp->Data().Type());
  EXPECT_NEAR(0.1, spotLightComp->Data().SpotInnerAngle().Radian(), 0.1);
  EXPECT_NEAR(0.5, spotLightComp->Data().SpotOuterAngle().Radian(), 0.1);
  EXPECT_NEAR(0.8, spotLightComp->Data().SpotFalloff(), 0.1);

  req.Clear();
  ignition::msgs::Set(req.mutable_diffuse(),
    ignition::math::Color(1.0f, 0.0f, 1.0f, 0.0f));
  ignition::msgs::Set(req.mutable_specular(),
    ignition::math::Color(0.3f, 0.3f, 0.3f, 0.3f));
  req.set_range(2.6f);
  req.set_name("spot");
  req.set_type(ignition::msgs::Light::SPOT);
  req.set_attenuation_linear(0.7f);
  req.set_attenuation_constant(0.6f);
  req.set_attenuation_quadratic(1.0f);
  req.set_cast_shadows(true);
  ignition::msgs::Set(req.mutable_direction(),
    ignition::math::Vector3d(1, 2, 3));
  req.set_spot_inner_angle(1.5f);
  req.set_spot_outer_angle(0.3f);
  req.set_spot_falloff(0.9f);

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());

  server.Run(true, 100, false);
  // Sleep for a small duration to allow Run thread to start
  IGN_SLEEP_MS(10);

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
  EXPECT_EQ(sdf::LightType::SPOT, spotLightComp->Data().Type());
  EXPECT_NEAR(1.5, spotLightComp->Data().SpotInnerAngle().Radian(), 0.1);
  EXPECT_NEAR(0.3, spotLightComp->Data().SpotOuterAngle().Radian(), 0.1);
  EXPECT_NEAR(0.9, spotLightComp->Data().SpotFalloff(), 0.1);
}

/////////////////////////////////////////////////
TEST_F(UserCommandsTest, Physics)
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

  // Check that the physics properties are the ones specified in the sdf
  auto worldEntity = ecm->EntityByComponents(components::World());
  EXPECT_NE(kNullEntity, worldEntity);
  auto physicsComp = ecm->Component<components::Physics>(worldEntity);
  ASSERT_NE(nullptr, physicsComp);
  EXPECT_DOUBLE_EQ(0.001, physicsComp->Data().MaxStepSize());
  EXPECT_DOUBLE_EQ(1.0, physicsComp->Data().RealTimeFactor());

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
