/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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
#include <ignition/msgs/sdf_generator_config.pb.h>
#include <sstream>
#include <tinyxml2.h>

#include <sdf/Collision.hh>
#include <sdf/Model.hh>
#include <sdf/Link.hh>
#include <sdf/Root.hh>
#include <sdf/Visual.hh>
#include <sdf/World.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>
#include <ignition/utilities/ExtraTestMacros.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "helpers/UniqueTestDirectoryEnv.hh"
#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
class SdfGeneratorFixture : public InternalFixture<::testing::Test>
{
  public: void LoadWorld(const std::string &_path)
  {
    ServerConfig serverConfig;
    serverConfig.SetResourceCache(test::UniqueTestDirectoryEnv::Path());
    serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH, _path));

    std::cout << "Loading: " << serverConfig.SdfFile() << std::endl;
    this->server = std::make_unique<Server>(serverConfig);
    EXPECT_FALSE(server->Running());
  }
  public: std::string RequestGeneratedSdf(const std::string &_worldName,
              const msgs::SdfGeneratorConfig &_req = msgs::SdfGeneratorConfig())
  {
    transport::Node node;

    msgs::StringMsg worldGenSdfRes;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/" + _worldName + "/generate_world_sdf"};
    EXPECT_TRUE(node.Request(service, _req, timeout, worldGenSdfRes, result));
    EXPECT_TRUE(result);
    return worldGenSdfRes.data();
  }

  public: std::unique_ptr<Server> server;
};

/////////////////////////////////////////////////
TEST_F(SdfGeneratorFixture, WorldWithModelsSpawnedAfterLoad)
{
  this->LoadWorld("test/worlds/save_world.sdf");

  EXPECT_NE(kNullEntity, this->server->EntityByName("inlineM1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("backpack1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("backpack2"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("backpack3"));
  EXPECT_FALSE(this->server->EntityByName("test_ground_plane").has_value());
  EXPECT_FALSE(this->server->EntityByName("spawned_model").has_value());

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

  // This has to be different from the backpack in order to test SDFormat
  // generation for a Fuel URI that was not known when simulation started.
  const std::string groundPlaneUri =
      "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/ground plane";

  transport::Node node;
  {
    // Spawn from Fuel
    msgs::EntityFactory req;
    req.set_sdf_filename(groundPlaneUri);
    req.set_name("test_ground_plane");

    msgs::Boolean res;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/save_world/create"};

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());

    req.Clear();
    // Spawn from SDF string
    req.set_sdf(modelStr);
    req.mutable_pose()->mutable_position()->set_x(10);

    EXPECT_TRUE(node.Request(service, req, timeout, res, result));
    EXPECT_TRUE(result);
    EXPECT_TRUE(res.data());
  }

  // Run an iteration and check it was created
  server->Run(true, 1, false);
  EXPECT_TRUE(this->server->EntityByName("test_ground_plane").has_value());
  EXPECT_NE(kNullEntity, this->server->EntityByName("test_ground_plane"));
  EXPECT_TRUE(this->server->EntityByName("spawned_model").has_value());
  EXPECT_NE(kNullEntity, this->server->EntityByName("spawned_model"));

  const std::string worldGenSdfRes = this->RequestGeneratedSdf("save_world");
  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(6u, world->ModelCount());

  EXPECT_TRUE(world->ModelNameExists("inlineM1"));
  EXPECT_TRUE(world->ModelNameExists("backpack1"));
  EXPECT_TRUE(world->ModelNameExists("backpack2"));
  EXPECT_TRUE(world->ModelNameExists("backpack3"));
  EXPECT_TRUE(world->ModelNameExists("test_ground_plane"));
  EXPECT_TRUE(world->ModelNameExists("spawned_model"));

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(worldGenSdfRes.c_str());
  ASSERT_NE(nullptr, genSdfDoc.RootElement());
  auto genWorld = genSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, genWorld);

  // Check that the Fuel model spawned after load uses an include tag and has
  // the correct URI
  std::size_t includeCount = 0;
  std::size_t groundPlaneCount = 0;

  for (auto genInclude = genWorld->FirstChildElement("include"); genInclude;
       genInclude = genInclude->NextSiblingElement("include"), ++includeCount)
  {
    auto name = genInclude->FirstChildElement("name");
    ASSERT_NE(nullptr, name);

    if (std::strcmp(name->GetText(), "test_ground_plane") == 0)
    {
      auto genUri = genInclude->FirstChildElement("uri");
      ASSERT_NE(nullptr, genUri);
      EXPECT_STREQ(groundPlaneUri.c_str(), genUri->GetText());
      ++groundPlaneCount;
    }
  }

  EXPECT_EQ(4u, includeCount);
  EXPECT_EQ(1u, groundPlaneCount);

  // Check that spawned_model is included in the generated world as an expanded
  // model
  std::size_t modelCount = 0;
  std::size_t spawnedModelCount = 0;
  for (auto genModel = genWorld->FirstChildElement("model"); genModel;
       genModel = genModel->NextSiblingElement("model"), ++modelCount)
  {
    auto name = genModel->Attribute("name");
    ASSERT_NE(nullptr, name);

    if (std::strcmp(name, "spawned_model") == 0)
    {
      ++spawnedModelCount;
    }
  }

  EXPECT_EQ(2u, modelCount);
  EXPECT_EQ(1u, spawnedModelCount);
}

/////////////////////////////////////////////////
// Test segfaults on Mac at startup, possible collision with test above?
TEST_F(SdfGeneratorFixture,
    IGN_UTILS_TEST_DISABLED_ON_MAC(ModelSpawnedWithNewName))
{
  this->LoadWorld("test/worlds/save_world.sdf");

  auto modelStr = R"(
<?xml version="1.0" ?>
<sdf version='1.6'>
  <model name='spawned_model'>
    <link name='link'/>
  </model>
</sdf>)";

  transport::Node node;
  msgs::EntityFactory req;
  msgs::Boolean res;
  bool result;
  unsigned int timeout = 5000;
  std::string service{"/world/save_world/create"};

  req.set_sdf(modelStr);
  req.set_name("new_model_name");

  EXPECT_TRUE(node.Request(service, req, timeout, res, result));
  EXPECT_TRUE(result);
  EXPECT_TRUE(res.data());
  // Run an iteration and check it was created
  server->Run(true, 1, false);
  EXPECT_TRUE(this->server->EntityByName("new_model_name").has_value());
  EXPECT_NE(kNullEntity, this->server->EntityByName("new_model_name"));

  const std::string worldGenSdfRes = this->RequestGeneratedSdf("save_world");
  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_TRUE(world->ModelNameExists("new_model_name"));
}

/////////////////////////////////////////////////
TEST_F(SdfGeneratorFixture, WorldWithNestedModel)
{
  this->LoadWorld("test/worlds/nested_model.sdf");

  EXPECT_NE(kNullEntity, this->server->EntityByName("model_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("link_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("collision_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("visual_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("model_01"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("link_01"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("collision_01"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("visual_01"));

  const std::string worldGenSdfRes =
      this->RequestGeneratedSdf("nested_model_world");

  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_EQ(1u, world->ModelCount());

  EXPECT_TRUE(world->ModelNameExists("model_00"));
  EXPECT_FALSE(world->ModelNameExists("model_01"));

  auto *model00 = world->ModelByName("model_00");
  ASSERT_NE(nullptr, model00);
  EXPECT_EQ(1u, model00->LinkCount());
  EXPECT_EQ(1u, model00->ModelCount());

  auto *link00 = model00->LinkByName("link_00");
  ASSERT_NE(nullptr, link00);
  EXPECT_EQ(1u, link00->CollisionCount());
  EXPECT_NE(nullptr, link00->CollisionByName("collision_00"));
  EXPECT_EQ(1u, link00->VisualCount());
  EXPECT_NE(nullptr, link00->VisualByName("visual_00"));

  auto *model01 = model00->ModelByName("model_01");
  ASSERT_NE(nullptr, model01);
  EXPECT_EQ(1u, model01->LinkCount());

  auto *link01 = model01->LinkByName("link_01");
  ASSERT_NE(nullptr, link01);
  EXPECT_EQ(1u, link01->CollisionCount());
  EXPECT_NE(nullptr, link01->CollisionByName("collision_01"));
  EXPECT_EQ(1u, link01->VisualCount());
  EXPECT_NE(nullptr, link01->VisualByName("visual_01"));

  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(worldGenSdfRes.c_str());
  ASSERT_NE(nullptr, genSdfDoc.RootElement());
  auto genWorld = genSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, genWorld);
}


/////////////////////////////////////////////////
TEST_F(SdfGeneratorFixture, ModelWithNestedIncludes)
{
  std::string path =
      common::joinPaths(PROJECT_SOURCE_PATH, "test", "worlds", "models");
  common::setenv("IGN_GAZEBO_RESOURCE_PATH", path);

  this->LoadWorld("test/worlds/model_nested_include.sdf");

  EXPECT_NE(kNullEntity, this->server->EntityByName("ground_plane"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("L0"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("C0"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("V0"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("M1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("M2"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("M3"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("coke"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("L1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("C1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("V1"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("include_nested_new_name"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("link_00"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("link_01"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("sphere"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("V"));
  EXPECT_NE(kNullEntity, this->server->EntityByName("C"));

  msgs::SdfGeneratorConfig req;
  req.mutable_global_entity_gen_config()
     ->mutable_save_fuel_version()->set_data(true);

  const std::string worldGenSdfRes =
      this->RequestGeneratedSdf("model_nested_include_world", req);

  // check that model w/ nested includes are not expanded
  tinyxml2::XMLDocument genSdfDoc;
  genSdfDoc.Parse(worldGenSdfRes.c_str());
  ASSERT_NE(nullptr, genSdfDoc.RootElement());
  auto genWorld = genSdfDoc.RootElement()->FirstChildElement("world");
  ASSERT_NE(nullptr, genWorld);

  auto model = genWorld->FirstChildElement("model");  // ground_plane
  ASSERT_NE(nullptr, model);
  model = model->NextSiblingElement("model");  // M1
  ASSERT_NE(nullptr, model);

  // M1's child include
  auto include = model->FirstChildElement("include");
  ASSERT_NE(nullptr, include);

  auto uri = include->FirstChildElement("uri");
  ASSERT_NE(nullptr, uri);
  ASSERT_NE(nullptr, uri->GetText());
  EXPECT_EQ("include_nested", std::string(uri->GetText()));

  auto name = include->FirstChildElement("name");
  ASSERT_NE(nullptr, name);
  ASSERT_NE(nullptr, name->GetText());
  EXPECT_EQ("include_nested_new_name", std::string(name->GetText()));

  auto pose = include->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
  ASSERT_NE(nullptr, pose->GetText());

  std::stringstream ss(pose->GetText());
  ignition::math::Pose3d p;
  ss >> p;
  EXPECT_EQ(ignition::math::Pose3d(1, 2, 3, 0, 0, 0), p);

  // M2
  model = model->FirstChildElement("model");
  ASSERT_NE(nullptr, model);

  // M2's child include
  include = model->FirstChildElement("include");
  ASSERT_NE(nullptr, include);

  uri = include->FirstChildElement("uri");
  ASSERT_NE(nullptr, uri);
  ASSERT_NE(nullptr, uri->GetText());
  EXPECT_EQ("sphere", std::string(uri->GetText()));

  name = include->FirstChildElement("name");
  EXPECT_EQ(nullptr, name);

  pose = include->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
  ASSERT_NE(nullptr, pose->GetText());

  ss = std::stringstream(pose->GetText());
  ss >> p;
  EXPECT_EQ(ignition::math::Pose3d(0, 2, 2, 0, 0, 0), p);

  // M3
  model = model->FirstChildElement("model");
  ASSERT_NE(nullptr, model);

  // M3's child include
  include = model->FirstChildElement("include");
  ASSERT_NE(nullptr, include);

  uri = include->FirstChildElement("uri");
  ASSERT_NE(nullptr, uri);
  ASSERT_NE(nullptr, uri->GetText());
  EXPECT_EQ(
    "https://fuel.ignitionrobotics.org/1.0/OpenRobotics/models/Coke Can/2",
     std::string(uri->GetText()));

  name = include->FirstChildElement("name");
  ASSERT_NE(nullptr, name);
  ASSERT_NE(nullptr, name->GetText());
  EXPECT_EQ("coke", std::string(name->GetText()));

  pose = include->FirstChildElement("pose");
  ASSERT_NE(nullptr, pose);
  ASSERT_NE(nullptr, pose->GetText());

  ss = std::stringstream(pose->GetText());
  ss >> p;
  EXPECT_EQ(ignition::math::Pose3d(2, 2, 2, 0, 0, 0), p);

  // check reloading generated sdf
  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
}

/////////////////////////////////////////////////
/// Main
int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  ::testing::AddGlobalTestEnvironment(
      new test::UniqueTestDirectoryEnv("save_world_test_cache"));
  return RUN_ALL_TESTS();
}
