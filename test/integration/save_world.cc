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
#include <tinyxml2.h>

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "helpers/UniqueTestDirectoryEnv.hh"
#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
class SdfGeneratorFixture : public ::testing::Test
{
  public: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
  }

  public: void LoadWorld(const std::string &_path)
  {
    ServerConfig serverConfig;
    serverConfig.SetResourceCache(test::UniqueTestDirectoryEnv::Path());
    serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH, _path));

    std::cout << "Loading: " << serverConfig.SdfFile() << std::endl;
    this->server = std::make_unique<Server>(serverConfig);
    EXPECT_FALSE(server->Running());
  }
  public: std::string RequestGeneratedSdf()
  {
    transport::Node node;
    msgs::SdfGeneratorConfig req;

    msgs::StringMsg worldGenSdfRes;
    bool result;
    unsigned int timeout = 5000;
    std::string service{"/world/save_world/generate_world_sdf"};
    EXPECT_TRUE(node.Request(service, req, timeout, worldGenSdfRes, result));
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

  const std::string worldGenSdfRes = this->RequestGeneratedSdf();
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
TEST_F(SdfGeneratorFixture, ModelSpawnedWithNewName)
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

  const std::string worldGenSdfRes = this->RequestGeneratedSdf();
  sdf::Root root;
  sdf::Errors err = root.LoadSdfString(worldGenSdfRes);
  EXPECT_TRUE(err.empty());
  auto *world = root.WorldByIndex(0);
  ASSERT_NE(nullptr, world);
  EXPECT_TRUE(world->ModelNameExists("new_model_name"));
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
