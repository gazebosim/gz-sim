/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <ignition/common/ColladaLoader.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/test_config.hh"

#include "helpers/UniqueTestDirectoryEnv.hh"

using namespace ignition;
using namespace gazebo;

/////////////////////////////////////////////////
class ColladaWorldExporterFixture : public ::testing::Test
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

  public: std::unique_ptr<Server> server;
};

/////////////////////////////////////////////////
TEST_F(ColladaWorldExporterFixture, ExportWorld)
{
  this->LoadWorld(common::joinPaths("test", "worlds",
        "collada_world_exporter.sdf"));

  // Cleanup
  common::removeAll("./collada_world_exporter_box_test");

  // The export directory shouldn't exist.
  EXPECT_FALSE(common::exists("./collada_world_exporter_box_test"));

  // Run one iteration which should export the world.
  server->Run(true, 1, false);

  // The export directory should now exist.
  EXPECT_TRUE(common::exists("./collada_world_exporter_box_test"));

  // Cleanup
  common::removeAll("./collada_world_exporter_box_test");
}

TEST_F(ColladaWorldExporterFixture, ExportWorldFromFuelWithSubmesh)
{
  ignition::common::setenv("IGN_GAZEBO_RESOURCE_PATH",
    (std::string(PROJECT_SOURCE_PATH) + "/test/worlds:" +
    std::string(PROJECT_SOURCE_PATH) + "/test/worlds/models").c_str());

  this->LoadWorld(common::joinPaths("test", "worlds",
        "collada_world_exporter_submesh.sdf"));

  const std::string outputPath = "./collada_world_exporter_submesh_test";

  // Cleanup
  common::removeAll(outputPath);

  // The export directory shouldn't exist.
  EXPECT_FALSE(common::exists(outputPath));

  // Run one iteration which should export the world.
  server->Run(true, 1, false);

  // The export directory should now exist.
  EXPECT_TRUE(common::exists(outputPath));

  // Original .dae file has two submeshes
  // .sdf loads them together and a submesh alone
  // Check that output has three nodes
  common::ColladaLoader loader;
  const common::Mesh *meshExported = loader.Load(common::joinPaths(
      outputPath, "meshes", "collada_world_exporter_submesh_test.dae"));
  EXPECT_EQ(3u, meshExported->SubMeshCount());

  // Cleanup
  common::removeAll(outputPath);
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
