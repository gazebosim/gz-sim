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
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/rendering/Scene.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/System.hh"

#include "plugins/MockSystem.hh"
#include "../test/helpers/Relay.hh"
#include "../test/helpers/EnvTestFixture.hh"

using namespace gz;
using namespace gz::sim;

/////////////////////////////////////////////////
class ServerFixture : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
TEST_F(ServerFixture, LoadSdfModel)
{
  gz::sim::ServerConfig serverConfig;

  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "models", "sphere", "model.sdf"));

  sim::Server server = Server(serverConfig);
  EXPECT_TRUE(server.HasEntity("sphere"));
}

/////////////////////////////////////////////////
TEST_F(ServerFixture, GZ_UTILS_TEST_DISABLED_ON_MAC(LoadSdfModelRelativeUri))
{

  class CheckMeshPlugin:
    public System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
  {
    private: common::ConnectionPtr connection{nullptr};

    private: rendering::ScenePtr scene{nullptr};

    private: EventManager *eventMgr{nullptr};

    void FindScene()
    {
      auto loadedEngNames = gz::rendering::loadedEngines();
      ASSERT_EQ(loadedEngNames.size(), 1);

      auto engineName = loadedEngNames[0];
      auto engine = gz::rendering::engine(engineName);
      ASSERT_TRUE(engine);
      ASSERT_EQ(engine->SceneCount(), 1);

      auto scenePtr = engine->SceneByIndex(0);
      ASSERT_NE(scenePtr, nullptr);
      ASSERT_TRUE(scenePtr->IsInitialized());
      ASSERT_NE(scenePtr->RootVisual(), nullptr);

      this->scene = scenePtr;
    };

    private: void CheckMeshes(){
      if (this->scene == nullptr){
        this->FindScene();
      }

      std::shared_ptr<rendering::Visual> v1 = this->scene->VisualByName(
        "relative_resource_uri::L1::V1"
      );
      // There's only one geometry under this visual, which has to be the mesh.
      EXPECT_EQ(v1->GeometryCount(), 1);
      std::shared_ptr<rendering::Geometry> v1Geom = v1->GeometryByIndex(0);
      // Attempt to cast the geometry into a mesh, in order to determine that
      // the mesh has been properly loaded.
      std::shared_ptr<rendering::Mesh> v1Mesh =
        std::dynamic_pointer_cast<rendering::Mesh>(v1Geom);
      EXPECT_NE(v1Mesh, nullptr);

      std::shared_ptr<rendering::Visual> v2 = this->scene->VisualByName(
        "relative_resource_uri::L1::V2"
      );
      // There should be no geometries under this visual, as the mesh file
      // refers to a non-existent file, and the mesh should not be loaded.
      EXPECT_EQ(v2->GeometryCount(), 0);
    }

    public: void Configure(
      const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &,
      EventManager &_eventMgr
    )
    {
      // Register CheckMeshes with the PreRender event.
      this->connection =
        _eventMgr.Connect<events::PreRender>(
          std::bind(&CheckMeshPlugin::CheckMeshes, this)
        );
      this->eventMgr = &_eventMgr;
    };

    public: void PreUpdate(const UpdateInfo &, EntityComponentManager &)
    {
      // Emit a ForceRender event so the PreRender event is triggered.
      this->eventMgr->Emit<events::ForceRender>();
    };
  };

  gz::sim::ServerConfig serverConfig;
  serverConfig.SetBehaviorOnSdfErrors(
      ServerConfig::SdfErrorBehavior::CONTINUE_LOADING);
  EXPECT_EQ(ServerConfig::SdfErrorBehavior::CONTINUE_LOADING,
      serverConfig.BehaviorOnSdfErrors());
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "models", "relative_resource_uri", "model2.sdf"));

  // Add the Sensors plugin so rendering is available.
  sdf::Plugin sdfSensorsPlugin = sdf::Plugin(
    "gz-sim-sensors-system",
    "gz::sim::systems::Sensors",
    "<render_engine>ogre2</render_engine>"
  );
  ServerConfig::PluginInfo sensorsPluginInfo = ServerConfig::PluginInfo(
    "default",
    "world",
    sdfSensorsPlugin
  );
  serverConfig.AddPlugin(sensorsPluginInfo);

  sim::Server server = Server(serverConfig);
  EXPECT_TRUE(server.HasEntity("relative_resource_uri"));
  EXPECT_TRUE(server.HasEntity("L1"));
  EXPECT_TRUE(server.HasEntity("V1"));
  EXPECT_TRUE(server.HasEntity("V2"));

  std::shared_ptr<CheckMeshPlugin> meshChecker =
    std::make_shared<CheckMeshPlugin>();
  std::optional<bool> meshCheckerAddSuccess = server.AddSystem(meshChecker);
  ASSERT_TRUE(meshCheckerAddSuccess);
  if (meshCheckerAddSuccess) {
    ASSERT_TRUE(meshCheckerAddSuccess.value());
  }
  ASSERT_TRUE(server.RunOnce());
  ASSERT_TRUE(server.RunOnce(false));

  // Tell server to stop loading if there are SDF errors
  // Server should not load because V2's visual mesh URI can not be resolved
  serverConfig.SetBehaviorOnSdfErrors(
      ServerConfig::SdfErrorBehavior::EXIT_IMMEDIATELY);
  EXPECT_EQ(ServerConfig::SdfErrorBehavior::EXIT_IMMEDIATELY,
      serverConfig.BehaviorOnSdfErrors());
  sim::Server server2 = Server(serverConfig);
  EXPECT_FALSE(server2.HasEntity("relative_resource_uri"));
  EXPECT_FALSE(server2.HasEntity("L1"));
  EXPECT_FALSE(server2.HasEntity("V1"));
  EXPECT_FALSE(server2.HasEntity("V2"));
}
