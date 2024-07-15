/*
 * Copyright (C) 2023 Open Source Robotics Foundation
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

#include <string>
#include <unordered_set>
#include <vector>

#include <gz/utils/ExtraTestMacros.hh>

#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>

#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "test_config.hh"

#include "gz/sim/rendering/Events.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace std::chrono_literals;

// Pointer to scene
rendering::ScenePtr g_scene;

// Map of model names to their poses
std::unordered_map<std::string, std::vector<math::Pose3d>> g_modelPoses;

// mutex to project model poses
std::mutex g_mutex;

/////////////////////////////////////////////////
void OnPostRender()
{
  if (!g_scene)
  {
    g_scene = rendering::sceneFromFirstRenderEngine();
  }
  ASSERT_TRUE(g_scene);

  auto rootVis = g_scene->RootVisual();
  ASSERT_TRUE(rootVis);

  // store all the model poses
  std::lock_guard<std::mutex> lock(g_mutex);
  for (unsigned int i = 0; i < rootVis->ChildCount(); ++i)
  {
    auto vis = rootVis->ChildByIndex(i);
    ASSERT_TRUE(vis);
    g_modelPoses[vis->Name()].push_back(vis->WorldPose());
  }
}

//////////////////////////////////////////////////
class ActorFixture : public InternalFixture<InternalFixture<::testing::Test>>
{
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    sdf::Plugin sdfPlugin;
    sdfPlugin.SetFilename("libMockSystem.so");
    sdfPlugin.SetName("gz::sim::MockSystem");
    auto plugin = sm.LoadPlugin(sdfPlugin);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();
    this->mockSystem = static_cast<sim::MockSystem *>(
        systemPtr->QueryInterface<sim::System>());
  }

  public: sim::SystemPluginPtr systemPtr;
  public: sim::MockSystem *mockSystem;

  private: sim::SystemLoader sm;
};

/////////////////////////////////////////////////
// Load the actor_trajectory.sdf world that animates a box (actor) to follow
// a trajectory. Verify that the box pose changes over time on the rendering
// side.
TEST_F(ActorFixture, ActorTrajectoryNoMesh)
{
  sim::ServerConfig serverConfig;

  const std::string sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/actor_trajectory.sdf";

  serverConfig.SetSdfFile(sdfFile);
  sim::Server server(serverConfig);

  common::ConnectionPtr postRenderConn;

  // A pointer to the ecm. This will be valid once we run the mock system
  sim::EntityComponentManager *ecm = nullptr;
  this->mockSystem->preUpdateCallback =
    [&ecm](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
    {
      ecm = &_ecm;
    };
  this->mockSystem->configureCallback =
    [&](const sim::Entity &,
           const std::shared_ptr<const sdf::Element> &,
           sim::EntityComponentManager &,
           sim::EventManager &_eventMgr)
    {
      postRenderConn = _eventMgr.Connect<sim::events::PostRender>(
          std::bind(&::OnPostRender));
    };

  server.AddSystem(this->systemPtr);
  server.Run(true, 500, false);
  ASSERT_NE(nullptr, ecm);

  // verify that pose of the animated box exists
  bool hasBoxPose = false;
  int sleep = 0;
  int maxSleep = 50;
  const std::string boxName = "animated_box";
  unsigned int boxPoseCount = 0u;
  while (!hasBoxPose && sleep++ < maxSleep)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_modelPoses.find(boxName) != g_modelPoses.end())
    {
      hasBoxPose = true;
      boxPoseCount = g_modelPoses.size();
    }
  }
  EXPECT_TRUE(hasBoxPose);
  EXPECT_LT(1u, boxPoseCount);

  // check that box is animated, i.e. pose changes over time
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_modelPoses.find(boxName);
    auto &poses = it->second;
    for (unsigned int i = 0; i < poses.size()-2; i+=2)
    {
      // There could be times when the rendering thread has not updated
      // between PostUpdates so two consecutive poses may still be the same.
      // So check for diff between every other pose
      EXPECT_NE(poses[i], poses[i+2]);
    }
  }

  g_scene.reset();
}
