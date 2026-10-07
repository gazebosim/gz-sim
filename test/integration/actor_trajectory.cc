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

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gz/math/Matrix4.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include <gz/rendering/Mesh.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>

#include "gz/sim/Actor.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "gz/sim/components/Actor.hh"
#include "gz/sim/components/Name.hh"
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

// Skeleton local transforms of the walker actor, one entry per render
std::vector<std::map<std::string, math::Matrix4d>> g_boneTransforms;

// World poses of the walker actor, one entry per render
std::vector<math::Pose3d> g_actorPoses;

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

/////////////////////////////////////////////////
void OnPostRenderBones()
{
  if (!g_scene)
  {
    g_scene = rendering::sceneFromFirstRenderEngine();
  }
  ASSERT_TRUE(g_scene);

  auto actorVis = g_scene->VisualByName("walker");
  if (!actorVis || actorVis->GeometryCount() == 0u)
    return;

  auto mesh = std::dynamic_pointer_cast<rendering::Mesh>(
      actorVis->GeometryByIndex(0));
  if (!mesh || !mesh->HasSkeleton())
    return;

  std::lock_guard<std::mutex> lock(g_mutex);
  g_boneTransforms.push_back(mesh->SkeletonLocalTransforms());
  g_actorPoses.push_back(actorVis->WorldPose());
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

/////////////////////////////////////////////////
// Load the actor_bone_transforms.sdf world that animates a skinned actor
// along a trajectory. Verify on the rendering side that the skeleton takes
// the bone transforms set through the BoneTransforms component, that the
// other bones hold their pose and the actor keeps following its trajectory,
// and that the animation plays again once the component is removed.
TEST_F(ActorFixture, GZ_UTILS_TEST_DISABLED_ON_MAC(ActorBoneTransforms))
{
  sim::ServerConfig serverConfig;

  const std::string sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/actor_bone_transforms.sdf";

  serverConfig.SetSdfFile(sdfFile);
  sim::Server server(serverConfig);

  common::ConnectionPtr postRenderConn;

  // Bone transforms given to the actor while setBones is true. The
  // component is removed once removeBones is true. Protected by g_mutex.
  std::map<std::string, math::Pose3d> boneTransforms;
  bool setBones = false;
  bool removeBones = false;

  this->mockSystem->preUpdateCallback =
    [&](const sim::UpdateInfo &, sim::EntityComponentManager &_ecm)
    {
      auto entity = _ecm.EntityByComponents(sim::components::Name("walker"));
      if (sim::kNullEntity == entity)
        return;

      std::lock_guard<std::mutex> lock(g_mutex);
      if (setBones)
      {
        sim::Actor actor(entity);
        actor.SetBoneTransforms(_ecm, boneTransforms);
      }
      else if (removeBones)
      {
        _ecm.RemoveComponent<sim::components::BoneTransforms>(entity);
        removeBones = false;
      }
    };
  this->mockSystem->configureCallback =
    [&](const sim::Entity &,
           const std::shared_ptr<const sdf::Element> &,
           sim::EntityComponentManager &,
           sim::EventManager &_eventMgr)
    {
      postRenderConn = _eventMgr.Connect<sim::events::PostRender>(
          std::bind(&::OnPostRenderBones));
    };

  server.AddSystem(this->systemPtr);
  server.Run(false, 0, false);

  // Wait until the recorded renders meet the condition
  auto waitFor = [](const std::function<bool()> &_condition)
  {
    int sleep = 0;
    int maxSleep = 600;
    while (sleep++ < maxSleep)
    {
      {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (_condition())
          return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
  };

  // True if the skeleton has the bone transforms given to the actor
  auto hasBoneTransforms =
    [&](const std::map<std::string, math::Matrix4d> &_transforms)
  {
    for (const auto &[name, pose] : boneTransforms)
    {
      auto it = _transforms.find(name);
      if (it == _transforms.end() ||
          !it->second.Equal(math::Matrix4d(pose), 1e-3))
      {
        return false;
      }
    }
    return true;
  };

  const unsigned int renderCount = 10u;

  // the script animates the skeleton and moves the actor
  ASSERT_TRUE(waitFor([&]()
      {
        return g_boneTransforms.size() >= renderCount;
      }));
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    ASSERT_FALSE(g_boneTransforms.back().empty());
    EXPECT_NE(g_boneTransforms.front(), g_boneTransforms.back());
    EXPECT_NE(g_actorPoses.front(), g_actorPoses.back());

    // give every other bone a rotation that no animation frame has
    bool give = true;
    for (const auto &[name, transform] : g_boneTransforms.back())
    {
      if (give)
      {
        boneTransforms[name] = math::Pose3d(transform.Translation(),
            math::Quaterniond(0.1, 0.2, 0.3));
      }
      give = !give;
    }
    ASSERT_FALSE(boneTransforms.empty());
    ASSERT_LT(boneTransforms.size(), g_boneTransforms.back().size());
    setBones = true;
  }

  // the skeleton takes the bone transforms
  EXPECT_TRUE(waitFor([&]()
      {
        return !g_boneTransforms.empty() &&
            hasBoneTransforms(g_boneTransforms.back());
      }));
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_boneTransforms.clear();
    g_actorPoses.clear();
  }

  // and keeps them, the other bones hold their pose and the actor follows
  // its trajectory
  ASSERT_TRUE(waitFor([&]()
      {
        return g_boneTransforms.size() >= renderCount;
      }));
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    for (const auto &transforms : g_boneTransforms)
    {
      EXPECT_TRUE(hasBoneTransforms(transforms));
      EXPECT_EQ(g_boneTransforms.front(), transforms);
    }
    EXPECT_NE(g_actorPoses.front(), g_actorPoses.back());

    setBones = false;
    removeBones = true;
  }

  // the animation plays again once the component is removed
  EXPECT_TRUE(waitFor([&]()
      {
        return !g_boneTransforms.empty() &&
            !hasBoneTransforms(g_boneTransforms.back());
      }));
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_boneTransforms.clear();
    g_actorPoses.clear();
  }
  ASSERT_TRUE(waitFor([&]()
      {
        return g_boneTransforms.size() >= renderCount;
      }));
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    for (const auto &transforms : g_boneTransforms)
      EXPECT_FALSE(hasBoneTransforms(transforms));
    EXPECT_NE(g_boneTransforms.front(), g_boneTransforms.back());
    EXPECT_NE(g_actorPoses.front(), g_actorPoses.back());
  }

  server.Stop();

  std::lock_guard<std::mutex> lock(g_mutex);
  g_boneTransforms.clear();
  g_actorPoses.clear();
  g_scene.reset();
}
