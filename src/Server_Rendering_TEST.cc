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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>

#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/sim/rendering/Events.hh>
#include <gz/rendering/Scene.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/EventManager.hh"
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

namespace
{
/// \brief Counts the frames rendered by the Sensors system through the
/// render events, and asks for a frame with ForceRender every few iterations.
class FrameCounter:
  public System,
  public ISystemConfigure,
  public ISystemPreUpdate
{
  /// \brief Constructor
  /// \param[in] _onDemand Count the connections to the render events as
  /// on-demand connections
  /// \param[in] _requestPeriod Ask for a frame on the iterations that are
  /// multiples of this, or never if 0.
  public: FrameCounter(bool _onDemand, uint64_t _requestPeriod)
    : onDemand(_onDemand), requestPeriod(_requestPeriod)
  {
  }

  public: void Configure(const Entity &,
      const std::shared_ptr<const sdf::Element> &,
      EntityComponentManager &, EventManager &_eventMgr) override
  {
    this->eventMgr = &_eventMgr;
    this->preRenderConn = _eventMgr.Connect<events::PreRender>([] {});
    this->renderConn = _eventMgr.Connect<events::Render>([] {});
    this->postRenderConn = _eventMgr.Connect<events::PostRender>(
        [this] { ++this->frames; });
    if (this->onDemand)
    {
      this->onDemandConn =
          _eventMgr.Connect<events::OnDemandRenderConnections>(
          [](unsigned int &_count) { _count += 3u; });
    }
  }

  public: void PreUpdate(const UpdateInfo &_info,
      EntityComponentManager &) override
  {
    if (this->requestPeriod > 0u &&
        _info.iterations % this->requestPeriod == 0u)
    {
      this->eventMgr->Emit<events::ForceRender>();
    }
  }

  /// \brief Frames rendered so far
  public: std::atomic<uint64_t> frames{0u};

  private: bool onDemand;
  private: uint64_t requestPeriod;
  private: EventManager *eventMgr{nullptr};
  private: common::ConnectionPtr preRenderConn;
  private: common::ConnectionPtr renderConn;
  private: common::ConnectionPtr postRenderConn;
  private: common::ConnectionPtr onDemandConn;
};

/// \brief Server with the Sensors system on an empty world
std::unique_ptr<sim::Server> emptyWorldWithSensors()
{
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(PROJECT_SOURCE_PATH,
      "test", "worlds", "empty.sdf"));
  serverConfig.AddPlugin(ServerConfig::PluginInfo("empty", "world",
      sdf::Plugin("gz-sim-sensors-system", "gz::sim::systems::Sensors",
      "<render_engine>ogre2</render_engine>")));
  return std::make_unique<sim::Server>(serverConfig);
}

/// \brief Wait until the counter stops changing
/// \return The final count
uint64_t settledFrames(const FrameCounter &_counter)
{
  uint64_t last = _counter.frames;
  for (int i = 0; i < 50; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const uint64_t now = _counter.frames;
    if (now == last && i >= 5)
      break;
    last = now;
  }
  return last;
}

/// \brief Run the server until the counter sees a frame, which means that
/// rendering is initialized, and let the frames in progress finish.
/// \return The frames rendered so far
uint64_t warmUp(sim::Server &_server, const FrameCounter &_counter)
{
  for (int i = 0; i < 1000 && _counter.frames == 0u; ++i)
  {
    _server.RunOnce(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return settledFrames(_counter);
}
}  // namespace

/////////////////////////////////////////////////
// Systems connected to the render events get a frame on every iteration.
TEST_F(ServerFixture,
    GZ_UTILS_TEST_DISABLED_ON_MAC(RenderEventsRenderEveryIteration))
{
  auto server = emptyWorldWithSensors();
  auto counter = std::make_shared<FrameCounter>(false, 10u);
  ASSERT_TRUE(server->AddSystem(counter).value_or(false));

  const uint64_t before = warmUp(*server, *counter);
  ASSERT_GT(before, 0u) << "Rendering did not start";

  ASSERT_TRUE(server->Run(true, 100, false));
  EXPECT_EQ(before + 100u, settledFrames(*counter));
}

/////////////////////////////////////////////////
// Systems that count their connections as on-demand only get the frames
// that they ask for, even if no sensor is due.
TEST_F(ServerFixture,
    GZ_UTILS_TEST_DISABLED_ON_MAC(OnDemandRenderEventsRenderWhenRequested))
{
  auto server = emptyWorldWithSensors();
  auto counter = std::make_shared<FrameCounter>(true, 10u);
  ASSERT_TRUE(server->AddSystem(counter).value_or(false));

  const uint64_t before = warmUp(*server, *counter);
  ASSERT_GT(before, 0u) << "Rendering did not start";

  ASSERT_TRUE(server->Run(true, 100, false));
  EXPECT_EQ(before + 10u, settledFrames(*counter));
}

/////////////////////////////////////////////////
// On-demand connections don't stop other connections from getting a frame
// on every iteration.
TEST_F(ServerFixture,
    GZ_UTILS_TEST_DISABLED_ON_MAC(OnDemandRenderEventsKeepOtherConnections))
{
  auto server = emptyWorldWithSensors();
  auto onDemand = std::make_shared<FrameCounter>(true, 10u);
  auto everyIteration = std::make_shared<FrameCounter>(false, 0u);
  ASSERT_TRUE(server->AddSystem(onDemand).value_or(false));
  ASSERT_TRUE(server->AddSystem(everyIteration).value_or(false));

  const uint64_t before = warmUp(*server, *onDemand);
  ASSERT_GT(before, 0u) << "Rendering did not start";

  ASSERT_TRUE(server->Run(true, 100, false));
  EXPECT_EQ(before + 100u, settledFrames(*onDemand));
}
