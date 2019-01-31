/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <array>

#include <ignition/common/Console.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"  // NOLINT(build/include)

#include "ignition/gazebo/components/Level.hh"
#include "ignition/gazebo/components/LevelEntityNames.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/Performer.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

class Relay
{
  public: Relay()
  {
    auto plugin = sm.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();
    this->mockSystem = static_cast<gazebo::MockSystem *>(
        systemPtr->QueryInterface<gazebo::System>());
  }

  public: Relay &OnPreUpdate(gazebo::MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnUpdate(gazebo::MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnPostUpdate(
              gazebo::MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: ignition::gazebo::SystemPluginPtr systemPtr;

  protected: gazebo::SystemLoader sm;
  protected: gazebo::MockSystem *mockSystem;
};

/// \brief A system to move models to arbitrary poses. Note that this does not
/// work if the physics system is running.
class ModelMover: public Relay
{
  public: explicit ModelMover(Entity _entity): Relay(), entity(_entity)
  {
    using namespace std::placeholders;
    this->mockSystem->preUpdateCallback =
        std::bind(&ModelMover::MoveModel, this, _1, _2);
  }

  /// \brief Sets the pose of the entity
  /// \param[in] _pose Commanded pose
  public: void SetPose(math::Pose3d _pose)
  {
    poseCmd = std::move(_pose);
  }

  /// \brief Sets the pose component of the entity to the commanded pose. This
  /// function meant to be called in the preupdate phase
  private: void MoveModel(const gazebo::UpdateInfo &,
                          gazebo::EntityComponentManager &_ecm)
  {
    if (this->poseCmd)
    {
      auto poseComp = _ecm.Component<components::Pose>(entity);
      *poseComp = components::Pose(*poseCmd);
      this->poseCmd.reset();
    }
  }

  /// \brief Entity to move
  private: Entity entity;
  /// \brief Pose command
  private: std::optional<math::Pose3d> poseCmd;
};

class LevelManagerFixture : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);

    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
      (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);

    ignition::gazebo::ServerConfig serverConfig;

    // Except tile_0, which is on the default level, every tile belongs to a
    // level. The name of the level corresponds to the tile in its suffix, i.e.,
    // level1 contains tile_1.
    serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                            "/test/worlds/levels.sdf");
    serverConfig.SetUseLevels(true);

    server = std::make_unique<gazebo::Server>(serverConfig);

    Relay testSystem;
    // Check entities loaded on the default level
    testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                            const gazebo::EntityComponentManager &_ecm)
    {
      _ecm.Each<components::Model, components::Name>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name) -> bool
          {
            this->loadedModels.push_back(_name->Data());
            return true;
          });

      _ecm.Each<components::Light, components::Name>(
          [&](const Entity &, const components::Light *,
              const components::Name *_name) -> bool
          {
            this->loadedLights.push_back(_name->Data());
            return true;
          });

      _ecm.EachErased<components::Model, components::Name>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name) -> bool
          {
            this->unloadedModels.push_back(_name->Data());
            return true;
          });
    });

    this->server->AddSystem(testSystem.systemPtr);
  }

  public: void RunServer()
  {
    // 3 iterations are required for unloading a level because the request to
    // erase entities is processed in the next iteration
    this->server->Run(true, 2, false);
    this->loadedModels.clear();
    this->loadedLights.clear();
    this->server->Run(true, 1, false);
  }

  public: std::unique_ptr<gazebo::Server> server;
  public: std::vector<std::string> loadedModels;
  public: std::vector<std::string> unloadedModels;
  public: std::vector<std::string> loadedLights;
};

/////////////////////////////////////////////////
/// Check default level includes entities not included by other levels
TEST_F(LevelManagerFixture, DefaultLevel)
{
  std::vector<std::set<std::string>> levelEntityNamesList;

  Relay recorder;
  // Check entities loaded on the default level
  recorder.OnPostUpdate([&](const gazebo::UpdateInfo &,
                            const gazebo::EntityComponentManager &_ecm)
      {
    _ecm.Each<components::DefaultLevel, components::LevelEntityNames>(
        [&](const Entity &, const components::DefaultLevel *,
            const components::LevelEntityNames *_levelEntityNames) -> bool
        {
          levelEntityNamesList.push_back(_levelEntityNames->Data());
          return true;
        });
  });

  this->server->AddSystem(recorder.systemPtr);
  const int iters = 10;
  this->server->Run(true, iters, false);
  EXPECT_EQ(static_cast<std::size_t>(iters), levelEntityNamesList.size());

  for (const auto &levelEntityNames : levelEntityNamesList)
  {
    EXPECT_TRUE(levelEntityNames.find("sun") != levelEntityNames.end());
    EXPECT_TRUE(levelEntityNames.find("tile_0") != levelEntityNames.end());
    // tile_1 should not be in the default level
    EXPECT_FALSE(levelEntityNames.find("tile_1") != levelEntityNames.end());
  }

  EXPECT_EQ(iters, std::count(this->loadedLights.begin(),
                              this->loadedLights.end(), "sun"));
  EXPECT_EQ(iters, std::count(this->loadedModels.begin(),
                              this->loadedModels.end(), "tile_0"));
  // tile_1 should not be loaded
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
}

///////////////////////////////////////////////
/// Check a level is loaded when a performer is inside a level
/// Check a level is unloaded when a performer is outside a level
TEST_F(LevelManagerFixture, LevelLoadUnload)
{
  ModelMover sphereMover(*this->server->EntityByName("sphere"));
  this->server->AddSystem(sphereMover.systemPtr);

  std::array entitiesNonDefault{"tile_1", "tile_2", "tile_3", "tile_4",
                                "tile_5"};

  // Run once and check levels
  this->server->Run(true, 1, false);

  // Non of the non-default levels should be loaded
  for (const auto &name : entitiesNonDefault)
  {
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), name));
  }

  // Move sphere into level1
  sphereMover.SetPose({40, 0, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should be loaded
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));

  // Move sphere out of level1
  sphereMover.SetPose({0, 0, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should be unloaded
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
}

///////////////////////////////////////////////
/// Check behaviour of level buffers
TEST_F(LevelManagerFixture, LevelBuffers)
{
  ModelMover sphereMover(*this->server->EntityByName("sphere"));
  this->server->AddSystem(sphereMover.systemPtr);

  // Move sphere into level1
  sphereMover.SetPose({40, 0, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should be loaded
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));

  // Move sphere out of level1 but remain in the buffer
  sphereMover.SetPose({40, 20, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should still be loaded
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));

  // Move sphere out of level1
  sphereMover.SetPose({0, 0, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should be unloaded
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));

  // Move sphere into level1's buffer
  sphereMover.SetPose({40, 20, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should remain unloaded when entering from outside the level
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
}

///////////////////////////////////////////////
/// Check that multiple performers can load/unload multiple levels independently
TEST_F(LevelManagerFixture, LevelsWithMultiplePerformers)
{
  ModelMover sphereMover(*this->server->EntityByName("sphere"));
  ModelMover boxMover(*this->server->EntityByName("box"));

  this->server->AddSystem(sphereMover.systemPtr);
  this->server->AddSystem(boxMover.systemPtr);

  const math::Pose3d noLevelPose{0, 0, 0, 0, 0, 0};
  const math::Pose3d level1Pose{40, -10, 0, 0, 0, 0};
  const math::Pose3d level2Pose{40, 30, 0, 0, 0, 0};

  // Move sphere into level1 and box to level2
  sphereMover.SetPose(level1Pose);
  boxMover.SetPose(level2Pose);

  EXPECT_EQ(0u, this->unloadedModels.size());

  this->RunServer();

  // Level1 should be loaded
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_2"));
  EXPECT_EQ(0u, this->unloadedModels.size());

  // Move sphere out of level1
  sphereMover.SetPose(noLevelPose);

  this->RunServer();

  // Level1 should be unloaded
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(1, std::count(this->unloadedModels.begin(),
                          this->unloadedModels.end(), "tile_1"));
  // Level2 should remain loaded because the box is still in there
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_2"));
  EXPECT_EQ(0, std::count(this->unloadedModels.begin(),
                          this->unloadedModels.end(), "tile_2"));

  // Check that the state of the levels remains the same for n iterations
  const int iters = 100;
  this->server->Run(true, iters, false);
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  // Level2 should remain loaded because the box is still in there
  EXPECT_EQ(1 + iters, std::count(this->loadedModels.begin(),
                                  this->loadedModels.end(), "tile_2"));

  // Check that a level remains loaded when performers move out of the level if
  // there is at least one performer left in the level

  // Move sphere to level2
  sphereMover.SetPose(level2Pose);
  this->RunServer();
  // Both performers are in level2
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_2"));

  this->server->Run(true, iters, false);
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(1 + iters, std::count(this->loadedModels.begin(),
                                  this->loadedModels.end(), "tile_2"));

  // Move box out of level2.
  boxMover.SetPose(noLevelPose);
  this->RunServer();
  // sphere is still in level2 so it should still be loaded
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_2"));

  this->server->Run(true, iters, false);
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(1 + iters, std::count(this->loadedModels.begin(),
                                  this->loadedModels.end(), "tile_2"));

  // Move sphere out of level2
  sphereMover.SetPose(noLevelPose);
  this->RunServer();
  // All performers are outside of levels
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_2"));

  this->server->Run(true, iters, false);
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_2"));
}

///////////////////////////////////////////////
/// Check that buffers work properly with multiple performers
TEST_F(LevelManagerFixture, LevelBuffersWithMultiplePerformers)
{
  ModelMover sphereMover(*this->server->EntityByName("sphere"));
  ModelMover boxMover(*this->server->EntityByName("box"));

  this->server->AddSystem(sphereMover.systemPtr);
  this->server->AddSystem(boxMover.systemPtr);

  const math::Pose3d noLevelPose{0, 0, 0, 0, 0, 0};
  const math::Pose3d level1Pose{40, 0, 0, 0, 0, 0};
  const math::Pose3d level1BufferPose{40, -20, 0, 0, 0, 0};

  // Move sphere into level1 and box to level1's buffer
  sphereMover.SetPose(level1Pose);
  boxMover.SetPose(level1BufferPose);

  this->RunServer();

  // Level1 should be loaded
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_2"));

  // Move sphere to level1's buffer
  sphereMover.SetPose(level1BufferPose);
  this->RunServer();

  // Level1 should still be loaded
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  // Level2 should remain unloaded
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_2"));
}
