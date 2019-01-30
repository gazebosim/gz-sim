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

#include <array>

#include <gtest/gtest.h>
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

class LevelManagerFixture : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    common::Console::SetVerbosity(4);

    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
      (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

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

  public: virtual Relay &OnPreUpdate(gazebo::MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: virtual Relay &OnUpdate(gazebo::MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: virtual Relay &OnPostUpdate(
              gazebo::MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: ignition::gazebo::SystemPluginPtr systemPtr;

  protected: gazebo::SystemLoader sm;
  protected: gazebo::MockSystem *mockSystem;
};

class ModelMover: public Relay
{
  public: ModelMover(Entity _entity): Relay(), entity(_entity)
  {
    using namespace std::placeholders;
    this->mockSystem->preUpdateCallback =
        std::bind(&ModelMover::MoveModel, this, _1, _2);
  }
  public: void SetPose(math::Pose3d _pose)
  {
    poseCmd = std::move(_pose);
  }

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

/////////////////////////////////////////////////
/// Check default level includes entities not included by other levels
TEST_F(LevelManagerFixture, DefaultLevel)
{
  ignition::gazebo::ServerConfig serverConfig;

  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                          "/test/worlds/levels.sdf");
  serverConfig.SetUseLevels(true);

  gazebo::Server server(serverConfig);

  server.SetUpdatePeriod(1ns);

  Relay testSystem;

  std::vector<std::set<std::string>> levelEntityNamesList;
  std::vector<std::string> loadedModels;
  std::vector<std::string> loadedLights;

  // Check entities loaded on the default level
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                          const gazebo::EntityComponentManager &_ecm)
  {
    _ecm.Each<components::DefaultLevel, components::LevelEntityNames>(
        [&](const Entity &, const components::DefaultLevel *,
            const components::LevelEntityNames *_levelEntityNames) -> bool
        {
          levelEntityNamesList.push_back(_levelEntityNames->Data());
          return true;
        });

    _ecm.Each<components::Model, components::Name>(
        [&](const Entity &, const components::Model *,
            const components::Name *_name) -> bool
        {
          loadedModels.push_back(_name->Data());
          return true;
        });
    _ecm.Each<components::Light, components::Name>(
        [&](const Entity &, const components::Light *,
            const components::Name *_name) -> bool
        {
          loadedLights.push_back(_name->Data());
          return true;
        });
  });

  server.AddSystem(testSystem.systemPtr);
  const size_t iters = 10;
  server.Run(true, iters, false);
  EXPECT_EQ(iters, levelEntityNamesList.size());

  for (const auto &levelEntityNames : levelEntityNamesList)
  {
    EXPECT_TRUE(levelEntityNames.find("sun") != levelEntityNames.end());
    EXPECT_TRUE(levelEntityNames.find("tile_0") != levelEntityNames.end());
    // tile_1 should not be in the default level
    EXPECT_FALSE(levelEntityNames.find("tile_1") != levelEntityNames.end());
  }

  EXPECT_EQ(static_cast<int>(iters),
            std::count(loadedLights.begin(), loadedLights.end(), "sun"));
  EXPECT_EQ(static_cast<int>(iters),
            std::count(loadedModels.begin(), loadedModels.end(), "tile_0"));
  // tile_1 should not be loaded
  EXPECT_EQ(0, std::count(loadedModels.begin(), loadedModels.end(), "tile_1"));
}

///////////////////////////////////////////////
/// Check a level is loaded when a performer is inside a level
/// Check a level is unloaded when a performer is outside a level
TEST_F(LevelManagerFixture, LevelLoadUnload)
{
  // Except tile_0, which is on the default level, every tile belongs to a
  // level. The name of the level corresponds to the tile in its suffix, i.e.,
  // level1 contains tile_1.
  ignition::gazebo::ServerConfig serverConfig;

  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                          "/test/worlds/levels.sdf");
  serverConfig.SetUseLevels(true);

  gazebo::Server server(serverConfig);

  auto sphereEntity = server.EntityByName("sphere");

  // This system will move the sphere to desired locations
  ModelMover sphereMover(*sphereEntity);

  std::vector<std::string> loadedModels;

  Relay testSystem;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm) {
    _ecm.Each<components::Model, components::Name>(
        [&](const Entity &, const components::Model *,
            const components::Name *_name) -> bool
        {
          loadedModels.push_back(_name->Data());
          return true;
        });
  });

  server.AddSystem(sphereMover.systemPtr);
  server.AddSystem(testSystem.systemPtr);

  std::array entitiesNonDefault{"tile_1", "tile_2", "tile_3", "tile_4",
                                "tile_5"};

  // Run once and check levels
  server.Run(true, 1, false);

  // Non of the non-default levels should be loaded
  for (const auto &name : entitiesNonDefault)
  {
    EXPECT_EQ(0, std::count(loadedModels.begin(), loadedModels.end(), name));
  }

  // Move sphere into level1
  sphereMover.SetPose({40, 0, 0, 0, 0, 0});

  loadedModels.clear();
  // 2 iterations are required to set the sphere in position and to let the
  // level manager load entities
  server.Run(true, 2, false);

  // Level1 should be loaded
  EXPECT_EQ(1, std::count(loadedModels.begin(), loadedModels.end(), "tile_1"));

  // Move sphere out of level1
  sphereMover.SetPose({0, 0, 0, 0, 0, 0});

  // 3 iterations are required for unloading a level because the request to
  // erase entities is processed in the next iteration
  server.Run(true, 2, false);
  loadedModels.clear();
  server.Run(true, 1, false);

  // Level1 should be unloaded
  EXPECT_EQ(0, std::count(loadedModels.begin(), loadedModels.end(), "tile_1"));
}

