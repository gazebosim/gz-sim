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

#include <vector>

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
#include "ignition/gazebo/components/LevelBuffer.hh"
#include "ignition/gazebo/components/LevelEntityNames.hh"
#include "ignition/gazebo/components/Light.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParentEntity.hh"
#include "ignition/gazebo/components/ParentLinkName.hh"
#include "ignition/gazebo/components/PerformerLevels.hh"
#include "ignition/gazebo/components/Pose.hh"

#include "../helpers/Relay.hh"

using namespace ignition;
using namespace gazebo;
using namespace std::chrono_literals;

//////////////////////////////////////////////////
/// \brief A system to move models to arbitrary poses. Note that this does not
/// work if the physics system is running.
class ModelMover: public test::Relay
{
  public: explicit ModelMover(Entity _entity): test::Relay(), entity(_entity)
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

  public: gazebo::Entity Id() const
  {
    return entity;
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
  private: gazebo::Entity entity;
  /// \brief Pose command
  private: std::optional<math::Pose3d> poseCmd;
};

//////////////////////////////////////////////////
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

    test::Relay testSystem;
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

      _ecm.Each<components::PerformerLevels>(
          [&](const Entity &_performer,
              const components::PerformerLevels *_levels) -> bool
          {
            this->performerLevels[_performer] = _levels->Data();
            return true;
          });

      _ecm.EachRemoved<components::Model, components::Name>(
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
    this->unloadedModels.clear();
    this->server->Run(true, 2, false);
    this->loadedModels.clear();
    this->loadedLights.clear();
    this->server->Run(true, 1, false);
  }

  public: std::unique_ptr<gazebo::Server> server;
  public: std::vector<std::string> loadedModels;
  public: std::vector<std::string> unloadedModels;
  public: std::vector<std::string> loadedLights;
  public: std::map<Entity, std::set<Entity>> performerLevels;
};

/////////////////////////////////////////////////
/// Check default level includes entities not included by other levels
TEST_F(LevelManagerFixture, DefaultLevel)
{
  std::vector<std::set<std::string>> levelEntityNamesList;

  test::Relay recorder;
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

  // There should be 2 performers
  EXPECT_EQ(2u, this->performerLevels.size());
}

///////////////////////////////////////////////
/// Check a level is loaded when a performer is inside a level
/// Check a level is unloaded when a performer is outside a level
TEST_F(LevelManagerFixture, LevelLoadUnload)
{
  ModelMover perf1(*this->server->EntityByName("sphere"));
  this->server->AddSystem(perf1.systemPtr);

  std::vector<std::string> entitiesNonDefault{"tile_1", "tile_2", "tile_3",
                                              "tile_4", "tile_5"};

  // Run once and check levels
  this->server->Run(true, 1, false);

  // Non of the non-default levels should be loaded
  for (const auto &name : entitiesNonDefault)
  {
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), name));
  }

  // Move performer into level1
  perf1.SetPose({40, 0, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should be loaded
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));

  // Check performer levels
  EXPECT_EQ(2u, this->performerLevels.size());

  auto spherePerf = *this->server->EntityByName("perf_sphere");
  EXPECT_NE(kNullEntity, spherePerf);
  EXPECT_NE(this->performerLevels.find(spherePerf),
      this->performerLevels.end());

  auto level1 = *this->server->EntityByName("level1");
  EXPECT_NE(kNullEntity, level1);
  EXPECT_EQ(1u, this->performerLevels[spherePerf].size());
  EXPECT_EQ(1u, this->performerLevels[spherePerf].count(level1));

  // Move performer out of level1
  perf1.SetPose({0, 0, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should be unloaded
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(1, std::count(this->unloadedModels.begin(),
                          this->unloadedModels.end(), "tile_1"));
}

///////////////////////////////////////////////
/// Check behaviour of level buffers
TEST_F(LevelManagerFixture, LevelBuffers)
{
  ModelMover perf1(*this->server->EntityByName("sphere"));
  this->server->AddSystem(perf1.systemPtr);

  // Move performer into level1
  perf1.SetPose({40, 0, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should be loaded
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));

  // Move performer out of level1 but remain in the buffer
  perf1.SetPose({40, 20, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should still be loaded
  EXPECT_EQ(1, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));

  // Move performer out of level1
  perf1.SetPose({0, 0, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should be unloaded
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
  EXPECT_EQ(1, std::count(this->unloadedModels.begin(),
                          this->unloadedModels.end(), "tile_1"));

  // Move performer into level1's buffer
  perf1.SetPose({40, 20, 0, 0, 0, 0});
  this->RunServer();
  // Level1 should remain unloaded when entering from outside the level
  EXPECT_EQ(0, std::count(this->loadedModels.begin(), this->loadedModels.end(),
                          "tile_1"));
}

///////////////////////////////////////////////
/// Check that multiple performers can load/unload multiple levels independently
TEST_F(LevelManagerFixture, LevelsWithMultiplePerformers)
{
  ModelMover perf1(*this->server->EntityByName("sphere"));
  ModelMover perf2(*this->server->EntityByName("box"));

  this->server->AddSystem(perf1.systemPtr);
  this->server->AddSystem(perf2.systemPtr);

  const math::Pose3d noLevelPose{0, 0, 0, 0, 0, 0};
  const math::Pose3d level1Pose{40, -10, 0, 0, 0, 0};
  const math::Pose3d level2Pose{40, 30, 0, 0, 0, 0};

  auto testSequence = [&](ModelMover &_perf1, ModelMover &_perf2)
  {
    igndbg << "Testing performer1 [" << _perf1.Id() << "] and performer2 ["
           << _perf2.Id() << "]\n";

    // Reset positions
    perf1.SetPose(noLevelPose);
    perf2.SetPose(noLevelPose);
    this->RunServer();

    // Move performer1 into level1 and performer2 to level2
    _perf1.SetPose(level1Pose);
    _perf2.SetPose(level2Pose);

    EXPECT_EQ(0u, this->unloadedModels.size());

    this->RunServer();

    // Level1 should be loaded
    EXPECT_EQ(1, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    EXPECT_EQ(1, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_2"));
    EXPECT_EQ(0u, this->unloadedModels.size());

    // Move performer1 out of level1
    _perf1.SetPose(noLevelPose);

    this->RunServer();

    // Level1 should be unloaded
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    EXPECT_EQ(1, std::count(this->unloadedModels.begin(),
                            this->unloadedModels.end(), "tile_1"));
    // Level2 should remain loaded because the performer2 is still in there
    EXPECT_EQ(1, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_2"));
    EXPECT_EQ(0, std::count(this->unloadedModels.begin(),
                            this->unloadedModels.end(), "tile_2"));

    // Check that the state of the levels remains the same for n iterations
    const int iters = 100;
    this->server->Run(true, iters, false);
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    // Level2 should remain loaded because the performer2 is still in there
    EXPECT_EQ(1 + iters, std::count(this->loadedModels.begin(),
                                    this->loadedModels.end(), "tile_2"));

    // Check that a level remains loaded when performers move out of the level
    // if there is at least one performer left in the level

    // Move performer1 to level2
    _perf1.SetPose(level2Pose);
    this->RunServer();
    // Both performers are in level2
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    EXPECT_EQ(1, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_2"));

    this->server->Run(true, iters, false);
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    EXPECT_EQ(1 + iters, std::count(this->loadedModels.begin(),
                                    this->loadedModels.end(), "tile_2"));

    // Move performer2 out of level2.
    _perf2.SetPose(noLevelPose);
    this->RunServer();
    // performer1 is still in level2 so it should still be loaded
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    EXPECT_EQ(1, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_2"));

    this->server->Run(true, iters, false);
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    EXPECT_EQ(1 + iters, std::count(this->loadedModels.begin(),
                                    this->loadedModels.end(), "tile_2"));

    // Move performer1 out of level2
    _perf1.SetPose(noLevelPose);
    this->RunServer();
    // All performers are outside of levels
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_2"));

    this->server->Run(true, iters, false);
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_2"));
  };

  testSequence(perf1, perf2);
  testSequence(perf2, perf1);
}

///////////////////////////////////////////////
/// Check that buffers work properly with multiple performers
TEST_F(LevelManagerFixture, LevelBuffersWithMultiplePerformers)
{
  ModelMover perf1(*this->server->EntityByName("sphere"));
  ModelMover perf2(*this->server->EntityByName("box"));

  this->server->AddSystem(perf1.systemPtr);
  this->server->AddSystem(perf2.systemPtr);

  const math::Pose3d noLevelPose{0, 0, 0, 0, 0, 0};
  const math::Pose3d level1Pose{40, 0, 0, 0, 0, 0};
  const math::Pose3d level1BufferPose{40, -20, 0, 0, 0, 0};

  auto testSequence = [&](ModelMover &_perf1, ModelMover &_perf2)
  {
    igndbg << "Testing performer1 [" << _perf1.Id() << "] and performer2 ["
           << _perf2.Id() << "]\n";
    // Move performer1 into level1 and performer2 to level1's buffer
    _perf1.SetPose(level1Pose);
    _perf2.SetPose(level1BufferPose);

    this->RunServer();

    // Level1 should be loaded
    EXPECT_EQ(1, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_2"));

    // Move performer1 to level1's buffer
    _perf1.SetPose(level1BufferPose);
    this->RunServer();

    // Level1 should still be loaded
    EXPECT_EQ(1, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_1"));
    // Level2 should remain unloaded
    EXPECT_EQ(0, std::count(this->loadedModels.begin(),
                            this->loadedModels.end(), "tile_2"));
  };

  testSequence(perf1, perf2);
  testSequence(perf2, perf1);
}
