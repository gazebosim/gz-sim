/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <optional>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "test_config.hh"  // NOLINT(build/include)

#include "gz/sim/components/Level.hh"
#include "gz/sim/components/LevelBuffer.hh"
#include "gz/sim/components/LevelEntityNames.hh"
#include "gz/sim/components/Light.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Performer.hh"
#include "gz/sim/components/PerformerLevels.hh"
#include "gz/sim/components/Pose.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/Relay.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;

//////////////////////////////////////////////////
/// \brief A system to move models to arbitrary poses. Note that this does not
/// work if the physics system is running.
class ModelMover: public test::Relay
{
  public: explicit ModelMover(Entity _entity): test::Relay(), entity(_entity)
  {
    using namespace std::placeholders;
    this->systemPtr->preUpdateCallback =
        std::bind(&ModelMover::MoveModel, this, _1, _2);
  }

  /// \brief Sets the pose of the entity
  /// \param[in] _pose Commanded pose
  public: void SetPose(math::Pose3d _pose)
  {
    poseCmd = std::move(_pose);
  }

  public: Entity Id() const
  {
    return entity;
  }

  /// \brief Sets the pose component of the entity to the commanded pose. This
  /// function meant to be called in the preupdate phase
  private: void MoveModel(const UpdateInfo &,
                          EntityComponentManager &_ecm)
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

//////////////////////////////////////////////////
class LevelManagerFixture : public InternalFixture<::testing::Test>
{
  // Documentation inherited
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    ServerConfig serverConfig;

    // Except tile_0, which is on the default level, every tile belongs to a
    // level. The name of the level corresponds to the tile in its suffix, i.e.,
    // level1 contains tile_1.
    serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
                            "/test/worlds/levels_no_performers.sdf");
    serverConfig.SetUseLevels(true);

    server = std::make_unique<Server>(serverConfig);

    // Add in the "box" performer using a service call
    transport::Node node;
    msgs::StringMsg req;
    msgs::Boolean rep;

    req.set_data("box");

    bool result;
    unsigned int timeout = 2000;
    bool executed = node.Request("/world/levels/level/set_performer",
        req, timeout, rep, result);
    EXPECT_TRUE(executed);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());

    req.set_data("sphere");
    executed = node.Request("/world/levels/level/set_performer",
        req, timeout, rep, result);
    EXPECT_TRUE(executed);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());

    // Attempt to set the same performer
    executed = node.Request("/world/levels/level/set_performer",
        req, timeout, rep, result);
    EXPECT_TRUE(executed);
    EXPECT_TRUE(result);
    EXPECT_TRUE(rep.data());

    test::Relay testSystem;
    // Check entities loaded on the default level
    testSystem.OnPostUpdate([&](const UpdateInfo &,
                            const EntityComponentManager &_ecm)
    {
      Entity sphere = _ecm.EntityByComponents(components::Name("sphere"));
      EXPECT_EQ(1u,
          _ecm.ChildrenByComponents(sphere, components::Performer()).size());
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

  public: std::unique_ptr<Server> server;
  public: std::vector<std::string> loadedModels;
  public: std::vector<std::string> unloadedModels;
  public: std::vector<std::string> loadedLights;
  public: std::map<Entity, std::set<Entity>> performerLevels;
};

/////////////////////////////////////////////////
/// Check default level includes entities not included by other levels
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(LevelManagerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(DefaultLevel))
{
  std::vector<std::set<std::string>> levelEntityNamesList;

  test::Relay recorder;
  // Check entities loaded on the default level
  recorder.OnPostUpdate([&](const UpdateInfo &,
                            const EntityComponentManager &_ecm)
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
TEST_F(LevelManagerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(LevelLoadUnload))
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
TEST_F(LevelManagerFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(LevelBuffers))
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
TEST_F(LevelManagerFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LevelsWithMultiplePerformers))
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
    gzdbg << "Testing performer1 [" << _perf1.Id() << "] and performer2 ["
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
TEST_F(LevelManagerFixture,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LevelBuffersWithMultiplePerformers))
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
    gzdbg << "Testing performer1 [" << _perf1.Id() << "] and performer2 ["
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
