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

#include "ignition/gazebo/Entity.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/ParticleEmitter.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/test_config.hh"

#include "helpers/Relay.hh"
#include "helpers/UniqueTestDirectoryEnv.hh"

using namespace ignition;
using namespace gazebo;

class ParticleEmitterTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
  public: void LoadWorld(const std::string &_path, bool _useLevels = false)
  {
    this->serverConfig.SetResourceCache(test::UniqueTestDirectoryEnv::Path());
    this->serverConfig.SetSdfFile(
        common::joinPaths(PROJECT_SOURCE_PATH, _path));
    this->serverConfig.SetUseLevels(_useLevels);

    this->server = std::make_unique<Server>(this->serverConfig);
    EXPECT_FALSE(this->server->Running());
    EXPECT_FALSE(*this->server->Running(0));
    using namespace std::chrono_literals;
    this->server->SetUpdatePeriod(1ns);
  }

  public: ServerConfig serverConfig;
  public: std::unique_ptr<Server> server;
};

/////////////////////////////////////////////////
// Load an SDF with a particle emitter and verify its properties.
TEST_F(ParticleEmitterTest, SDFLoad)
{
  const ignition::math::Pose3d expectedPose(0, 0, 0, 0, 0, 0);
  bool updateChecked{false};

  this->LoadWorld("test/worlds/particle_emitter.sdf");

  // Create a system that checks a particle emitter.
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &,
                              const gazebo::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::ParticleEmitter,
                  components::Name,
                  components::Pose>(
            [&](const ignition::gazebo::Entity &/*_entity*/,
                const components::ParticleEmitter *_emitter,
                const components::Name *_name,
                const components::Pose *_pose) -> bool
            {
              updateChecked = true;

              EXPECT_EQ("particle_emitter_smoke_emitter", _name->Data());
              EXPECT_EQ(expectedPose, _pose->Data());
              EXPECT_TRUE(_emitter->Data().data.emitting());
              EXPECT_DOUBLE_EQ(2, _emitter->Data().data.lifetime());
              EXPECT_DOUBLE_EQ(10, _emitter->Data().data.min_velocity());
              EXPECT_DOUBLE_EQ(20, _emitter->Data().data.max_velocity());
              EXPECT_DOUBLE_EQ(10, _emitter->Data().data.scale_rate());

              return true;
            });
      });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, 1, false);

  EXPECT_TRUE(updateChecked);
}

/////////////////////////////////////////////////
/// Main
int main(int _argc, char **_argv)
{
  ::testing::InitGoogleTest(&_argc, _argv);
  ::testing::AddGlobalTestEnvironment(
      new test::UniqueTestDirectoryEnv("particle_emitter_test_cache"));
  return RUN_ALL_TESTS();
}
