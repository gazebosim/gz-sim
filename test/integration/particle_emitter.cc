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

#include <string>

#include <gz/common/Util.hh>
#include <gz/math/Color.hh>
#include <gz/msgs/Utility.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParticleEmitter.hh"
#include "gz/sim/components/Pose.hh"
#include "test_config.hh"

#include "helpers/EnvTestFixture.hh"
#include "helpers/Relay.hh"

using namespace gz;
using namespace sim;

class ParticleEmitterTest : public InternalFixture<::testing::Test>
{
  public: void LoadWorld(const std::string &_path, bool _useLevels = false)
  {
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
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(ParticleEmitterTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SDFLoad))
{
  bool updateCustomChecked{false};
  bool updateDefaultChecked{false};

  this->LoadWorld("test/worlds/particle_emitter.sdf");

  // Create a system that checks a particle emitter.
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const sim::UpdateInfo &,
                              const sim::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::ParticleEmitter,
                  components::Name,
                  components::Pose>(
            [&](const gz::sim::Entity &,
                const components::ParticleEmitter *_emitter,
                const components::Name *_name,
                const components::Pose *_pose) -> bool
            {
              if (_name->Data() == "smoke_emitter")
              {
                updateCustomChecked = true;

                EXPECT_EQ("smoke_emitter", _name->Data());
                EXPECT_EQ(_name->Data(), _emitter->Data().name());
                EXPECT_EQ(msgs::ParticleEmitter_EmitterType_BOX,
                    _emitter->Data().type());
                EXPECT_EQ(math::Pose3d(0, 1, 0, 0, 0, 0), _pose->Data());
                EXPECT_EQ(_pose->Data(),
                    msgs::Convert(_emitter->Data().pose()));
                EXPECT_EQ(math::Vector3d(2, 2, 2),
                    msgs::Convert(_emitter->Data().size()));
                EXPECT_DOUBLE_EQ(5.0, _emitter->Data().rate().data());
                EXPECT_DOUBLE_EQ(1.0, _emitter->Data().duration().data());
                EXPECT_TRUE(_emitter->Data().emitting().data());
                EXPECT_EQ(math::Vector3d(3, 3, 3),
                    msgs::Convert(_emitter->Data().particle_size()));
                EXPECT_DOUBLE_EQ(2.0, _emitter->Data().lifetime().data());
                // TODO(anyone) add material check here
                EXPECT_DOUBLE_EQ(10.0,
                    _emitter->Data().min_velocity().data());
                EXPECT_DOUBLE_EQ(20.0,
                    _emitter->Data().max_velocity().data());
                EXPECT_EQ(math::Color::Blue,
                    msgs::Convert(_emitter->Data().color_start()));
                EXPECT_EQ(math::Color::Green,
                    msgs::Convert(_emitter->Data().color_end()));
                EXPECT_DOUBLE_EQ(10.0, _emitter->Data().scale_rate().data());

                // color range image is empty because the emitter system
                // will not be able to find a file that does not exist
                EXPECT_EQ("/path/to/dummy_image.png",
                    _emitter->Data().color_range_image().data());

                EXPECT_TRUE(_emitter->Data().has_particle_scatter_ratio());
                EXPECT_FLOAT_EQ(0.01f,
                    _emitter->Data().particle_scatter_ratio().data());
              }
              else
              {
                updateDefaultChecked = true;

                EXPECT_EQ("smoke_generator", _name->Data());
                EXPECT_EQ(msgs::ParticleEmitter_EmitterType_POINT,
                    _emitter->Data().type());
                EXPECT_EQ(math::Pose3d(0, 0, 0, 0, 0, 0), _pose->Data());
                EXPECT_EQ(_pose->Data(),
                    msgs::Convert(_emitter->Data().pose()));
                EXPECT_EQ(math::Vector3d(1, 1, 1),
                    msgs::Convert(_emitter->Data().size()));
                EXPECT_DOUBLE_EQ(10.0, _emitter->Data().rate().data());
                EXPECT_DOUBLE_EQ(0.0, _emitter->Data().duration().data());
                EXPECT_TRUE(_emitter->Data().emitting().data());
                EXPECT_EQ(math::Vector3d(1, 1, 1),
                    msgs::Convert(_emitter->Data().particle_size()));
                EXPECT_DOUBLE_EQ(5.0, _emitter->Data().lifetime().data());
                // TODO(anyone) add material check here
                EXPECT_DOUBLE_EQ(1.0, _emitter->Data().min_velocity().data());
                EXPECT_DOUBLE_EQ(1.0, _emitter->Data().max_velocity().data());
                EXPECT_EQ(math::Color::White,
                    msgs::Convert(_emitter->Data().color_start()));
                EXPECT_EQ(math::Color::White,
                    msgs::Convert(_emitter->Data().color_end()));
                EXPECT_DOUBLE_EQ(0.0, _emitter->Data().scale_rate().data());
                EXPECT_EQ("", _emitter->Data().color_range_image().data());
              }

              return true;
            });
      });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, 1, false);

  EXPECT_TRUE(updateCustomChecked);
  EXPECT_TRUE(updateDefaultChecked);
}
