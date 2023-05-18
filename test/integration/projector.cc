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

#include <gz/common/Util.hh>
#include <gz/math/Color.hh>
#include <gz/msgs/Utility.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Projector.hh"
#include "test_config.hh"

#include "helpers/EnvTestFixture.hh"
#include "helpers/Relay.hh"

using namespace gz;
using namespace sim;

class ProjectorTest : public InternalFixture<::testing::Test>
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
// Load an SDF with a projector and verify its properties.
// See https://github.com/gazebosim/gz-sim/issues/1175
TEST_F(ProjectorTest, GZ_UTILS_TEST_DISABLED_ON_WIN32(SDFLoad))
{
  bool projectorChecked{false};

  this->LoadWorld(common::joinPaths("test", "worlds", "projector.sdf"));

  // Create a system that checks a projector.
  test::Relay testSystem;
  testSystem.OnPostUpdate([&](const sim::UpdateInfo &,
                              const sim::EntityComponentManager &_ecm)
      {
        _ecm.Each<components::Projector,
                  components::Name,
                  components::Pose>(
            [&](const gz::sim::Entity &,
                const components::Projector *_projector,
                const components::Name *_name,
                const components::Pose *_pose) -> bool
            {
              if (_name->Data() == "projector")
              {
                projectorChecked = true;

                EXPECT_EQ("projector", _name->Data());
                EXPECT_EQ(_name->Data(), _projector->Data().Name());
                EXPECT_EQ(math::Pose3d(0, 1, 0, 0, 0, 0), _pose->Data());
                EXPECT_EQ(_pose->Data(), _projector->Data().RawPose());
                EXPECT_DOUBLE_EQ(2.0, _projector->Data().NearClip());
                EXPECT_DOUBLE_EQ(7.0, _projector->Data().FarClip());
                EXPECT_EQ(math::Angle(0.5), _projector->Data().HorizontalFov());
                EXPECT_EQ(0x01, _projector->Data().VisibilityFlags());
                EXPECT_EQ(common::joinPaths("path", "to", "dummy_image.png"),
                    _projector->Data().Texture());
              }
              return true;
            });
      });

  this->server->AddSystem(testSystem.systemPtr);
  this->server->Run(true, 1, false);

  EXPECT_TRUE(projectorChecked);
}
