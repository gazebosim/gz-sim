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
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <chrono>
#include <gz/common/Filesystem.hh>
#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>
#include <optional>
#include <string>

#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "helpers/EnvTestFixture.hh"
#include "plugins/MockSystem.hh"

using namespace gz;
namespace components = gz::sim::components;

class PythonSystemLoaderTest : public InternalFixture<::testing::Test>
{
};

/////////////////////////////////////////////////
void worldReset()
{
  gz::msgs::WorldControl req;
  gz::msgs::Boolean rep;
  req.mutable_reset()->set_all(true);
  transport::Node node;

  unsigned int timeout = 1000;
  bool result;
  bool executed =
    node.Request("/world/default/control", req, timeout, rep, result);

  ASSERT_TRUE(executed);
  ASSERT_TRUE(result);
  ASSERT_TRUE(rep.data());
}

/////////////////////////////////////////////////
TEST_F(PythonSystemLoaderTest,
       GZ_UTILS_TEST_DISABLED_ON_WIN32(LoadMultipleSystems))
{
  common::setenv("GZ_SIM_SYSTEM_PLUGIN_PATH",
                 common::joinPaths(std::string(PROJECT_SOURCE_PATH), "python",
                                   "test", "plugins"));

  sim::ServerConfig serverConfig;
  serverConfig.SetSdfFile(common::joinPaths(std::string(PROJECT_SOURCE_PATH),
      "test", "worlds", "python_system_loader.sdf"));

  sim::Server server(serverConfig);
  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::optional<math::Pose3d> postUpdateModelPose;
  // Create a system that adds a recreate component to models
  auto testSystem = std::make_shared<sim::MockSystem>();
  testSystem->postUpdateCallback =
      [&](const sim::UpdateInfo &, const sim::EntityComponentManager &_ecm)
  {
    auto testModel = _ecm.EntityByComponents(components::Model(),
        components::Name("box"));
    if (testModel != sim::kNullEntity)
    {
      postUpdateModelPose = sim::worldPose(testModel, _ecm);
    }
    return false;
  };
  server.AddSystem(testSystem);
  server.Run(true, 1, false);
  ASSERT_TRUE(postUpdateModelPose.has_value());
  EXPECT_EQ(math::Pose3d(0, 0, 10, 0, 0, 0), *postUpdateModelPose);

  server.RunOnce(true);
  std::optional<math::Pose3d> afterResetModelPose;
  // Since the order of reset callbacks may not be reliable, we'll use the
  // following PreUpdate call to record the model pose after reset. The test
  // system does not set the world pose after reset.
  testSystem->preUpdateCallback = (
      [&](const sim::UpdateInfo &, const sim::EntityComponentManager &_ecm)
      {
        auto testModel = _ecm.EntityByComponents(components::Model(),
                                                 components::Name("box"));
        if (testModel != sim::kNullEntity)
        {
          afterResetModelPose = sim::worldPose(testModel, _ecm);
        }
        return false;
      });

  worldReset();
  server.Run(true, 400, false);
  ASSERT_TRUE(afterResetModelPose.has_value());
  EXPECT_NEAR(10.0, afterResetModelPose->X(), 1e-3);
}
