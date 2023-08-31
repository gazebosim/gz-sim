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

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/world_control.pb.h>

#include <string>
#include <vector>

#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <gz/transport/Node.hh>
#include <gz/utils/ExtraTestMacros.hh>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/EventManager.hh"
#include "gz/sim/SdfEntityCreator.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/SystemLoader.hh"
#include "gz/sim/Types.hh"
#include "test_config.hh"

#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/World.hh"

#include "plugins/MockSystem.hh"
#include "../helpers/EnvTestFixture.hh"

using namespace gz;
using namespace sim;
using namespace std::chrono_literals;
namespace components = gz::sim::components;

//////////////////////////////////////////////////
class ResetFixture: public InternalFixture<::testing::Test>
{
  protected: void SetUp() override
  {
    InternalFixture::SetUp();

    sdf::Plugin sdfPlugin;
    sdfPlugin.SetName("gz::sim::MockSystem");
    sdfPlugin.SetFilename("MockSystem");
    auto plugin = sm.LoadPlugin(sdfPlugin);
    EXPECT_TRUE(plugin.has_value());
    this->systemPtr = plugin.value();
    this->mockSystem = static_cast<sim::MockSystem *>(
        systemPtr->QueryInterface<sim::System>());
  }

  public: gz::sim::SystemPluginPtr systemPtr;
  public: sim::MockSystem *mockSystem;

  private: sim::SystemLoader sm;
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
/// This test checks that that the physics system handles cases where entities
/// are removed and then added back
TEST_F(ResetFixture, GZ_UTILS_TEST_DISABLED_ON_WIN32(HandleReset))
{
  gz::sim::ServerConfig serverConfig;

  const std::string sdfFile = common::joinPaths(PROJECT_SOURCE_PATH,
    "test", "worlds", "reset.sdf");

  serverConfig.SetSdfFile(sdfFile);

  sdf::Root root;
  root.Load(sdfFile);
  sim::Server server(serverConfig);

  // A pointer to the ecm. This will be valid once we run the mock system
  sim::EntityComponentManager *ecm = nullptr;

  this->mockSystem->configureCallback =
    [&ecm](const Entity &,
           const std::shared_ptr<const sdf::Element> &,
           EntityComponentManager &_ecm,
           EventManager &)
    {
      ecm = &_ecm;
    };

  server.AddSystem(this->systemPtr);
  // Verify initial conditions of the world
  {
    ASSERT_NE(nullptr, ecm);
    auto entity = ecm->EntityByComponents(components::Name("box"));
    ASSERT_NE(kNullEntity, entity);
    auto poseComp = ecm->Component<components::Pose>(entity);
    ASSERT_NE(nullptr, poseComp);
    EXPECT_FLOAT_EQ(poseComp->Data().Z(), 5.0f);

    EXPECT_EQ(1u, this->mockSystem->configureCallCount);
    EXPECT_EQ(0u, this->mockSystem->resetCallCount);
    EXPECT_EQ(0u, this->mockSystem->preUpdateCallCount);
    EXPECT_EQ(0u, this->mockSystem->updateCallCount);
    EXPECT_EQ(0u, this->mockSystem->postUpdateCallCount);
  }

  // Run so that things will happen in the world
  // In this case, the box should fall some
  server.Run(true, 100, false);
  {
    auto entity = ecm->EntityByComponents(components::Name("box"));
    ASSERT_NE(kNullEntity, entity);
    auto poseComp = ecm->Component<components::Pose>(entity);
    ASSERT_NE(nullptr, poseComp);
    EXPECT_LT(poseComp->Data().Z(), 5.0f);

    EXPECT_EQ(1u, this->mockSystem->configureCallCount);
    EXPECT_EQ(0u, this->mockSystem->resetCallCount);
    EXPECT_EQ(100u, this->mockSystem->preUpdateCallCount);
    EXPECT_EQ(100u, this->mockSystem->updateCallCount);
    EXPECT_EQ(100u, this->mockSystem->postUpdateCallCount);
  }

  // Validate update info in the reset
  this->mockSystem->resetCallback =
    [](const sim::UpdateInfo &_info,
       sim::EntityComponentManager &)
    {
      EXPECT_EQ(0u, _info.iterations);
      EXPECT_EQ(std::chrono::steady_clock::duration{0}, _info.simTime);
    };

  // Send command to reset to initial state
  worldReset();

  // It takes two iterations for this to propagate,
  // the first is for the message to be received and internal state setup
  server.Run(true, 1, false);
  EXPECT_EQ(1u, this->mockSystem->configureCallCount);
  EXPECT_EQ(0u, this->mockSystem->resetCallCount);
  EXPECT_EQ(101u, this->mockSystem->preUpdateCallCount);
  EXPECT_EQ(101u, this->mockSystem->updateCallCount);
  EXPECT_EQ(101u, this->mockSystem->postUpdateCallCount);

  // The second iteration is where the reset actually occurs.
  server.Run(true, 1, false);
  {
    ASSERT_NE(nullptr, ecm);
    auto entity = ecm->EntityByComponents(components::Name("box"));
    ASSERT_NE(kNullEntity, entity);
    auto poseComp = ecm->Component<components::Pose>(entity);
    ASSERT_NE(nullptr, poseComp);
    EXPECT_FLOAT_EQ(poseComp->Data().Z(), 5.0f);

    EXPECT_EQ(1u, this->mockSystem->configureCallCount);
    EXPECT_EQ(1u, this->mockSystem->resetCallCount);

    // These should not increment, because only reset is called
    EXPECT_EQ(101u, this->mockSystem->preUpdateCallCount);
    EXPECT_EQ(101u, this->mockSystem->updateCallCount);
    EXPECT_EQ(101u, this->mockSystem->postUpdateCallCount);
  }

  server.Run(true, 100, false);
  {
    ASSERT_NE(nullptr, ecm);
    auto entity = ecm->EntityByComponents(components::Name("box"));
    ASSERT_NE(kNullEntity, entity);
    auto poseComp = ecm->Component<components::Pose>(entity);
    ASSERT_NE(nullptr, poseComp);
    EXPECT_LT(poseComp->Data().Z(), 5.0f);

    EXPECT_EQ(1u, this->mockSystem->configureCallCount);
    EXPECT_EQ(1u, this->mockSystem->resetCallCount);
    EXPECT_EQ(201u, this->mockSystem->preUpdateCallCount);
    EXPECT_EQ(201u, this->mockSystem->updateCallCount);
    EXPECT_EQ(201u, this->mockSystem->postUpdateCallCount);
  }
}
