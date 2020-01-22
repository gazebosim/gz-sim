/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <ignition/common/Console.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Model.hh"

#include "ignition/gazebo/test_config.hh"
#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;

class BuoyancyTest : public ::testing::Test
{
  // Documentation inherited
  protected: void SetUp() override
  {
    ignition::common::Console::SetVerbosity(4);
    setenv("IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
           (std::string(PROJECT_BINARY_PATH) + "/lib").c_str(), 1);
  }
};

class Relay
{
  public: Relay()
  {
    auto plugin = loader.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem =
        dynamic_cast<MockSystem *>(systemPtr->QueryInterface<System>());
    EXPECT_NE(nullptr, this->mockSystem);
  }

  public: Relay &OnPreUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->preUpdateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnUpdate(MockSystem::CallbackType _cb)
  {
    this->mockSystem->updateCallback = std::move(_cb);
    return *this;
  }

  public: Relay &OnPostUpdate(MockSystem::CallbackTypeConst _cb)
  {
    this->mockSystem->postUpdateCallback = std::move(_cb);
    return *this;
  }

  public: SystemPluginPtr systemPtr;

  private: SystemLoader loader;
  private: MockSystem *mockSystem;
};



/////////////////////////////////////////////////
TEST_F(BuoyancyTest, Movement)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/buoyancy.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  std::size_t iterations = 1000;

  bool finished = false;
  Relay testSystem;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Check pose
    Entity submarine = _ecm.EntityByComponents(
        components::Model(), components::Name("submarine"));

    Entity submarineSinking = _ecm.EntityByComponents(
        components::Model(), components::Name("submarine_sinking"));

    Entity submarineBuoyant = _ecm.EntityByComponents(
        components::Model(), components::Name("submarine_buoyant"));

    ASSERT_NE(submarine, kNullEntity);
    ASSERT_NE(submarineSinking, kNullEntity);
    ASSERT_NE(submarineBuoyant, kNullEntity);

    auto submarinePose = _ecm.Component<components::Pose>(submarine);
    ASSERT_NE(submarinePose , nullptr);

    auto submarineSinkingPose = _ecm.Component<components::Pose>(
        submarineSinking);
    ASSERT_NE(submarineSinkingPose , nullptr);

    auto submarineBuoyantPose = _ecm.Component<components::Pose>(
        submarineBuoyant);
    ASSERT_NE(submarineSinkingPose , nullptr);

    // The "submarine" should stay in its starting location of 0, 0, 1.5 meters.
    EXPECT_NEAR(0, submarinePose->Data().Pos().X(), 1e-2);
    EXPECT_NEAR(0, submarinePose->Data().Pos().Y(), 1e-2);
    EXPECT_NEAR(0, submarinePose->Data().Pos().Z(), 1e-2);

    if (_info.iterations > 10)
    {
      EXPECT_LT(submarineSinkingPose->Data().Pos().Z(),
                submarinePose->Data().Pos().Z());
      EXPECT_GT(submarineBuoyantPose->Data().Pos().Z(),
                submarinePose->Data().Pos().Z());
    }

    if (_info.iterations == iterations)
    {
      EXPECT_NEAR(-1.63, submarineSinkingPose->Data().Pos().Z(), 1e-2);
      EXPECT_NEAR(4.90, submarineBuoyantPose->Data().Pos().Z(), 1e-2);
      finished = true;
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, iterations, false);
  EXPECT_TRUE(finished);
}
