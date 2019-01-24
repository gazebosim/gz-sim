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
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

#define tol 10e-6

using namespace ignition;
using namespace gazebo;

/// \brief Test DiffDrive system
class DiffDriveTest : public ::testing::TestWithParam<int>
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
    auto plugin = loader.LoadPlugin("libMockSystem.so",
                                "ignition::gazebo::MockSystem",
                                nullptr);
    EXPECT_TRUE(plugin.has_value());

    this->systemPtr = plugin.value();

    this->mockSystem = static_cast<MockSystem *>(
        systemPtr->QueryInterface<System>());
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
TEST_P(DiffDriveTest, PublishCmd)
{
  // Start server
  ServerConfig serverConfig;
  serverConfig.SetSdfFile(std::string(PROJECT_SOURCE_PATH) +
      "/test/worlds/diff_drive.sdf");

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  // Create a system that records the vehicle poses
  Relay testSystem;

  std::vector<math::Pose3d> poses;
  testSystem.OnPostUpdate([&poses](const gazebo::UpdateInfo &,
    const gazebo::EntityComponentManager &_ecm)
    {
      auto id = _ecm.EntityByComponents(
        components::Model(),
        components::Name("vehicle"));
      EXPECT_NE(kNullEntity, id);

      auto poseComp = _ecm.Component<components::Pose>(id);
      ASSERT_NE(nullptr, poseComp);

      poses.push_back(poseComp->Data());
    });
  server.AddSystem(testSystem.systemPtr);

  // Run server and check that vehicle didn't move
  server.Run(true, 1000, false);

  EXPECT_EQ(1000u, poses.size());

  for (const auto &pose : poses)
  {
    EXPECT_EQ(poses[0], pose);
  }

  // Publish command and check that vehicle moved
  transport::Node node;
  auto pub = node.Advertise<msgs::Twist>("/model/vehicle/cmd_vel");

  msgs::Twist msg;
  msgs::Set(msg.mutable_linear(), math::Vector3d(0.5, 0, 0));
  msgs::Set(msg.mutable_angular(), math::Vector3d(0.0, 0, 0.2));

  pub.Publish(msg);

  server.Run(true, 3000, false);

  EXPECT_EQ(4000u, poses.size());

  EXPECT_LT(poses[0].Pos().X(), poses[3999].Pos().X());
  EXPECT_LT(poses[0].Pos().Y(), poses[3999].Pos().Y());
  EXPECT_NEAR(poses[0].Pos().Z(), poses[3999].Pos().Z(), tol);
  EXPECT_NEAR(poses[0].Rot().X(), poses[3999].Rot().X(), tol);
  EXPECT_NEAR(poses[0].Rot().Y(), poses[3999].Rot().Y(), tol);
  EXPECT_LT(poses[0].Rot().Z(), poses[3999].Rot().Z());
}

// Run multiple times
INSTANTIATE_TEST_CASE_P(ServerRepeat, DiffDriveTest,
    ::testing::Range(1, 2));
