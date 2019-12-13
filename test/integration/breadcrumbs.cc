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

#include <ignition/msgs/empty.pb.h>
#include <ignition/msgs/twist.pb.h>

#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/test_config.hh"

#include "plugins/MockSystem.hh"

using namespace ignition;
using namespace gazebo;

class BreadcrumbsTest : public ::testing::Test
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
// The test checks breadcrumbs are deployed at the correct pose
TEST_F(BreadcrumbsTest, DeployAtOffset)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/breadcrumbs.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deployB1 =
      node.Advertise<msgs::Empty>("/model/vehicle_blue/breadcrumbs/B1/deploy");

  std::size_t iterTestStart = 1000;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Start moving the vehicle
    // After 1000 iterations, stop the vehicle, spawn a breadcrumb
    // Check the pose of the breadcrumb
    if (_info.iterations == iterTestStart)
    {
      msgs::Twist msg;
      msg.mutable_linear()->set_x(0.5);
      cmdVel.Publish(msg);
    }
    else if (_info.iterations == iterTestStart + 1000)
    {
      // Stop vehicle
      cmdVel.Publish(msgs::Twist());
    }
    else if (_info.iterations == iterTestStart + 1200)
    {
      // Deploy
      deployB1.Publish(msgs::Empty());
    }
    else if (_info.iterations == iterTestStart + 2000)
    {
      // Check pose
      Entity vehicleBlue = _ecm.EntityByComponents(
          components::Model(), components::Name("vehicle_blue"));
      ASSERT_NE(vehicleBlue, kNullEntity);

      auto poseVehicle = _ecm.Component<components::Pose>(vehicleBlue);
      ASSERT_NE(poseVehicle , nullptr);

      EXPECT_GT(poseVehicle->Data().Pos().X(), 0.4);

      // The first breadcrumb
      Entity b1 = _ecm.EntityByComponents(components::Model(),
                                          components::Name("B1_0"));
      ASSERT_NE(b1, kNullEntity);
      auto poseB1 = _ecm.Component<components::Pose>(b1);

      ASSERT_NE(poseB1, nullptr);

      auto poseDiff = poseVehicle->Data().Inverse() * poseB1->Data();
      EXPECT_NEAR(poseDiff.Pos().X(), -1.2, 1e-2);
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, iterTestStart + 2001, false);
}

/////////////////////////////////////////////////
// The test checks max deployments
TEST_F(BreadcrumbsTest, MaxDeployments)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/breadcrumbs.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deployB1 =
      node.Advertise<msgs::Empty>("/model/vehicle_blue/breadcrumbs/B1/deploy");

  std::size_t iterTestStart = 1000;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Start moving the vehicle
    // Every 1000 iterations, deploy
    // After 5000 iterations, check the number of breadcrumbs deployed
    if (_info.iterations == iterTestStart)
    {
      msgs::Twist msg;
      msg.mutable_linear()->set_x(0.5);
      cmdVel.Publish(msg);
    }
    else if ((_info.iterations + iterTestStart + 1) % 1000 == 0)
    {
      // Deploy
      deployB1.Publish(msgs::Empty());
    }
    else if (_info.iterations == iterTestStart + 5000)
    {
      std::size_t breadCrumbCount{0};
      _ecm.Each<components::Model, components::Name>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name)
          {
            if (_name->Data().find("B1") == 0)
            {
              ++breadCrumbCount;
            }

            return true;
          });

      EXPECT_EQ(breadCrumbCount, 3u);
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, iterTestStart + 5001, false);
}

/////////////////////////////////////////////////
// The test checks that including models from fuel works. Also checks custom
// topic
TEST_F(BreadcrumbsTest, FuelDeploy)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/breadcrumbs.sdf";
  serverConfig.SetSdfFile(sdfFile);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deploy = node.Advertise<msgs::Empty>("/fuel_deploy");

  const std::size_t iterTestStart = 1000;
  const std::size_t nIters = iterTestStart + 2500;
  const std::size_t maxDeployments = 5;
  std::size_t deployCount = 0;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Start moving the vehicle
    // Every 500 iterations, deploy
    // After 3500 iterations, check the number of breadcrumbs deployed
    if ((_info.iterations + iterTestStart + 1) % 500 == 0)
    {
      if (deployCount < maxDeployments)
      {
        deploy.Publish(msgs::Empty());
        ++deployCount;
      }
    }
    else if (_info.iterations == nIters)
    {
      std::size_t breadCrumbCount{0};
      _ecm.Each<components::Model, components::Name>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name)
          {
            if (_name->Data().find("B2") == 0)
            {
              ++breadCrumbCount;
            }

            return true;
          });

      EXPECT_EQ(breadCrumbCount, deployCount);
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, nIters, false);
}
