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

#include <sdf/Root.hh>
#include <sdf/World.hh>

#include <ignition/common/Console.hh>
#include <ignition/transport/Node.hh>

#include "ignition/gazebo/Server.hh"
#include "ignition/gazebo/SystemLoader.hh"
#include "ignition/gazebo/components/Name.hh"
#include "ignition/gazebo/components/Pose.hh"
#include "ignition/gazebo/components/Model.hh"
#include "ignition/gazebo/test_config.hh"

#include "../helpers/Relay.hh"

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

  test::Relay testSystem;
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

  test::Relay testSystem;
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

  test::Relay testSystem;
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

/////////////////////////////////////////////////
// The test checks that breadcrumbs can be performers
TEST_F(BreadcrumbsTest, Performer)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/breadcrumbs.sdf";
  serverConfig.SetSdfFile(sdfFile);
  serverConfig.SetUseLevels(true);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deploy = node.Advertise<msgs::Empty>(
      "/model/vehicle_blue/breadcrumbs/B1_perf/deploy");

  const std::size_t iterTestStart = 1000;
  const std::size_t nIters = iterTestStart + 10000;

  std::optional<math::Pose3d> initialPose;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Deploy a performer breadcrumb on a tile that's on a level, and ensure
    // that it keeps the tile from being unloaded.

    // Deploy, move outside of the level, check
    if (_info.iterations == iterTestStart)
    {
      msgs::Twist msg;
      msg.mutable_linear()->set_x(2);
      cmdVel.Publish(msg);
      deploy.Publish(msgs::Empty());
    }
    else if (_info.iterations == iterTestStart + 1000)
    {
      // get the initial pose of the performer while the level is loaded
      _ecm.Each<components::Model, components::Name, components::Pose>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name, const components::Pose *_pose)
          {
            if (_name->Data().find("B1_perf") == 0)
            {
              initialPose = _pose->Data();
              return false;
            }
            return true;
          });
    }
    else if (_info.iterations == nIters)
    {
      math::Pose3d finalPose;
      // get the final pose of the performer
      _ecm.Each<components::Model, components::Name, components::Pose>(
          [&](const Entity &, const components::Model *,
              const components::Name *_name, const components::Pose *_pose)
          {
            if (_name->Data().find("B1_perf") == 0)
            {
              finalPose = _pose->Data();
              return false;
            }

            return true;
          });
      ASSERT_TRUE(initialPose.has_value());
      std::cout << "Init: " << initialPose->Pos() << " Final: "
                << finalPose.Pos() << std::endl;
      EXPECT_NEAR(initialPose->Pos().Z(), finalPose.Pos().Z(), 1e-3);
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, nIters, false);
}

/////////////////////////////////////////////////
// Test that the volume of the performer is set when deploying a performer
// breadcrumb
TEST_F(BreadcrumbsTest, PerformerSetVolume)
{
  // Start server
  ServerConfig serverConfig;
  const auto sdfFile = std::string(PROJECT_SOURCE_PATH) +
    "/test/worlds/breadcrumbs.sdf";
  serverConfig.SetSdfFile(sdfFile);
  serverConfig.SetUseLevels(true);

  Server server(serverConfig);
  EXPECT_FALSE(server.Running());
  EXPECT_FALSE(*server.Running(0));

  using namespace std::chrono_literals;
  server.SetUpdatePeriod(1ns);

  test::Relay testSystem;
  transport::Node node;
  auto deploy = node.Advertise<msgs::Empty>(
      "/model/vehicle_blue/breadcrumbs/B1_perf_large_volume/deploy");

  const std::size_t iterTestStart = 1000;
  const std::size_t nIters = iterTestStart + 2000;

  std::optional<math::Pose3d> initialPose;
  testSystem.OnPostUpdate([&](const gazebo::UpdateInfo &_info,
                             const gazebo::EntityComponentManager &_ecm)
  {
    // Deploy a performer breadcrumb on a tile that's on the default a level,
    // and check that it causes tile_1 to be loaded since the performer's volume
    // is large enough to overlap with tile_1.

    // Set pose to tile_2 , check if tile_1 is still loaded
    if (_info.iterations == iterTestStart)
    {
      msgs::Pose req;
      req.set_name("vehicle_blue");
      req.mutable_position()->set_x(30);
      msgs::Boolean rep;
      bool result;
      node.Request("/world/breadcrumbs/set_pose", req, 2000, rep, result);
      EXPECT_TRUE(result);
    }
    else if (_info.iterations == iterTestStart + 500)
    {
      // Check that tile_1 is unloaded
      Entity tileEntity = _ecm.EntityByComponents(components::Model(),
                                                  components::Name("tile_1"));
      EXPECT_EQ(kNullEntity, tileEntity);

      deploy.Publish(msgs::Empty());
    }
    else if (_info.iterations == nIters)
    {
      Entity tileEntity = _ecm.EntityByComponents(components::Model(),
                                                  components::Name("tile_1"));
      EXPECT_NE(kNullEntity, tileEntity);
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, nIters, false);
}

/////////////////////////////////////////////////
// The test verifies breadcrumbs physics is disabled using disable_physics_time
TEST_F(BreadcrumbsTest, DeployDisablePhysics)
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

  test::Relay testSystem;
  transport::Node node;
  auto cmdVel = node.Advertise<msgs::Twist>("/model/vehicle_blue/cmd_vel");
  auto deployB2 =
      node.Advertise<msgs::Empty>("/model/vehicle_blue/breadcrumbs/B2/deploy");

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
      deployB2.Publish(msgs::Empty());
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
      Entity b2 = _ecm.EntityByComponents(components::Model(),
                                          components::Name("B2_0"));
      ASSERT_NE(b2, kNullEntity);
      auto poseB2 = _ecm.Component<components::Pose>(b2);

      ASSERT_NE(poseB2, nullptr);

      // check pose of breadcrumb
      auto poseDiff = poseVehicle->Data().Inverse() * poseB2->Data();
      EXPECT_NEAR(-2.2, poseDiff.Pos().X(), 1e-2);
      EXPECT_NEAR(0.0, poseDiff.Pos().Y(), 1e-2);

      // Verify that the breadcrumb stopped falling after 0.5s.
      sdf::Root root;
      root.Load(sdfFile);
      const sdf::World *world = root.WorldByIndex(0);
      double gz = world->Gravity().Z();
      double z0 = 2.0;
      double t = 0.5;
      double z = z0 + gz/2*t*t;
      EXPECT_NEAR(z, poseDiff.Pos().Z(), 1e-2);
    }
  });

  server.AddSystem(testSystem.systemPtr);
  server.Run(true, iterTestStart + 2001, false);
}

/////////////////////////////////////////////////
// The test verifies that if allow_renaming is true, the Breadcrumb system
// renames spawned models if a model with the same name exists.
TEST_F(BreadcrumbsTest, AllowRenaming)
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

  transport::Node node;
  auto deployB1 =
      node.Advertise<msgs::Empty>("/model/vehicle_blue/breadcrumbs/B1/deploy");
  auto noRenameDeploy =
      node.Advertise<msgs::Empty>("/no_rename_deploy");
  auto renameDeploy =
      node.Advertise<msgs::Empty>("/rename_deploy");

  server.Run(true, 1, false);
  deployB1.Publish(msgs::Empty());
  server.Run(true, 100, false);
  EXPECT_TRUE(server.HasEntity("B1_0"));

  // Deploying via "/no_rename_deploy" will try to spawn B1_0, but since the
  // model already exists, the spawn should fail.
  auto curEntityCount = server.EntityCount().value();
  noRenameDeploy.Publish(msgs::Empty());
  server.Run(true, 100, false);
  EXPECT_EQ(curEntityCount, server.EntityCount().value());

  // Deploying via "/rename_deploy" will try to spawn B1_0, but since the
  // model already exists, it will spawn B1_0_1 instead.
  renameDeploy.Publish(msgs::Empty());
  server.Run(true, 100, false);
  EXPECT_TRUE(server.HasEntity("B1_0_1"));
}
